"""Shared-memory hand26 in / q_target out (EE process ↔ retarget process).

Layout is fixed-size numpy views over ``multiprocessing.shared_memory``.
SDK stays in the EE process; the worker never calls ``xrt.init()``.

Views are built **once** at construct/attach time; hot reads use
``np.frombuffer(...).copy()`` so we never re-wrap the buffer every tick.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from multiprocessing import shared_memory

import numpy as np

MAGIC = b"FAHD"
VERSION = 2  # bumped: MAX_DOF 16→24 (wuji/linker need ≥20)
N_JOINTS = 26
N_COLS = 8  # pad OpenXR rows to 8 floats
MAX_DOF = 24  # xhand1=12, wuji/linker=20; headroom for larger hands

# Header: magic(4) ver(u32) seq(u64) t(f64) flags(u32) la(i32) ra(i32)
_IN_HEADER_FMT = "<4sIQdIii"
_IN_HEADER_SIZE = struct.calcsize(_IN_HEADER_FMT)
_IN_JOINTS = N_JOINTS * N_COLS * 2  # left+right
_IN_BYTES = _IN_HEADER_SIZE + _IN_JOINTS * 8

# Out: magic ver seq t flags ldof rdof lheld rheld + q[2*MAX_DOF]
_OUT_HEADER_FMT = "<4sIQdIiiii"
_OUT_HEADER_SIZE = struct.calcsize(_OUT_HEADER_FMT)
_OUT_BYTES = _OUT_HEADER_SIZE + MAX_DOF * 2 * 8
# seq sits after magic(4)+ver(u32) → offset 8
_OUT_SEQ_OFFSET = 8

FLAG_LEFT_VALID = 1 << 0
FLAG_RIGHT_VALID = 1 << 1
FLAG_LIVE = 1 << 2  # FSM gate open + stream ready (hint; worker may skip)


@dataclass
class HandShmBundle:
    """Parent-owned shm pair; child attaches by name."""

    in_shm: shared_memory.SharedMemory
    out_shm: shared_memory.SharedMemory
    create: bool = True
    # Cached buffer views (filled in __post_init__).
    _in_joints: np.ndarray = field(init=False, repr=False)
    _out_qs: np.ndarray = field(init=False, repr=False)
    _in_buf: memoryview = field(init=False, repr=False)
    _out_buf: memoryview = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._in_buf = self.in_shm.buf
        self._out_buf = self.out_shm.buf
        # One-time ndarray wrappers over shm (writes go through these).
        self._in_joints = np.ndarray(
            (_IN_JOINTS,),
            dtype=np.float64,
            buffer=self._in_buf,
            offset=_IN_HEADER_SIZE,
        )
        self._out_qs = np.ndarray(
            (MAX_DOF * 2,),
            dtype=np.float64,
            buffer=self._out_buf,
            offset=_OUT_HEADER_SIZE,
        )

    @classmethod
    def create_pair(cls, *, prefix: str = "fa_hand") -> "HandShmBundle":
        in_shm = shared_memory.SharedMemory(create=True, size=_IN_BYTES, name=None)
        out_shm = shared_memory.SharedMemory(create=True, size=_OUT_BYTES, name=None)
        in_shm.buf[:4] = MAGIC
        out_shm.buf[:4] = MAGIC
        struct.pack_into("<I", in_shm.buf, 4, VERSION)
        struct.pack_into("<I", out_shm.buf, 4, VERSION)
        return cls(in_shm=in_shm, out_shm=out_shm, create=True)

    @classmethod
    def attach(cls, in_name: str, out_name: str) -> "HandShmBundle":
        return cls(
            in_shm=shared_memory.SharedMemory(name=in_name),
            out_shm=shared_memory.SharedMemory(name=out_name),
            create=False,
        )

    @property
    def in_name(self) -> str:
        return self.in_shm.name

    @property
    def out_name(self) -> str:
        return self.out_shm.name

    def close(self) -> None:
        for shm in (self.in_shm, self.out_shm):
            try:
                shm.close()
            except Exception:
                pass

    def unlink(self) -> None:
        if not self.create:
            return
        for shm in (self.in_shm, self.out_shm):
            try:
                shm.unlink()
            except Exception:
                pass

    def write_input(
        self,
        *,
        frame_seq: int,
        t_mono: float,
        live: bool,
        left: tuple[int, np.ndarray] | None,
        right: tuple[int, np.ndarray] | None,
    ) -> None:
        flags = 0
        if live:
            flags |= FLAG_LIVE
        la = ra = 0
        joints = self._in_joints
        joints.fill(0.0)
        if left is not None:
            flags |= FLAG_LEFT_VALID
            la = int(left[0])
            _pack_joints26(joints[: N_JOINTS * N_COLS], left[1])
        if right is not None:
            flags |= FLAG_RIGHT_VALID
            ra = int(right[0])
            _pack_joints26(joints[N_JOINTS * N_COLS :], right[1])
        struct.pack_into(
            _IN_HEADER_FMT,
            self._in_buf,
            0,
            MAGIC,
            VERSION,
            int(frame_seq) & 0xFFFFFFFFFFFFFFFF,
            float(t_mono),
            int(flags),
            int(la),
            int(ra),
        )

    def read_input(self) -> dict:
        magic, ver, seq, t_mono, flags, la, ra = struct.unpack_from(
            _IN_HEADER_FMT, self._in_buf, 0
        )
        if magic != MAGIC or int(ver) != VERSION:
            raise RuntimeError(f"bad hand shm input header magic={magic!r} ver={ver}")
        # Snapshot floats once; do not keep a live view across the GIL.
        joints = np.frombuffer(
            self._in_buf, dtype=np.float64, count=_IN_JOINTS, offset=_IN_HEADER_SIZE
        ).copy()
        left = joints[: N_JOINTS * N_COLS].reshape(N_JOINTS, N_COLS)
        right = joints[N_JOINTS * N_COLS :].reshape(N_JOINTS, N_COLS)
        return {
            "frame_seq": int(seq),
            "t_mono": float(t_mono),
            "flags": int(flags),
            "live": bool(int(flags) & FLAG_LIVE),
            "left_valid": bool(int(flags) & FLAG_LEFT_VALID),
            "right_valid": bool(int(flags) & FLAG_RIGHT_VALID),
            "left_active": int(la),
            "right_active": int(ra),
            "left_joints": left,
            "right_joints": right,
        }

    def write_output(
        self,
        *,
        frame_seq: int,
        t_mono: float,
        left_q: np.ndarray | None,
        right_q: np.ndarray | None,
        left_held: bool = False,
        right_held: bool = False,
    ) -> None:
        flags = 0
        ldof = rdof = 0
        qs = self._out_qs
        qs.fill(0.0)
        if left_q is not None:
            flags |= FLAG_LEFT_VALID
            q = np.asarray(left_q, dtype=np.float64).reshape(-1)
            ldof = int(min(q.size, MAX_DOF))
            qs[:ldof] = q[:ldof]
        if right_q is not None:
            flags |= FLAG_RIGHT_VALID
            q = np.asarray(right_q, dtype=np.float64).reshape(-1)
            rdof = int(min(q.size, MAX_DOF))
            qs[MAX_DOF : MAX_DOF + rdof] = q[:rdof]
        struct.pack_into(
            _OUT_HEADER_FMT,
            self._out_buf,
            0,
            MAGIC,
            VERSION,
            int(frame_seq) & 0xFFFFFFFFFFFFFFFF,
            float(t_mono),
            int(flags),
            int(ldof),
            int(rdof),
            1 if left_held else 0,
            1 if right_held else 0,
        )

    def peek_out_seq(self) -> int:
        """Cheap seq read without copying joint payload."""
        return int(struct.unpack_from("<Q", self._out_buf, _OUT_SEQ_OFFSET)[0])

    def read_output(self) -> dict | None:
        magic, ver, seq, t_mono, flags, ldof, rdof, lheld, rheld = struct.unpack_from(
            _OUT_HEADER_FMT, self._out_buf, 0
        )
        if magic != MAGIC:
            return None
        if int(ver) != VERSION:
            raise RuntimeError(f"bad hand shm output version {ver}")
        qs = np.frombuffer(
            self._out_buf, dtype=np.float64, count=MAX_DOF * 2, offset=_OUT_HEADER_SIZE
        ).copy()
        left_q = right_q = None
        if int(flags) & FLAG_LEFT_VALID and ldof > 0:
            left_q = qs[: int(ldof)].copy()
        if int(flags) & FLAG_RIGHT_VALID and rdof > 0:
            right_q = qs[MAX_DOF : MAX_DOF + int(rdof)].copy()
        return {
            "frame_seq": int(seq),
            "t_mono": float(t_mono),
            "left_q": left_q,
            "right_q": right_q,
            "left_held": bool(lheld),
            "right_held": bool(rheld),
        }


def _pack_joints26(dst_flat: np.ndarray, joints: np.ndarray) -> None:
    j = np.asarray(joints, dtype=np.float64)
    if j.ndim != 2 or j.shape[0] != N_JOINTS:
        j = j.reshape(N_JOINTS, -1)
    cols = min(int(j.shape[1]), N_COLS)
    view = dst_flat.reshape(N_JOINTS, N_COLS)
    view.fill(0.0)
    view[:, :cols] = j[:, :cols]
