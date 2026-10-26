# Local XR / dexterous-hand config for ./run.sh vr-xrt
# Sourced by run.sh after ROOT_DIR is set.
#
# 换手：改 configs/local.yaml 的 hands.type / hands.method
# 优先级：环境变量 FA_HAND / FA_HAND_CONFIG / FA_HAND_RETARGETING > local.yaml
#
# XHand1 method 见 local.yaml.example（dexpilot 捏合最好；curl 要采食指侧角）
# LinkerHand 重采：停 vr-xrt 后 python -m xr_hand_retarget.calibrate --config $FA_HAND_CONFIG

FA_XRT_TELEOP_CONFIG="${FA_XRT_TELEOP_CONFIG:-$ROOT_DIR/vr_pose_publisher/configs/xrt_teleop.yaml}"

_fa_local_fields() {
  local f="$ROOT_DIR/configs/local.yaml"
  [[ -f "$f" ]] || return 1
  python3 - "$f" <<'PY'
import re, sys
from pathlib import Path
text = Path(sys.argv[1]).read_text(encoding="utf-8")
block = text
m = re.search(r"(?m)^hands:\s*$", text)
if m:
    lines = []
    for line in text[m.end():].splitlines():
        if line and not line[0].isspace() and not line.startswith("#"):
            break
        lines.append(line)
    block = "\n".join(lines)

def pick(src, key):
    mm = re.search(rf"(?m)^\s*{key}:\s*([A-Za-z0-9_+-]+)", src)
    return mm.group(1).strip().lower() if mm else ""

typ = pick(block, "type") or pick(text, "type")
method = pick(block, "method") or pick(text, "method") or pick(block, "方法")
print(typ)
print(method)
PY
}

_FA_LOCAL_TYPE=""
_FA_LOCAL_METHOD=""
if _fa_local="$( _fa_local_fields || true )" && [[ -n "${_fa_local}" ]]; then
  _FA_LOCAL_TYPE="$(printf '%s\n' "$_fa_local" | sed -n '1p')"
  _FA_LOCAL_METHOD="$(printf '%s\n' "$_fa_local" | sed -n '2p')"
fi

if [[ -z "${FA_HAND:-}" ]]; then
  FA_HAND="${_FA_LOCAL_TYPE:-o6}"
fi

case "${FA_HAND}" in
  xhand1|xhand)
    _FA_HAND_YAML="$ROOT_DIR/xr_hand_retarget/configs/xhand1.yaml"
    ;;
  wuji|wuji_hand2|hand2)
    _FA_HAND_YAML="$ROOT_DIR/xr_hand_retarget/configs/wuji_hand2.yaml"
    ;;
  o6|linkerhand_o6|linker_o6)
    _FA_HAND_YAML="$ROOT_DIR/xr_hand_retarget/configs/o6.yaml"
    ;;
  l6|linkerhand_l6|linker_l6)
    _FA_HAND_YAML="$ROOT_DIR/xr_hand_retarget/configs/l6.yaml"
    ;;
  o7|linkerhand_o7|linker_o7)
    _FA_HAND_YAML="$ROOT_DIR/xr_hand_retarget/configs/o7.yaml"
    ;;
  *)
    echo "未知 FA_HAND=${FA_HAND}（应为 xhand1 | wuji | o6 | l6 | o7）"
    echo "可在 configs/local.yaml 设 hands.type，或 FA_HAND=o6|l6|o7 ./run.sh vr-xrt wrist"
    exit 1
    ;;
esac

FA_HAND_CONFIG="${FA_HAND_CONFIG:-$_FA_HAND_YAML}"
FA_XHAND1_CONFIG="${FA_HAND_CONFIG}"

# local.yaml method 覆盖包装 yaml 的 retargeting.type（除非环境里已设）
if [[ -z "${FA_HAND_RETARGETING:-}" && -n "${_FA_LOCAL_METHOD}" ]]; then
  FA_HAND_RETARGETING="${_FA_LOCAL_METHOD}"
fi
FA_HAND_RETARGETING="${FA_HAND_RETARGETING:-}"
FA_XHAND1_RETARGETING="${FA_HAND_RETARGETING}"
