#!/bin/bash
#
# USB端口检查脚本（v2 强化版）
# - 优先检测别名 /dev/{imu,encoder,cmd}_usb，再回退原始端口
# - 临时设置串口为 115200/8N1/raw 再读取
# - 滑动窗口搜索 0x55(0x51/52/53) 与 0xA5 帧，避免首字节误判
# - 支持 --slow 慢速模式
#
echo "=========================================="
echo "🔍 USB端口检查工具 (v2)"
echo "=========================================="
echo ""

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# 读取参数：默认/慢速
READ_ATTEMPTS=5
READ_TIMEOUT=2
HEX_TIMEOUT=3
TOTAL_READ_BYTES=1024
if [[ "$1" == "--slow" ]]; then
  READ_ATTEMPTS=8
  READ_TIMEOUT=4
  HEX_TIMEOUT=6
  TOTAL_READ_BYTES=2048
  shift
  echo "🔧 慢速模式：每端口最长等待 ≈ $((READ_ATTEMPTS*READ_TIMEOUT + HEX_TIMEOUT)) 秒"
  echo ""
fi

configure_serial_raw() {
  local dev="$1"
  stty -F "$dev" 115200 cs8 -cstopb -parenb -ixon -ixoff -icanon -echo -echoe -echok -echoctl -echoke -opost min 0 time 5 2>/dev/null || true
}

read_hex_window() {
  local dev="$1"
  local bytes="$2"
  local chunk_size=256
  local count=$(( (bytes + chunk_size - 1) / chunk_size ))
  dd if="$dev" bs=$chunk_size count=$count status=none 2>/dev/null | hexdump -v -e '1/1 "%02x "' 2>/dev/null
}

analyze_buffer() {
  local hexstr="$1"
  local has_imu=0
  local has_a5=0
  if echo " $hexstr " | grep -qiE ' 55 (51|52|53)'; then
    has_imu=1
  fi
  local a5count
  a5count=$(echo " $hexstr " | grep -o -i ' a5 ' | wc -l | tr -d ' ')
  if [ "$a5count" -ge 3 ]; then
    has_a5=1
  fi
  if [ "$has_imu" -eq 1 ]; then
    echo "IMU"
  elif [ "$has_a5" -eq 1 ]; then
    echo "ENCODER"
  elif [ -z "$hexstr" ] || [ "$(echo "$hexstr" | tr -d ' ' | wc -c)" -lt 2 ]; then
    echo "CMD"
  else
    echo "UNKNOWN"
  fi
}

echo "📋 1. 检查设备与别名"
echo "----------------------------------------"
for alias in /dev/imu_usb /dev/encoder_usb /dev/cmd_usb; do
  if [ -e "$alias" ] || [ -L "$alias" ]; then
    target=$(readlink -f "$alias" 2>/dev/null || echo "$alias")
    perms=$(ls -l "$target" 2>/dev/null | awk '{print $1, $3, $4}')
    echo -e "${GREEN}✅ $alias${NC}"
    [ "$target" != "$alias" ] && echo "   指向: $target"
    [ -n "$perms" ] && echo "   权限: $perms"
  else
    echo -e "${RED}❌ $alias (不存在)${NC}"
  fi
done
for port in /dev/ttyCH341USB{0..3}; do
  [ -e "$port" ] && echo -e "${GREEN}✅ $port${NC}"
done
echo ""

echo "📋 2. 占用情况"
echo "----------------------------------------"
check_busy() {
  local dev="$1"
  if [ -e "$dev" ] || [ -L "$dev" ]; then
    users=$(lsof "$dev" 2>/dev/null | grep -v COMMAND)
    if [ -z "$users" ]; then
      echo -e "${GREEN}✅ $dev: 未被占用${NC}"
    else
      echo -e "${YELLOW}⚠️  $dev: 被占用${NC}"
      echo "$users" | sed 's/^/   /'
    fi
  fi
}
for alias in /dev/imu_usb /dev/encoder_usb /dev/cmd_usb; do
  check_busy "$alias"
done
for port in /dev/ttyCH341USB{0..3}; do
  check_busy "$port"
done
echo ""

echo "📋 3. 自动识别功能（滑动窗口+原始模式）"
echo "----------------------------------------"
echo "正在读取数据并分析..."
echo ""

declare -A usb_function
declare -A usb_data_format

declare -a probe_devs=()
for alias in /dev/imu_usb /dev/encoder_usb /dev/cmd_usb; do
  [ -e "$alias" ] || [ -L "$alias" ] && probe_devs+=("$alias")
done
for port in /dev/ttyCH341USB{0..3}; do
  [ -e "$port" ] && probe_devs+=("$port")
done

for dev in "${probe_devs[@]}"; do
  label=$(basename "$dev")
  echo -e "${BLUE}检测 ${label} ($dev):${NC}"

  configure_serial_raw "$dev"

  hexbuf=""
  data_found=false
  for attempt in $(seq 1 "$READ_ATTEMPTS"); do
    chunk=$(timeout "$READ_TIMEOUT" bash -c "dd if=\"$dev\" bs=256 count=$((TOTAL_READ_BYTES/256)) status=none 2>/dev/null | hexdump -v -e '1/1 \"%02x \"' 2>/dev/null")
    if [ -n "$chunk" ]; then
      hexbuf="$hexbuf $chunk"
      data_found=true
    fi
    [ "$data_found" = true ] && break
    sleep 0.3
  done
  if [ "$data_found" = false ]; then
    line=$(timeout "$HEX_TIMEOUT" hexdump -C "$dev" 2>/dev/null | head -1 | awk '{$1=""; print}' | sed 's/|.*//g')
    hexbuf="$hexbuf $(echo "$line" | xargs -n1 | awk '{printf "%s ", $0}')"
  fi

  role=$(analyze_buffer "$hexbuf")
  case "$role" in
    "IMU")
      usb_function[$dev]="IMU数据端口 (imu_publisher)"
      usb_data_format[$dev]="检测到 0x55 0x51/52/53 帧头（滑窗）"
      echo -e "   数据格式: ${GREEN}${usb_data_format[$dev]}${NC}"
      echo -e "   功能识别: ${GREEN}✅ ${usb_function[$dev]}${NC}"
      ;;
    "ENCODER")
      usb_function[$dev]="编码器数据端口 (odom_publisher)"
      usb_data_format[$dev]="检测到多次 0xA5 帧头（滑窗）"
      echo -e "   数据格式: ${GREEN}${usb_data_format[$dev]}${NC}"
      echo -e "   功能识别: ${GREEN}✅ ${usb_function[$dev]}${NC}"
      ;;
    "CMD")
      usb_function[$dev]="速度指令端口 (serial_bridge)"
      usb_data_format[$dev]="无持续数据（空闲）"
      echo -e "   数据格式: ${GREEN}${usb_data_format[$dev]}${NC}"
      echo -e "   功能识别: ${GREEN}✅ ${usb_function[$dev]}${NC}"
      ;;
    *)
      first=$(echo "$hexbuf" | awk '{print $1}')
      usb_function[$dev]="未知功能"
      usb_data_format[$dev]="未知/未对齐（首字节: 0x${first})"
      echo -e "   数据格式: ${YELLOW}${usb_data_format[$dev]}${NC}"
      echo -e "   功能识别: ${YELLOW}⚠️  需要进一步确认${NC}"
      ;;
  esac
  echo ""
done

echo "=========================================="
echo "📊 识别结果"
echo "=========================================="
echo ""
for dev in "${!usb_function[@]}"; do
  alias_hint=""
  if [ "$dev" = "/dev/imu_usb" ]; then
    target=$(readlink -f /dev/imu_usb 2>/dev/null | xargs basename || true)
    alias_hint="[指向: ${target}]"
  elif [ "$dev" = "/dev/encoder_usb" ]; then
    target=$(readlink -f /dev/encoder_usb 2>/dev/null | xargs basename || true)
    alias_hint="[指向: ${target}]"
  elif [ "$dev" = "/dev/cmd_usb" ]; then
    target=$(readlink -f /dev/cmd_usb 2>/dev/null | xargs basename || true)
    alias_hint="[指向: ${target}]"
  fi
  echo -e "  ${GREEN}${dev}${NC} → ${GREEN}${usb_function[$dev]}${NC} ${alias_hint}"
done
echo ""
echo -e "  ${GREEN}✅ 建议始终使用稳定别名进行配置（不受USB编号变化影响）${NC}"
echo -e "     imu_publisher_node     → /dev/imu_usb"
echo -e "     odom_publisher_node    → /dev/encoder_usb"
echo -e "     serial_bridge_node     → /dev/cmd_usb"
echo ""
echo "检查完成！"
echo "=========================================="

exit 0

