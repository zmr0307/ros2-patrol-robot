#!/usr/bin/env bash
set -euo pipefail

RULE_SRC="/home/nvidia/robot_ws/imu_usb.rules"
RULE_DST="/etc/udev/rules.d/99-robot-usb.rules"

echo "准备安装 udev 规则："
echo "  源: ${RULE_SRC}"
echo "  目标: ${RULE_DST}"
echo
if [[ ! -f "${RULE_SRC}" ]]; then
  echo "未找到规则文件: ${RULE_SRC}"
  exit 1
fi

echo "需要sudo权限复制规则文件并重载udev。请执行："
echo "  sudo cp ${RULE_SRC} ${RULE_DST}"
echo "  sudo rm -f /etc/udev/rules.d/99-imu_usb.rules  # 移除旧版本，避免重复创建 /dev/imu_usb"
echo "  sudo udevadm control --reload"
echo "  sudo udevadm trigger"
echo
echo "完成后请验证别名是否存在："
echo "  ls -l /dev/imu_usb /dev/encoder_usb /dev/cmd_usb"
echo
echo "如需撤销："
echo "  sudo rm -f ${RULE_DST} && sudo udevadm control --reload && sudo udevadm trigger"
echo "  然后重新插拔相关USB设备"

