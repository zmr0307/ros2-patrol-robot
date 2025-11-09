#!/bin/bash
#
# USB端口检查脚本
# 功能：检查USB设备连接状态和每个端口的对应关系
# 用法：./check_usb_ports.sh
#

echo "=========================================="
echo "🔍 USB端口检查工具"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检查USB设备是否存在
echo "📋 1. 检查USB设备连接状态"
echo "----------------------------------------"
for port in /dev/ttyCH341USB{0..3}; do
    if [ -e "$port" ]; then
        perms=$(ls -l "$port" | awk '{print $1, $3, $4}')
        echo -e "${GREEN}✅ $port${NC}"
        echo "   权限: $perms"
    else
        echo -e "${RED}❌ $port (不存在)${NC}"
    fi
done
echo ""

# 检查USB设备占用情况
echo "📋 2. 检查USB设备占用情况"
echo "----------------------------------------"
for port in /dev/ttyCH341USB{0..3}; do
    if [ -e "$port" ]; then
        users=$(lsof "$port" 2>/dev/null | grep -v COMMAND)
        if [ -z "$users" ]; then
            echo -e "${GREEN}✅ $port: 未被占用${NC}"
        else
            echo -e "${YELLOW}⚠️  $port: 被占用${NC}"
            echo "$users" | sed 's/^/   /'
        fi
    fi
done
echo ""

# 检查USB设备数据格式（判断对应关系）
echo "📋 3. 检查USB端口数据格式（判断对应关系）"
echo "----------------------------------------"
echo "正在读取数据（3秒）..."
echo ""

# 检查USB0（应该是速度指令端口，通常无数据或很少数据）
if [ -e "/dev/ttyCH341USB0" ]; then
    echo -e "${BLUE}USB0 (/dev/ttyCH341USB0):${NC}"
    timeout 1 hexdump -C /dev/ttyCH341USB0 2>&1 | head -3 > /tmp/usb0_data.txt
    if [ -s /tmp/usb0_data.txt ]; then
        first_byte=$(timeout 1 hexdump -C /dev/ttyCH341USB0 2>&1 | head -1 | awk '{print $2}')
        if [ "$first_byte" = "a5" ]; then
            echo -e "   数据格式: ${YELLOW}0xA5开头 (可能是编码器数据)${NC}"
        elif [ "$first_byte" = "55" ]; then
            echo -e "   数据格式: ${YELLOW}0x55开头 (IMU数据)${NC}"
        else
            echo -e "   数据格式: ${GREEN}无固定格式 (可能是速度指令端口，空闲)${NC}"
        fi
        echo -e "   判断: ${GREEN}✅ 速度指令端口 (serial_bridge)${NC}"
    else
        echo -e "   数据格式: ${GREEN}无数据 (空闲)${NC}"
        echo -e "   判断: ${GREEN}✅ 速度指令端口 (serial_bridge)${NC}"
    fi
    echo ""
fi

# 检查USB1（应该是IMU数据，0x55开头）
if [ -e "/dev/ttyCH341USB1" ]; then
    echo -e "${BLUE}USB1 (/dev/ttyCH341USB1):${NC}"
    # 使用cat读取，更可靠
    timeout 2 cat /dev/ttyCH341USB1 2>&1 | od -An -tx1 | head -1 > /tmp/usb1_data.txt 2>&1
    if [ -s /tmp/usb1_data.txt ]; then
        first_byte=$(cat /tmp/usb1_data.txt | awk '{print $1}')
        if [ "$first_byte" = "55" ]; then
            echo -e "   数据格式: ${GREEN}0x55开头 (JY901B IMU格式)${NC}"
            echo -e "   数据示例: $(timeout 1 cat /dev/ttyCH341USB1 2>&1 | od -An -tx1 | head -1 | cut -c1-30)"
            echo -e "   判断: ${GREEN}✅ IMU数据端口 (imu_publisher)${NC}"
        elif [ "$first_byte" = "a5" ]; then
            echo -e "   数据格式: ${YELLOW}0xA5开头 (编码器格式)${NC}"
            echo -e "   判断: ${RED}❌ 错误！应该是IMU端口${NC}"
        else
            echo -e "   数据格式: ${YELLOW}未知格式 (首字节: 0x$first_byte)${NC}"
            echo -e "   判断: ${YELLOW}⚠️  需要进一步确认${NC}"
        fi
    else
        echo -e "   数据格式: ${RED}无数据${NC}"
        echo -e "   判断: ${YELLOW}⚠️  IMU可能未连接或未启动${NC}"
    fi
    echo ""
fi

# 检查USB2（应该是编码器数据，0xA5开头）
if [ -e "/dev/ttyCH341USB2" ]; then
    echo -e "${BLUE}USB2 (/dev/ttyCH341USB2):${NC}"
    # 使用cat读取，更可靠
    timeout 2 cat /dev/ttyCH341USB2 2>&1 | od -An -tx1 | head -1 > /tmp/usb2_data.txt 2>&1
    if [ -s /tmp/usb2_data.txt ]; then
        first_byte=$(cat /tmp/usb2_data.txt | awk '{print $1}')
        if [ "$first_byte" = "a5" ]; then
            echo -e "   数据格式: ${GREEN}0xA5开头 (STM32编码器格式)${NC}"
            echo -e "   数据示例: $(timeout 1 cat /dev/ttyCH341USB2 2>&1 | od -An -tx1 | head -1 | cut -c1-30)"
            echo -e "   判断: ${GREEN}✅ 编码器数据端口 (odom_publisher)${NC}"
        elif [ "$first_byte" = "55" ]; then
            echo -e "   数据格式: ${YELLOW}0x55开头 (IMU格式)${NC}"
            echo -e "   判断: ${RED}❌ 错误！应该是编码器端口${NC}"
        else
            echo -e "   数据格式: ${YELLOW}未知格式 (首字节: 0x$first_byte)${NC}"
            echo -e "   判断: ${YELLOW}⚠️  需要进一步确认${NC}"
        fi
    else
        echo -e "   数据格式: ${RED}无数据${NC}"
        echo -e "   判断: ${YELLOW}⚠️  编码器可能未连接或未启动${NC}"
    fi
    echo ""
fi

# 检查USB3（通常不存在）
if [ -e "/dev/ttyCH341USB3" ]; then
    echo -e "${BLUE}USB3 (/dev/ttyCH341USB3):${NC}"
    echo -e "   状态: ${YELLOW}设备存在（但通常不使用）${NC}"
    echo ""
fi

# 总结
echo "=========================================="
echo "📊 端口对应关系总结"
echo "=========================================="
echo ""
echo "预期配置："
echo -e "  ${GREEN}USB0${NC} → serial_bridge_node (速度指令发送到STM32)"
echo -e "  ${GREEN}USB1${NC} → imu_publisher_node (IMU数据接收，0x55帧头)"
echo -e "  ${GREEN}USB2${NC} → odom_publisher_node (编码器数据接收，0xA5帧头)"
echo ""
echo "检查完成！"
echo "=========================================="

# 清理临时文件
rm -f /tmp/usb*_data.txt

