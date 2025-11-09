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

# 检查USB设备数据格式（自动识别功能）
echo "📋 3. 自动识别USB端口功能（通过数据格式）"
echo "----------------------------------------"
echo "正在读取数据并分析（每个端口约5秒，总共约20秒）..."
echo ""

# 存储检测结果
declare -A usb_function
declare -A usb_data_format

# 检查所有USB端口，自动识别功能
for port_num in 0 1 2 3; do
    port="/dev/ttyCH341USB${port_num}"
    if [ ! -e "$port" ]; then
        continue
    fi
    
    echo -e "${BLUE}检测 USB${port_num} ($port):${NC}"
    
    # 尝试读取数据（多次尝试以提高可靠性）
    data_found=false
    first_byte=""
    
    # 方法1: 使用od读取（增加读取时间和尝试次数）
    for attempt in 1 2 3 4 5; do
        timeout 2 cat "$port" 2>/dev/null | od -An -tx1 2>/dev/null | head -1 > /tmp/usb${port_num}_data.txt 2>&1
        if [ -s /tmp/usb${port_num}_data.txt ]; then
            first_byte=$(cat /tmp/usb${port_num}_data.txt | awk '{print $1}' | tr -d ' ')
            if [ -n "$first_byte" ] && [ "$first_byte" != "" ] && [ "$first_byte" != "0000000" ]; then
                data_found=true
                break
            fi
        fi
        sleep 0.2
    done
    
    # 方法2: 如果方法1失败，尝试hexdump（增加读取时间）
    if [ "$data_found" = false ]; then
        timeout 3 hexdump -C "$port" 2>/dev/null | head -1 > /tmp/usb${port_num}_hex.txt 2>&1
        if [ -s /tmp/usb${port_num}_hex.txt ]; then
            first_byte=$(cat /tmp/usb${port_num}_hex.txt | awk '{print $2}' | tr -d ' ')
            if [ -n "$first_byte" ] && [ "$first_byte" != "" ] && [ "$first_byte" != "00000000" ]; then
                data_found=true
            fi
        fi
    fi
    
    # 根据数据格式判断功能
    if [ "$data_found" = true ] && [ -n "$first_byte" ]; then
        first_byte_lower=$(echo "$first_byte" | tr '[:upper:]' '[:lower:]')
        
        if [ "$first_byte_lower" = "55" ]; then
            usb_function[$port_num]="IMU数据端口 (imu_publisher)"
            usb_data_format[$port_num]="0x55开头 (JY901B IMU格式)"
            echo -e "   数据格式: ${GREEN}${usb_data_format[$port_num]}${NC}"
            echo -e "   功能识别: ${GREEN}✅ ${usb_function[$port_num]}${NC}"
        elif [ "$first_byte_lower" = "a5" ]; then
            usb_function[$port_num]="编码器数据端口 (odom_publisher)"
            usb_data_format[$port_num]="0xA5开头 (STM32编码器格式)"
            echo -e "   数据格式: ${GREEN}${usb_data_format[$port_num]}${NC}"
            echo -e "   功能识别: ${GREEN}✅ ${usb_function[$port_num]}${NC}"
        else
            usb_function[$port_num]="未知功能"
            usb_data_format[$port_num]="未知格式 (首字节: 0x$first_byte)"
            echo -e "   数据格式: ${YELLOW}${usb_data_format[$port_num]}${NC}"
            echo -e "   功能识别: ${YELLOW}⚠️  需要进一步确认${NC}"
        fi
    else
        # 无数据，可能是速度指令端口（通常空闲）
        usb_function[$port_num]="速度指令端口 (serial_bridge)"
        usb_data_format[$port_num]="无数据 (空闲)"
        echo -e "   数据格式: ${GREEN}${usb_data_format[$port_num]}${NC}"
        echo -e "   功能识别: ${GREEN}✅ ${usb_function[$port_num]}${NC}"
    fi
    echo ""
done

# 总结
echo "=========================================="
echo "📊 USB端口功能识别结果"
echo "=========================================="
echo ""

# 显示自动识别的结果
echo "🔍 自动识别结果："
for port_num in 0 1 2 3; do
    port="/dev/ttyCH341USB${port_num}"
    if [ -e "$port" ]; then
        if [ -n "${usb_function[$port_num]}" ]; then
            # 检查是否符合预期
            expected=""
            case $port_num in
                0)
                    expected="速度指令端口"
                    ;;
                1)
                    expected="IMU数据端口"
                    ;;
                2)
                    expected="编码器数据端口"
                    ;;
            esac
            
            if [[ "${usb_function[$port_num]}" == *"$expected"* ]]; then
                echo -e "  ${GREEN}USB${port_num}${NC} → ${GREEN}${usb_function[$port_num]}${NC} ✅"
            else
                echo -e "  ${YELLOW}USB${port_num}${NC} → ${YELLOW}${usb_function[$port_num]}${NC} ⚠️  (可能与预期不符)"
            fi
        else
            echo -e "  ${YELLOW}USB${port_num}${NC} → ${YELLOW}未识别${NC}"
        fi
    fi
done
echo ""

# 检查配置建议
echo "💡 配置建议："
imu_port=""
encoder_port=""
speed_port=""

for port_num in 0 1 2 3; do
    if [ -n "${usb_function[$port_num]}" ]; then
        if [[ "${usb_function[$port_num]}" == *"IMU"* ]]; then
            imu_port="USB${port_num}"
        elif [[ "${usb_function[$port_num]}" == *"编码器"* ]]; then
            encoder_port="USB${port_num}"
        elif [[ "${usb_function[$port_num]}" == *"速度指令"* ]]; then
            speed_port="USB${port_num}"
        fi
    fi
done

if [ -n "$imu_port" ]; then
    imu_num=${imu_port#USB}
    echo -e "  ${GREEN}✅ IMU端口: ${imu_port}${NC} → 配置 imu_publisher_node 使用 /dev/ttyCH341USB${imu_num}"
fi
if [ -n "$encoder_port" ]; then
    encoder_num=${encoder_port#USB}
    echo -e "  ${GREEN}✅ 编码器端口: ${encoder_port}${NC} → 配置 odom_publisher_node 使用 /dev/ttyCH341USB${encoder_num}"
fi
if [ -n "$speed_port" ]; then
    speed_num=${speed_port#USB}
    echo -e "  ${GREEN}✅ 速度指令端口: ${speed_port}${NC} → 配置 serial_bridge_node 使用 /dev/ttyCH341USB${speed_num}"
fi

if [ -z "$imu_port" ] || [ -z "$encoder_port" ] || [ -z "$speed_port" ]; then
    echo -e "  ${YELLOW}⚠️  部分端口未识别，请检查设备连接${NC}"
fi

echo ""
echo "检查完成！"
echo "=========================================="

# 清理临时文件
rm -f /tmp/usb*_data.txt

