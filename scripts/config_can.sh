#!/usr/bin/env bash

set -Eeuo pipefail

interface="${1:-can0}"
bitrate="${2:-1000000}"
txqueuelen="${3:-100}"

if ! command -v ip >/dev/null 2>&1; then
    echo "错误：未找到 ip 命令，请先安装 iproute2。" >&2
    exit 1
fi

if [[ ! "$interface" =~ ^[[:alnum:]_.:-]+$ ]]; then
    echo "错误：无效的 CAN 接口名：$interface" >&2
    exit 1
fi

if [[ ! "$bitrate" =~ ^[1-9][0-9]*$ ]]; then
    echo "错误：波特率必须是正整数：$bitrate" >&2
    exit 1
fi

if [[ ! -e "/sys/class/net/$interface" ]]; then
    echo "错误：网络接口 $interface 不存在。" >&2
    echo "可用接口：" >&2
    ip -brief link show >&2
    exit 1
fi

# Reuse an interface that is already up at the requested bitrate.  This keeps
# repeated launcher runs idempotent and avoids an unnecessary sudo prompt.
current_details="$(ip -details link show "$interface")"
if grep -q 'state UP' <<<"$current_details" \
    && grep -Eq "bitrate[[:space:]]+$bitrate([[:space:]]|$)" <<<"$current_details"; then
    echo "$interface 已处于 UP 状态，波特率为 $bitrate bit/s；无需重新配置。"
    ip -details -statistics link show "$interface"
    exit 0
fi

if (( EUID == 0 )); then
    privileged=()
elif command -v sudo >/dev/null 2>&1; then
    privileged=(sudo)
else
    echo "错误：需要 root 权限，但系统中没有 sudo。" >&2
    exit 1
fi

echo "正在配置 $interface，波特率 $bitrate bit/s ..."
"${privileged[@]}" ip link set "$interface" down
"${privileged[@]}" ip link set "$interface" type can bitrate "$bitrate"
"${privileged[@]}" ip link set "$interface" txqueuelen "$txqueuelen"
"${privileged[@]}" ip link set "$interface" up

echo "CAN 接口配置完成："
ip -details -statistics link show "$interface"
