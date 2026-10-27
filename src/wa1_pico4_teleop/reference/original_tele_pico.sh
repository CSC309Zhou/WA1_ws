#!/bin/bash
# 从环境变量严格校验并解析 ROBOT_TYPE
# 仅允许: ZYD, WA1, WA2, I2, WA1_400K, WA1_400L, WA2_LS, WA2_TY20, WA2_L, U2_WA1
PARSED_TYPE=$(echo "${ROBOT_TYPE}" | tr 'A-Z' 'a-z' | tr -d ' \n\r')

case "$PARSED_TYPE" in
    zyd)
        ROBOT_TYPE="zyd"
        ;;
    wa1|wa1_400k|wa1_400l|u2_wa1)
        ROBOT_TYPE="wa1"
        ;;
    wa2|wa2_ls|wa2_ty20|wa2_l)
        ROBOT_TYPE="wa2"
        ;;
    i2)
        ROBOT_TYPE="i2"
        ;;
    *)
        echo "Error: 不支持的 ROBOT_TYPE 环境变量值: '${ROBOT_TYPE}'" >&2
        echo "支持的合法值包含: ZYD, WA1, WA2, I2, WA1_400K, WA1_400L, WA2_LS, WA2_TY20, WA2_L, U2_WA1" >&2
        exit 1
        ;;
esac

prefix_log() {
    local tag="$1"; shift
    stdbuf -oL -eL "$@" 2>&1 | sed -u "s/^/[$tag] /"
}

cleanup() {
    trap '' SIGTERM SIGINT
    kill -INT 0
    wait
    exit 0
}

trap cleanup SIGTERM SIGINT
prefix_log talker rosrun picoxr talker &
sleep 1
prefix_log ee_delta rosrun picoxr ee_delta_teleop _robot_type:=${ROBOT_TYPE} &
sleep 1
prefix_log xr_logger rosrun picoxr xr_logger.py &
wait
