#!/bin/bash

set -e

# 可选参数1: data 根目录，默认使用安装目录下的 share/<pkg>/data
DATA_ROOT="$1"

if [ -z "$DATA_ROOT" ]; then
	SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
	DATA_ROOT="$(cd "$SCRIPT_DIR/../../share/livox_automatic_calibration_ros2/data" && pwd)"
fi

echo "Using data root: $DATA_ROOT"
"$(dirname "$0")/mapping" "$DATA_ROOT"
"$(dirname "$0")/calibration" "$DATA_ROOT"
"$(dirname "$0")/fitline" "$DATA_ROOT"
