#!/bin/bash
####################
# 同步拉取多相机的图片流并存入本地
####################

# 如需帮助，可改为true
HELP=false

# 共有相机的数量
NUM_CAMERAS=1

# 使用相机的列表（列表为当前使用相机的序列号集）
#CAMERA_LIST="7L03E0EPAK00002, 7L03E0EPAK00005, 7L03E0EPAK00022, 7L03E0EPAK00026"
CAMERA_LIST="7L03E0EPAK00002"

# 输出路径
OUTPUT_DIR="./data/save_frames"

####################
# 运行标示物跟踪
####################

EXE_PATH="./build-Release/exe/save_frames"

if $HELP
then
$EXE_PATH --helpshort
else
$EXE_PATH \
--num_cameras=$NUM_CAMERAS \
--camera_list="${CAMERA_LIST}" \
--output_dir=$OUTPUT_DIR
fi
