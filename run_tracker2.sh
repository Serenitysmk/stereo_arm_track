#!/bin/bash
####################
# 可配置参数
####################

# 如需帮助，可改为true
HELP=false

# 共有相机的数量
NUM_CAMERAS=2

# 使用相机的列表（列表为当前使用相机的序列号集）
CAMERA_LIST="7L03E0EPAK00005, 7L03E0EPAK00022"

# 配置文件的路径（默认为根目录的./config）
CONFIG_PATH="./config"

# 图像显示的缩放系数
IMAGE_DISPLAY_SCALE=0.25

# 世界显示的缩放系数
WORLD_DISPLAY_SCALE=5

# 最大显示标示物的轨迹长度
MAX_TRACK_LENGTH=10000

# 是否使用录用的视频数据
INPUT_FROM_VIDEOS=true

# 如果使用录用好的视频，视频路径（默认./data/videos）
VIDEO_PATH="./data_transport/for_marker_detection"

# 输出路径
OUTPUT_DIR="./result/tracker2"

####################
# 运行标示物跟踪
####################

EXE_PATH="./build-Release/exe/run_tracker"

if $HELP
then
$EXE_PATH --helpshort
else
$EXE_PATH \
--num_cameras=$NUM_CAMERAS \
--camera_list="${CAMERA_LIST}" \
--config_path=$CONFIG_PATH \
--image_display_scale=$IMAGE_DISPLAY_SCALE \
--world_display_scale=$WORLD_DISPLAY_SCALE \
--max_track_length=$MAX_TRACK_LENGTH \
--input_from_videos=$INPUT_FROM_VIDEOS \
--video_path=$VIDEO_PATH \
--output_dir=$OUTPUT_DIR
fi
