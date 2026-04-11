# Livox 双雷达自动标定工具（ROS2 版）

本仓库可作为独立子仓库直接使用，适用于 Livox 双雷达（如 MID-70）外参自动标定。

## 一、仅 clone 子仓库时的使用教程

### 1. 获取代码
```bash
git clone https://github.com/HHhydrogen/Livox_automatic_calibration_ros2.git
cd Livox_automatic_calibration_ros2
```

### 2. 依赖要求
1. ROS2（建议 Humble 及以上）
2. PCL（>= 1.7）
3. Eigen
4. colcon（若按 ROS2 工作区方式构建）

### 3. 构建方式
推荐 ROS2 工作区构建：
```bash
cd <your_ws>
mkdir -p src
# 将本仓库放到 <your_ws>/src/
colcon build --packages-select livox_automatic_calibration_ros2
source install/setup.bash
```

也支持普通 CMake 构建：
```bash
cd Livox_automatic_calibration_ros2
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 4. 数据准备
在 data_root 下准备：
```text
<data_root>/
  Base_LiDAR_Frames/
    100000.pcd
    100001.pcd
    ...
  Target_LiDAR_Frames/
    100000.pcd
    100001.pcd
    ...
  H-LiDAR-Map-data/
  Init_Matrix.txt
  parameters.txt
```

要求：
1. Base 与 Target 必须同名同步。
2. 文件名必须从 100000.pcd 连续递增。
3. Init_Matrix.txt 提供粗略初值即可。

### 5. 运行标定
ROS2 方式：
```bash
source install/setup.bash
ros2 run livox_automatic_calibration_ros2 mapping <data_root>
ros2 run livox_automatic_calibration_ros2 calibration <data_root>
ros2 run livox_automatic_calibration_ros2 fitline <data_root>
```

一键脚本方式：
```bash
source install/setup.bash
./install/livox_automatic_calibration_ros2/lib/livox_automatic_calibration_ros2/run.sh <data_root>
```

### 6. 输出结果
1. H-LiDAR-Map-data/H_LiDAR_Map.pcd：基准地图
2. T_Matrix.txt：基准轨迹
3. calib_data.txt：逐帧标定数据
4. fitline 终端输出中的 Result Matrix：最终外参

## 二、代码结构与功能说明

### 目录结构
```text
include/
  blam_slam/                # SLAM 主流程接口
  point_cloud_filter/       # 点云滤波
  point_cloud_odometry/     # 里程计配准
  point_cloud_localization/ # 基于地图的定位
  point_cloud_mapper/       # 地图维护
  geometry_utils/           # 几何计算工具
  parameter_utils/          # 参数读取
  ransac.h / fitline.h      # 拟合相关头文件

src/
  mapping/                  # 基准雷达建图（生成地图与轨迹）
  calibration/              # 目标雷达逐帧标定
  ransac/                   # 参数拟合，输出最终矩阵

data/                       # 输入输出数据目录
run.sh                      # 串联 mapping->calibration->fitline
```

### 可执行程序职责
1. mapping：读取 Base_LiDAR_Frames，构建局部地图并输出 T_Matrix。
2. calibration：读取目标点云、地图和轨迹，逐帧估计外参并输出 calib_data。
3. fitline：对 calib_data 做拟合，输出最终外参矩阵。

## 三、常见问题

1. No numbered PCD frames found
- 含义：未检测到从 100000.pcd 开始的连续帧。
- 处理：检查命名和目录。

2. Couldn't read .../100000.pcd（Exit Code 255）
- 含义：输入路径正确但文件不存在。
- 处理：确认 data_root 路径与文件是否存在。

3. Couldn't read H_LiDAR_Map
- 含义：calibration 启动前未成功执行 mapping。
- 处理：先运行 mapping 并确认地图文件生成。
