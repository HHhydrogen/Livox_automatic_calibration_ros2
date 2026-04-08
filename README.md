# Livox 外参自动标定工具 v0.1 快速手册

## 简介
本工具用于 Livox 多雷达外参自动标定（适用于 Mid、Horizon 等型号）。
标定流程核心思路是：

1. 使用基准雷达（Base LiDAR）序列点云建图。
2. 将目标雷达（Target LiDAR）点云持续配准到该地图。
3. 迭代优化每帧外参估计，最后通过拟合得到稳定的最终外参矩阵。

![image](./pic/1.png)
**图 1** 红色：基准雷达建图结果；绿色：目标雷达自动标定结果

## 依赖
需要以下依赖：

- CMake
- PCL (>= 1.7)
- Eigen

建议安装 ROS 环境（通常会包含以上依赖）。

## 编译
在本目录执行：

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

编译后会生成 3 个可执行文件：

- `mapping`：建图
- `calibration`：逐帧标定
- `fitline`：参数拟合并输出最终结果

## ROS2 用法（colcon）
在工作区根目录执行：

```bash
colcon build --packages-select livox_automatic_calibration_ros2
source install/setup.bash
```

可执行程序：

- `ros2 run livox_automatic_calibration_ros2 mapping [data_root]`
- `ros2 run livox_automatic_calibration_ros2 calibration [data_root]`
- `ros2 run livox_automatic_calibration_ros2 fitline [data_root]`

说明：

- `data_root` 可选；不传时默认使用包安装目录的 `share/livox_automatic_calibration_ros2/data`。
- 推荐显式传入你自己的数据目录（避免把结果写到 install 目录）。
- 也可以直接运行安装后的脚本：

```bash
./install/livox_automatic_calibration_ros2/lib/livox_automatic_calibration_ros2/run.sh [data_root]
```

## 运行
### 1. 准备标定输入数据
在你的数据目录（下文记作 `data_root`）中准备以下结构：

- 基准雷达帧：`data/Base_LiDAR_Frames/*.pcd`
- 目标雷达帧：`data/Target_LiDAR_Frames/*.pcd`
- 初值外参：`data/Init_Matrix.txt`

PCD 命名要求：

- 从 `100000.pcd` 开始编号。
- 后续按整数递增命名（例如 `100001.pcd`、`100002.pcd` ...）。
- 两个目录中同名文件必须表示同一时刻数据（时间同步）。

建议目录示例：

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

### 2. 运行标定流程

推荐（ROS2 工作区方式）：

```bash
source install/setup.bash
ros2 run livox_automatic_calibration_ros2 mapping <data_root>
ros2 run livox_automatic_calibration_ros2 calibration <data_root>
ros2 run livox_automatic_calibration_ros2 fitline <data_root>
```

或一键脚本方式：

```bash
source install/setup.bash
./install/livox_automatic_calibration_ros2/lib/livox_automatic_calibration_ros2/run.sh <data_root>
```

脚本顺序为：

1. 执行 `mapping`，生成基准地图 `data/H-LiDAR-Map-data/H_LiDAR_Map.pcd`。
2. 执行 `calibration`，逐帧估计外参。
3. 执行 `fitline`，拟合并输出最终结果矩阵。

### 3. 输出文件

- `H-LiDAR-Map-data/H_LiDAR_Map.pcd`：基准雷达构建子图结果
- `T_Matrix.txt`：基准雷达轨迹矩阵
- `calib_data.txt`：逐帧标定数据
- 终端 `fitline` 输出中的 `Result Matrix`：最终外参

### 4. 常见报错排查

1. `No numbered PCD frames found ... (expect 100000.pcd...)`
    - 含义：未找到从 `100000.pcd` 开始的连续帧。
    - 处理：检查命名是否从 `100000.pcd` 起，且 Base/Target 两侧同名同步。

2. `Couldn't read .../100000.pcd` 或 `Exit Code 255`
    - 含义：程序已启动，但输入数据缺失或路径错误。
    - 处理：确认 `<data_root>/Base_LiDAR_Frames/100000.pcd` 与 `<data_root>/Target_LiDAR_Frames/100000.pcd` 存在。

3. `Couldn't read H_LiDAR_Map`
    - 含义：`calibration` 启动时未找到 mapping 结果。
    - 处理：先成功运行 `mapping`，确认 `<data_root>/H-LiDAR-Map-data/H_LiDAR_Map.pcd` 已生成。

![image](./pic/output.png)
**图 2** 示例数据输出

## 使用注意事项
- 必须保证双雷达数据严格同步。
- 基准雷达建图质量直接影响标定精度。
- 采集时运动尽量平稳、缓慢，建议先做运动畸变校正。
- 初值外参不要求非常精确，但应大致对齐。
- Mid-40/Horizon 常用按 100ms 一帧导出 PCD；Mid-70 也建议先采用等时间窗口切帧并保证双雷达同窗口同步。

## 你在当前 radar 仓库里做双 MID-70 自动标定的步骤
下面步骤针对你当前仓库已包含 `livox_ros2_driver` 和 `Livox_SDK` 的情况。

### 步骤 1：配置双雷达驱动
1. 修改 `src/ridar_driver/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json`，填入两台 MID-70 的广播码，并将对应设备 `enable_connect` 设为 `true`。
2. 使用 `multi_topic=1`（每个雷达单独话题），保证两雷达数据分开发布。
3. 建议两雷达都使用同一时间基准（硬件同步优先，至少保证系统时钟一致）。

### 步骤 2：录制双雷达原始数据
1. 启动驱动后确认有两个点云话题（例如两个不同设备话题）。
2. 录制 rosbag2，确保同时录到两路点云。
3. 采集轨迹建议“慢速环绕 + 多方向转动”，环境尽量选几何结构丰富、静态障碍物较多的场地。

### 步骤 3：将两路点云转换为同步 PCD 序列
1. 从 rosbag2 中分别导出 base 与 target 两路点云。
2. 以固定时间窗切帧（建议 100ms/帧）并导出为 PCD。
3. 统一命名为 `100000.pcd` 起，保持两路同名文件严格时间对齐。

### 步骤 4：把数据放入标定目录
在你的 `data_root` 目录执行：

```bash
mkdir -p <data_root>/Base_LiDAR_Frames <data_root>/Target_LiDAR_Frames <data_root>/H-LiDAR-Map-data
```

然后：

1. 将基准雷达 PCD 放到 `<data_root>/Base_LiDAR_Frames/`。
2. 将目标雷达 PCD 放到 `<data_root>/Target_LiDAR_Frames/`。
3. 填写 `<data_root>/Init_Matrix.txt`（使用机械测量值或经验值作为初值）。
4. 确认 `<data_root>/parameters.txt` 存在（可复制包内默认参数后按需要调整）。

### 步骤 5：编译并运行自动标定

```bash
colcon build --packages-select livox_automatic_calibration_ros2
source install/setup.bash
ros2 run livox_automatic_calibration_ros2 mapping <data_root>
ros2 run livox_automatic_calibration_ros2 calibration <data_root>
ros2 run livox_automatic_calibration_ros2 fitline <data_root>
```

### 步骤 6：查看输出并验证
1. 查看终端中 `fitline` 打印的 `Result Matrix`，这就是目标雷达到基准雷达的外参结果。
2. 将结果写回你的融合链路后，在重叠区域检查是否存在明显错层。
3. 若结果不稳定：优先检查时间同步、初值矩阵是否过差、以及建图数据是否存在动态干扰。

## 支持
- 邮箱：cs@livoxtech.com
- GitHub Issues

## 引用
```bibtex
@article{gong2018target,
    title={A Target-Free Automatic Self-Calibration Approach for Multibeam Laser Scanners},
    author={Gong, Zheng and Wen, Chenglu and Wang, Cheng and Li, Jonathan},
    journal={IEEE Transactions on Instrumentation and Measurement},
    volume={67},
    number={1},
    pages={238--240},
    year={2018},
    publisher={IEEE}
}
```

**Developer: [Livox](https://www.livoxtech.com/)**

