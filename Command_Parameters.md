# OpenCDA 仿真命令参数文档

## 目录

1. [项目结构概述](#项目结构概述)
2. [命令行参数](#命令行参数)
3. [YAML配置参数详解](#yaml配置参数详解)
4. [可用的仿真场景](#可用的仿真场景)
5. [使用示例](#使用示例)
6. [参数配置最佳实践](#参数配置最佳实践)

---

## 项目结构概述

OpenCDA是一个开源的协作自动驾驶仿真平台，基于CARLA模拟器。项目主要目录结构如下：

```
OpenCDA-Deployment/
├── opencda.py                          # 主入口脚本
├── opencda/
│   ├── core/                           # 核心功能模块
│   │   ├── actuation/                  # 执行器模块
│   │   ├── application/                # 应用模块（单车、车队等）
│   │   ├── common/                     # 通用工具
│   │   ├── plan/                       # 规划模块
│   │   ├── safety/                     # 安全管理
│   │   ├── sensing/                    # 感知和定位
│   │   └── map/                        # 地图管理
│   ├── scenario_testing/               # 仿真场景
│   │   ├── config_yaml/                # 场景配置文件
│   │   ├── scenarios/                  # 地图文件
│   │   ├── evaluations/                # 评估工具
│   │   ├── utils/                      # 仿真工具函数
│   │   └── *.py                        # 各种仿真脚本
│   ├── customize/                      # 定制模块
│   └── co_simulation/                  # 与SUMO共仿真
├── scripts/                            # 辅助脚本
├── docs/                               # 文档
└── requirements.txt                    # Python依赖

```

---

## 命令行参数

### 基本语法

```bash
python opencda.py -t <scenario_name> [options]
```

### 参数详解

#### 必需参数

| 参数 | 长名称 | 类型 | 说明 | 示例 |
|------|--------|------|------|------|
| `-t` | `--test_scenario` | string | **必需**。仿真场景名称，必须与`opencda/scenario_testing/`文件夹中的Python脚本名称和`opencda/scenario_testing/config_yaml/`文件夹中的YAML文件名称相匹配 | `single_2lanefree_carla` |

#### 可选参数

| 参数 | 长名称 | 类型 | 默认值 | 说明 |
|------|--------|------|--------|------|
| `--record` | - | bool | False | 是否记录仿真过程并保存为`.log`日志文件 |
| `--apply_ml` | - | bool | False | 是否启用深度学习框架 |
| `-v` | `--version` | string | `0.9.11` | CARLA模拟器版本。支持`0.9.11`和`0.9.12` |

### 命令行示例

```bash
# 基础单车场景测试
python opencda.py -t single_2lanefree_carla

# 指定CARLA版本
python opencda.py -t single_2lanefree_carla -v 0.9.12

# 启用日志记录
python opencda.py -t single_2lanefree_carla --record

# 启用机器学习（需先安装PyTorch）
python opencda.py -t single_town06_carla --apply_ml

# 完整示例：车队稳定性测试，使用0.9.12版本，启用日志和ML
python opencda.py -t platoon_stability_2lanefree_carla -v 0.9.12 --record --apply_ml
```

---

## YAML配置参数详解

YAML配置文件定义了仿真的所有参数。系统采用OmegaConf进行配置管理，支持配置继承和覆盖。

### 配置加载流程

1. 加载`default.yaml`（默认参数）
2. 加载场景特定的YAML文件（如`single_2lanefree_carla.yaml`）
3. 后加载的配置覆盖前面的设置

### 世界配置 (world)

定义CARLA模拟器的基本参数。

```yaml
world:
  sync_mode: true              # 是否启用同步模式（推荐true）
  client_port: 2000            # CARLA客户端连接端口
  fixed_delta_seconds: 0.05    # 仿真时间步长（秒），0.05s = 20fps
  seed: 11                     # 随机种子（固定值保证可重复性）
  weather:
    sun_altitude_angle: 15     # 太阳高度角，范围[-90, 90]，-90为午夜，90为中午
    cloudiness: 0              # 云量，范围[0, 100]，0为晴朗，100为最厚
    precipitation: 0           # 降雨量，范围[0, 100]，100为最大降雨
    precipitation_deposits: 0  # 地面湿度，范围[0, 100]，影响水坑生成
    wind_intensity: 0          # 风力强度，影响降雨效果
    fog_density: 0             # 雾密度，范围[0, 100]
    fog_distance: 0            # 雾能见度开始距离（米）
    fog_falloff: 0             # 雾衰减率
    wetness: 0                 # 地面湿度程度
```

### 感知与定位 - 车辆基础配置

#### 感知模块 (vehicle_base.sensing.perception)

```yaml
vehicle_base:
  sensing:
    perception:
      activate: false          # 是否激活感知模块
                               # false: 从CARLA服务器直接获取对象位置
                               # true: 使用真实传感器（相机/激光雷达）
      camera:
        visualize: 1           # 显示多少个摄像头图像，0为不显示
        num: 1                 # 挂载的摄像头数量，最多4个
        positions:             # 相机相对位置和朝向 [x, y, z, yaw]
          - [2.5, 0, 1.0, 0]   # 格式：前方中间摄像头
      lidar:                   # 激光雷达配置
        visualize: true        # 是否可视化激光点云
        channels: 32           # 激光雷达通道数
        range: 50              # 测量范围（米）
        points_per_second: 100000  # 每秒生成的点数
        rotation_frequency: 20 # 旋转频率（Hz）
        upper_fov: 10.0        # 上视场角（度）
        lower_fov: -30.0       # 下视场角（度）
        dropoff_general_rate: 0.0    # 通用信号丢失率
        dropoff_intensity_limit: 1.0 # 强度限制丢失率
        dropoff_zero_intensity: 0.0  # 零强度丢失率
        noise_stddev: 0.0      # 高斯噪声标准差
```

#### 定位模块 (vehicle_base.sensing.localization)

```yaml
    localization:
      activate: false          # 是否激活定位模块
                               # false: 从CARLA服务器直接获取车辆位置
                               # true: 使用GNSS + IMU + Kalman滤波
      dt: 0.05                 # Kalman滤波器时间步长（应等于world.fixed_delta_seconds）
      gnss:                    # GNSS传感器参数
        noise_alt_stddev: 0.001        # 高度测量噪声标准差（米）
        noise_lat_stddev: 1.0e-6       # 纬度测量噪声标准差
        noise_lon_stddev: 1.0e-6       # 经度测量噪声标准差
        heading_direction_stddev: 0.1  # 方向角噪声标准差（度）
        speed_stddev: 0.2              # 速度测量噪声标准差（m/s）
      debug_helper:            # 调试辅助参数
        show_animation: false  # 是否显示实时轨迹动画
        show_plotting: false   # 仿真后是否显示定位误差图表
        x_scale: 1.0           # X轴缩放因子（用于可视化）
        y_scale: 100.0         # Y轴缩放因子（用于可视化）
```

#### RSU基础配置 (rsu_base)

RSU（路侧单元）配置与车辆类似，但包含更多传感器：

```yaml
rsu_base:
  sensing:
    perception:
      camera:
        num: 4                 # RSU通常配置更多摄像头
        positions:
          - [2.5, 0, 1.0, 0]
          - [0.0, 0.3, 1.8, 100]
          - [0.0, -0.3, 1.8, -100]
          - [-2.0, 0.0, 1.5, 180]
      lidar:
        channels: 32
        range: 120             # RSU视距更远
        points_per_second: 1000000
        # ... 其他参数
    localization:
      activate: true
```

### 地图管理 (vehicle_base.map_manager)

```yaml
  map_manager:
    pixels_per_meter: 2        # 栅格化地图分辨率（像素/米）
    raster_size: [224, 224]    # 栅格地图尺寸（像素H×W）
    lane_sample_resolution: 0.1 # 车道采样分辨率（每0.1m采样一个点）
    visualize: true            # 是否可视化栅格地图
    activate: true             # 是否激活地图管理器
```

### 安全管理 (vehicle_base.safety_manager)

```yaml
  safety_manager:
    print_message: true        # 是否打印安全警告信息
    collision_sensor:
      history_size: 30         # 碰撞检测历史记录大小
      col_thresh: 1            # 碰撞阈值
    stuck_dector:              # 卡住检测
      len_thresh: 500          # 路径长度阈值（米）
      speed_thresh: 0.5        # 速度阈值（m/s）
    offroad_dector: []         # 离道路检测
    traffic_light_detector:    # 交通灯检测
      light_dist_thresh: 20    # 检测距离（米）
```

### 行为与控制参数 (vehicle_base.behavior)

```yaml
  behavior:
    # 速度控制
    max_speed: 111             # 最大速度（km/h）
    tailgate_speed: 121        # 需要快速接近前方车辆时的速度（km/h）
    speed_lim_dist: 3          # 目标速度 = max_speed - speed_lim_dist
    speed_decrease: 15         # 跟车模式中的减速值（km/h）
    safety_time: 4             # TTC（碰撞时间）安全阈值（秒）
    emergency_param: 0.4       # 紧急制动判断参数
    
    # 行为控制
    ignore_traffic_light: true # 是否无视交通信号灯
    overtake_allowed: true     # 是否允许超车（车队中通常为false）
    collision_time_ahead: 1.5  # 碰撞检测时间提前量（秒）
    overtake_counter_recover: 35 # 两次超车之间的最小步数
    sample_resolution: 4.5     # 路径点采样间隔（米）
    
    # 局部规划器
    local_planner:
      buffer_size: 12          # 路点缓冲区大小
      trajectory_update_freq: 15 # 轨迹更新频率（步）
      waypoint_update_freq: 9  # 路点更新频率（步）
      min_dist: 3              # 移除当前位置附近的路点的最小距离（米）
      trajectory_dt: 0.20      # 轨迹采样时间间隔（秒）
      debug: false             # 是否显示前进/历史路点
      debug_trajectory: false  # 是否绘制轨迹点和路径
```

### 控制器参数 (vehicle_base.controller)

```yaml
  controller:
    type: pid_controller       # 控制器类型（目前仅支持PID）
    args:
      lat:                     # 横向（转向）控制参数
        k_p: 0.75              # 比例系数
        k_d: 0.02              # 微分系数
        k_i: 0.4               # 积分系数
      lon:                     # 纵向（加速/制动）控制参数
        k_p: 0.37              # 比例系数
        k_d: 0.024             # 微分系数
        k_i: 0.032             # 积分系数
      dynamic: false           # 是否使用动态PID参数调整
      dt: 0.05                 # 控制器时间步长（应等于world.fixed_delta_seconds）
      max_brake: 1.0           # 最大制动力（0-1）
      max_throttle: 1.0        # 最大油门（0-1）
      max_steering: 0.3        # 最大转向角（弧度）
```

### V2X通信 (vehicle_base.v2x)

```yaml
  v2x:
    enabled: true              # 是否启用V2X通信
    communication_range: 35    # 通信范围（米）
```

### 背景交通管理 (carla_traffic_manager)

```yaml
carla_traffic_manager:
  sync_mode: true              # 必须与world.sync_mode相同
  global_distance: 5           # 车辆间的最小距离（米）
  global_speed_perc: -100      # 全局速度百分比偏移
                               # CARLA默认速度30km/h
                               # -100表示60km/h，20表示24km/h
  set_osm_mode: true           # 是否启用OSM模式
  auto_lane_change: false      # 是否自动变道
  ignore_lights_percentage: 0  # 忽视交通灯的车辆百分比（0-100）
  random: false                # 是否随机选择车辆颜色和模型
  vehicle_list: []             # 背景车辆列表（见下方说明）
  range: []                     # 当vehicle_list为~时的生成范围
```

#### 背景车辆配置说明

```yaml
carla_traffic_manager:
  vehicle_list:
    - spawn_position: [-330, 8.3, 0.3, 0, 0, 0]  # [x, y, z, roll, pitch, yaw]
      vehicle_speed_perc: 0    # 该车辆的速度偏移百分比
    - spawn_position: [-320, 4.8, 0.3, 0, 0, 0]
      vehicle_speed_perc: -200
```

或使用范围生成：

```yaml
  range:                       # [x_min, x_max, y_min, y_max, x_step, y_step, num_vehicles]
    - -400
    - 400
    - 0
    - 10
    - 50
    - 5
    - 20
```

### 车队基础参数 (platoon_base)

```yaml
platoon_base:
  max_capacity: 10             # 车队最大容量
  inter_gap: 0.6               # 队内车辆间的目标时间间隔（秒）
  open_gap: 1.2                # 队外合并时的时间间隔（秒）
  warm_up_speed: 55            # 协作合并所需的热身速度（km/h）
  change_leader_speed: true    # 是否改变领导车速度
  leader_speeds_profile: [85, 95]  # 领导车的速度档次（km/h）
  stage_duration: 10           # 每个速度阶段的持续时间（秒）
```

### 仿真场景定义 (scenario)

```yaml
scenario:
  single_cav_list: []          # 单车列表
  platoon_list: []             # 车队列表
  rsu_list: []                 # 路侧单元列表（可选）
```

#### 单车配置示例

```yaml
scenario:
  single_cav_list:
    - name: cav1               # 车辆唯一标识符
      spawn_position: [-370, 8.3, 0.3, 0, 0, 0]  # 生成位置和朝向
      spawn_special: [0.34]    # 可选：特殊生成位置（如合并车道）
      destination: [700, 8.3, 0.3]  # 目的地坐标
      v2x:
        communication_range: 45  # 该车的通信范围（覆盖全局设置）
      behavior:                # 可选：覆盖全局行为参数
        local_planner:
          debug_trajectory: true
          debug: true
```

#### 车队配置示例

```yaml
scenario:
  platoon_list:
    - name: platoon1           # 车队标识符
      destination: [1000, 8.3, 0.3]  # 车队目的地
      members:                 # 队员列表（第一个为领导车）
        - name: cav1
          spawn_position: [-350, 8.3, 0.3, 0, 0, 0]
          perception:
            camera:
              visualize: 1
              num: 1
              positions: [[2.5, 0, 1.0, 0]]
            lidar:
              visualize: true
          behavior:
            local_planner:
              debug_trajectory: true
              debug: false
        - name: cav2
          spawn_position: [-360, 8.3, 0.3, 0, 0, 0]
        - name: cav3
          spawn_position: [-370, 8.3, 0.3, 0, 0, 0]
```

### SUMO共仿真配置（可选）

如果使用CARLA-SUMO共仿真，需要额外配置：

```yaml
sumo:
  port: 8813                   # SUMO连接端口
  sumo_file_parent_path: "path/to/sumo/config"  # SUMO配置目录
```

---

## 可用的仿真场景

### 场景列表

以下是所有内置仿真场景的列表，包括其配置文件和脚本：

| 场景名称 | 配置文件 | 脚本文件 | 描述 | 应用类型 |
|---------|---------|---------|------|---------|
| `single_2lanefree_carla` | `single_2lanefree_carla.yaml` | `single_2lanefree_carla.py` | 单车在定制化2车道高速公路上行驶 | 单车 |
| `single_town05_cosim` | `single_town05_cosim.yaml` | `single_town05_cosim.py` | 单车在Town05地图上，使用CARLA-SUMO共仿真 | 单车 + SUMO |
| `single_town06_carla` | `single_town06_carla.yaml` | `single_town06_carla.py` | 单车在城市Town06地图上完整功能测试 | 单车 + 感知 |
| `single_town06_cosim` | `single_town06_cosim.yaml` | `single_town06_cosim.py` | 单车在Town06，CARLA-SUMO共仿真 | 单车 + SUMO |
| `single_2lanefree_cosim` | `single_2lanefree_cosim.yaml` | `single_2lanefree_cosim.py` | 单车在2车道高速公路，CARLA-SUMO共仿真 | 单车 + SUMO |
| `single_intersection_town06_carla` | `single_intersection_town06_carla.yaml` | `single_intersection_town06_carla.py` | 单车通过城市路口 | 单车 |
| `platoon_stability_2lanefree_carla` | `platoon_stability_2lanefree_carla.yaml` | `platoon_stability_2lanefree_carla.py` | 4车车队稳定性测试（领导车变速） | 车队 |
| `platoon_joining_2lanefree_carla` | `platoon_joining_2lanefree_carla.yaml` | `platoon_joining_2lanefree_carla.py` | 单车从合并车道加入既有车队 | 车队 + 合并 |
| `platoon_joining_2lanefree_cosim` | `platoon_joining_2lanefree_cosim.yaml` | `platoon_joining_2lanefree_cosim.py` | 车队合并，CARLA-SUMO共仿真 | 车队 + SUMO |
| `platoon_joining_town06_carla` | `platoon_joining_town06_carla.yaml` | `platoon_joining_town06_carla.py` | 单车超过多辆车后加入车队（城市场景） | 车队 + 感知 |
| `v2xp_online_carla` | `v2xp_online_carla.yaml` | `v2xp_online_carla.py` | V2X感知在线测试 | V2X + 感知 |
| `v2xp_datadump_town06_carla` | `v2xp_datadump_town06_carla.yaml` | `v2xp_datadump_town06_carla.py` | V2X感知数据导出 | V2X + 数据导出 |
| `openscenario_carla` | `openscenario_carla.yaml` | `openscenario_carla.py` | OpenScenario格式的仿真 | 场景驱动 |

### 场景详细说明

#### 1. 单车场景

##### single_2lanefree_carla
- **用途**: 测试单个CAV的基本功能
- **地图**: 定制化2车道高速公路（2lane_freeway_simplified）
- **背景交通**: 5-8辆背景车辆
- **推荐参数**:
  ```bash
  python opencda.py -t single_2lanefree_carla -v 0.9.12
  ```

##### single_town06_carla
- **用途**: 完整功能测试（感知、定位、规划、控制）
- **地图**: CARLA官方Town06城市地图
- **背景交通**: 自动生成
- **特点**: 需要启用`--apply_ml`才能完整运行
- **推荐参数**:
  ```bash
  python opencda.py -t single_town06_carla -v 0.9.12 --apply_ml
  ```

##### single_intersection_town06_carla
- **用途**: 测试路口通过场景
- **地图**: Town06城市地图
- **特点**: 包含交通灯和复杂的路口场景

#### 2. 车队场景

##### platoon_stability_2lanefree_carla
- **用途**: 测试车队稳定性
- **配置**: 4车车队，领导车周期性改变速度
- **评估**: 队员能否维持目标时间间隔
- **推荐参数**:
  ```bash
  python opencda.py -t platoon_stability_2lanefree_carla -v 0.9.12
  ```

##### platoon_joining_2lanefree_carla
- **用途**: 测试协作合并和加入场景
- **配置**: 既有4车车队 + 1辆合并车
- **地图**: 定制化2车道高速公路（包含合并车道）
- **推荐参数**:
  ```bash
  python opencda.py -t platoon_joining_2lanefree_carla -v 0.9.12
  ```

##### platoon_joining_town06_carla
- **用途**: 城市场景下的车队合并
- **特点**: 需要感知能力，对多个车辆的目标跟踪
- **推荐参数**:
  ```bash
  python opencda.py -t platoon_joining_town06_carla -v 0.9.12 --apply_ml
  ```

#### 3. 共仿真场景（CARLA + SUMO）

##### 场景命名规则
以`_cosim`结尾的场景使用CARLA-SUMO共仿真，需要：
1. 安装SUMO模拟器
2. 设置`SUMO_HOME`环境变量
3. 安装`traci`库：`pip install traci==1.26.0`

##### 示例：single_2lanefree_cosim
```bash
# 设置SUMO环境（Windows）
set SUMO_HOME=C:\path\to\sumo

# 运行共仿真
python opencda.py -t single_2lanefree_cosim -v 0.9.12
```

#### 4. V2X相关场景

##### v2xp_online_carla
- **用途**: 在线V2X感知能力验证
- **特点**: 多个CAV共享感知信息

##### v2xp_datadump_town06_carla
- **用途**: 导出V2X感知和定位数据用于离线分析
- **输出**: YAML格式的仿真数据

---

## 使用示例

### 示例1：基础单车仿真

运行最简单的单车场景：

```bash
# 首先启动CARLA服务（另一个终端）
# Windows: path\to\carla\CarlaUE4.exe
# Linux: path/to/carla/CarlaUE4.sh

# 然后运行仿真
python opencda.py -t single_2lanefree_carla -v 0.9.12
```

你会看到：
- 单个CAV在定制化高速公路上行驶
- CAV尝试以100km/h的速度到达目的地
- 安全管理器监控碰撞风险

### 示例2：车队稳定性测试

测试4车车队在领导车变速情况下的稳定性：

```bash
python opencda.py -t platoon_stability_2lanefree_carla -v 0.9.12 --record
```

在YAML配置中，领导车的速度档案：
```yaml
platoon_base:
  leader_speeds_profile: [85, 95]    # 交替速度
  stage_duration: 10                 # 每个速度持续10秒
```

### 示例3：启用感知和数据记录

运行带完整感知功能的场景并记录日志：

```bash
python opencda.py -t single_town06_carla -v 0.9.12 --apply_ml --record
```

此时：
- YOLOv5模型加载用于对象检测
- 相机和激光雷达点云被可视化
- 仿真日志保存为`single_town06_carla.log`

### 示例4：自定义配置

创建新的仿真场景（以`my_scenario`为例）：

1. **创建Python脚本** `opencda/scenario_testing/my_scenario.py`：

```python
def run_scenario(opt, scenario_params):
    """
    opt: 包含命令行参数的对象
      - opt.test_scenario: 场景名称
      - opt.apply_ml: 是否启用ML
      - opt.version: CARLA版本
      - opt.record: 是否记录
    scenario_params: 合并后的YAML配置字典
    """
    # 你的仿真逻辑
    pass
```

2. **创建YAML配置** `opencda/scenario_testing/config_yaml/my_scenario.yaml`：

```yaml
description: "My custom scenario"

vehicle_base:
  behavior:
    max_speed: 100

scenario:
  single_cav_list:
    - name: my_cav
      spawn_position: [0, 0, 0, 0, 0, 0]
      destination: [500, 0, 0]
```

3. **运行自定义场景**：

```bash
python opencda.py -t my_scenario -v 0.9.12
```

---

## 参数配置最佳实践

### 1. CARLA版本支持

- 推荐使用 **0.9.12** 版本
- **不支持** 0.9.13 及更高版本

```bash
# 检查版本
python opencda.py -t single_2lanefree_carla -v 0.9.12
```

### 2. 同步模式

**强烈推荐启用同步模式**：
```yaml
world:
  sync_mode: true              # 确保仿真和控制同步
  fixed_delta_seconds: 0.05    # 20 FPS
```

同步模式的优点：
- 确定性的仿真结果
- 易于复现
- 控制更加稳定

### 3. 传感器配置

#### 不使用感知（快速仿真）
```yaml
vehicle_base:
  sensing:
    perception:
      activate: false          # 直接从CARLA获取位置
    localization:
      activate: false          # 直接从CARLA获取状态
```

**优点**: 仿真速度快，适合算法验证  
**缺点**: 不能测试真实感知系统

#### 使用完整感知
```yaml
vehicle_base:
  sensing:
    perception:
      activate: true
      camera:
        visualize: 1
        num: 1
      lidar:
        visualize: true
    localization:
      activate: true
```

**优点**: 完整测试感知和定位  
**缺点**: 仿真速度较慢

### 4. PID控制器调参指南

#### 初始参数值
```yaml
controller:
  args:
    lat:
      k_p: 0.75    # 转向控制
      k_d: 0.02
      k_i: 0.4
    lon:
      k_p: 0.37    # 加速控制
      k_d: 0.024
      k_i: 0.032
```

#### 调参步骤
1. **首先调整Kp**（比例系数）
   - 增大Kp会增加响应速度
   - 但过大会造成振荡

2. **然后调整Kd**（微分系数）
   - 增大Kd可以减少超调
   - 过大会增加噪声敏感性

3. **最后调整Ki**（积分系数）
   - 增大Ki可以消除稳态误差
   - 过大会造成延迟

### 5. V2X通信范围

```yaml
vehicle_base:
  v2x:
    enabled: true
    communication_range: 35    # 35米
```

**设置指导**：
- 高速场景：35-50米
- 城市场景：20-30米
- 停泊场景：10-20米

### 6. 时间间隔配置

重要的时间参数必须一致：

```yaml
world:
  fixed_delta_seconds: 0.05    # 20 FPS

vehicle_base:
  sensing:
    localization:
      dt: 0.05                 # 必须相同
  controller:
    args:
      dt: 0.05                 # 必须相同
```

### 7. 调试技巧

#### 启用轨迹可视化
```yaml
vehicle_base:
  behavior:
    local_planner:
      debug: true              # 显示路点
      debug_trajectory: true   # 显示规划路径
```

#### 启用定位调试
```yaml
vehicle_base:
  sensing:
    localization:
      debug_helper:
        show_animation: true            # 实时轨迹
        show_plotting: true             # 仿真后显示误差
```

#### 启用日志记录
```bash
python opencda.py -t my_scenario -v 0.9.12 --record
```

### 8. 离线仿真的可重复性

要获得完全可重复的仿真结果：

1. 固定随机种子
```yaml
world:
  seed: 42                     # 固定值
```

2. 启用同步模式
```yaml
world:
  sync_mode: true
```

3. 不随机化背景车辆
```yaml
carla_traffic_manager:
  random: false
  ignore_lights_percentage: 0
  auto_lane_change: false
```

### 9. 性能优化

#### 降低仿真负荷
```yaml
world:
  fixed_delta_seconds: 0.1     # 改为10 FPS（牺牲精度提高速度）

vehicle_base:
  sensing:
    perception:
      activate: false          # 禁用传感器仿真
    localization:
      activate: false

  map_manager:
    visualize: false           # 禁用可视化

carla_traffic_manager:
  vehicle_list: []             # 减少背景车辆
```

#### 提高计算效率
```yaml
vehicle_base:
  behavior:
    local_planner:
      buffer_size: 6           # 减小缓冲区
      trajectory_update_freq: 20
      waypoint_update_freq: 12
```

---

## 参考资源

- **OpenCDA官方文档**: [GitHub](https://github.com/ucla-mobility/OpenCDA)
- **CARLA模拟器**: [CARLA官网](https://carla.org/)
- **OmegaConf配置库**: [官方文档](https://github.com/omry/omegaconf)
- **SUMO模拟器**: [SUMO官网](https://sumo.dlr.de/)

---
