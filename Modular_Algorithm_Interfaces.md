# OpenCDA 模块化算法接口文档

## 目录

1. [架构总览](#架构总览)
2. [感知模块接口](#感知模块接口)
3. [定位模块接口](#定位模块接口)
4. [决策模块接口](#决策模块接口)
5. [控制模块接口](#控制模块接口)
6. [数据流与集成](#数据流与集成)
7. [自定义算法开发指南](#自定义算法开发指南)

---

## 架构总览

### 系统架构图

```
┌─────────────────────────────────────────────────────────────┐
│                    仿真主循环 (Vehicle Manager)              │
│  virtual_manager.run_step() / vehicle_manager.run_step()   │
└──────────┬──────────────────────────────────────────────────┘
           │
           ├─────────────────────────────────────────────┐
           │                                             │
           ▼                                             ▼
┌──────────────────────┐                  ┌──────────────────────┐
│  感知和定位模块      │                  │  行为和控制模块      │
├──────────────────────┤                  ├──────────────────────┤
│                      │                  │                      │
│ 1. 感知模块          │                  │ 1. 行为规划          │
│    PerceptionMgr     │──┐               │    BehaviorAgent     │
│                      │  │               │                      │
│ 2. 定位模块          │  │               │ 2. 轨迹规划          │
│    LocalizationMgr   │  │               │    LocalPlanner      │
│                      │  │               │                      │
│ 3. V2X通信           │  │               │ 3. 控制执行          │
│    V2XManager        │  │               │    ControlManager    │
│                      │  │               │                      │
└──────────────────────┘  │               └──────────────────────┘
           ▲               │
           │               └───────────────────────┐
           │                                       │
           └───────────────────────────────────────┘
                  (信息交互)
```

### 核心接口调用顺序

```
1. update_info()
   ├─ localization.localize()      → 获取位置和速度
   ├─ perception.detect(ego_pos)   → 获取周围对象
   └─ agent.update_information()   → 更新决策输入

2. run_step(target_speed)
   ├─ agent.run_step()             → 行为规划 + 局部规划
   │  ├─ collision_manager()       → 碰撞检测
   │  ├─ local_planner.run_step()  → 轨迹生成
   │  └─ 返回: target_speed, target_loc
   │
   ├─ controller.run_step()        → 控制计算
   │  ├─ lon_run_step(target_speed) → 纵向PID
   │  ├─ lat_run_step(target_loc)   → 横向PID
   │  └─ 返回: carla.VehicleControl
   │
   └─ vehicle.apply_control()      → 应用控制

3. 结果评估与反馈
   ├─ safety_manager.run_step()
   ├─ v2x_manager.update()
   └─ data_dumper.run_step()
```

---

## 感知模块接口

感知模块负责检测和识别周围环境中的动态和静态对象。

### 1. 文件位置

```
opencda/core/sensing/perception/
├── perception_manager.py          # 感知管理器（主接口）
├── sensor_transformation.py       # 传感器坐标变换
├── obstacle_vehicle.py            # 检测到的车辆对象
├── static_obstacle.py             # 静态障碍物（交通灯等）
└── o3d_lidar_libs.py             # 3D点云处理
```

### 2. 核心类：PerceptionManager

#### 初始化

```python
class PerceptionManager(object):
    """
    感知模块管理器
    
    参数
    ----------
    vehicle : carla.Vehicle
        CARLA车辆对象
    
    config_yaml : dict
        感知模块配置参数
        - activate: bool，是否激活感知
        - camera: dict，摄像头配置
        - lidar: dict，激光雷达配置
    
    data_dumping : bool
        是否进行数据导出
    """
    
    def __init__(self, vehicle, config_yaml, data_dumping=False):
        self.activate = config_yaml.get('activate', False)
        self.camera_num = config_yaml['camera']['num']
        self.lidar_visualize = config_yaml['lidar']['visualize']
        
        # 初始化摄像头和激光雷达传感器
        self.rgb_camera = []        # RGB摄像头列表
        self.lidar = None           # 激光雷达对象
        self.ml_manager = None      # ML/DL模型管理（YOLOv5）
```

#### 核心方法：detect()

**接口定义**
```python
def detect(self, ego_pos):
    """
    检测周围对象。
    
    参数
    ----------
    ego_pos : carla.Transform
        自车当前位置和姿态
    
    返回
    -------
    objects : dict
        检测结果字典，包含：
        {
            'vehicles': [               # 检测到的车辆列表
                {
                    'position': [x, y, z],  # 3D坐标
                    'confidence': score,    # 置信度
                    'velocity': [vx, vy],   # 速度
                    'id': object_id         # 对象ID
                },
                ...
            ],
            'traffic_lights': [         # 交通灯列表
                {
                    'state': 'Red/Green/Yellow',
                    'position': [x, y, z],
                    'id': light_id
                },
                ...
            ]
        }
    """
```

**两种工作模式**

1. **deactivate_mode** (直接从CARLA获取)
   ```python
   def deactivate_mode(self, objects):
       """
       禁用感知时，直接从CARLA服务器获取对象信息
       速度快，用于快速验证算法
       """
       # 直接调用CARLA API获取世界中的所有车辆
       # - 对象位置
       # - 对象速度
       # - 对象类型
       objects['vehicles'] = [...]
       objects['traffic_lights'] = [...]
       return objects
   ```

2. **activate_mode** (真实传感器仿真)
   ```python
   def activate_mode(self, objects):
       """
       激活感知时，使用YOLOv5 + LiDAR融合进行目标检测
       
       处理流程:
       1. 从摄像头获取RGB图像
       2. 运行YOLOv5目标检测 → 2D检测框
       3. 从激光雷达获取3D点云
       4. 将点云投影到像素坐标系
       5. 2D检测框 + 3D点云融合 → 3D目标定位
       6. 计算目标速度（从CARLA服务器获取）
       7. 可视化（可选）
       """
       # YOLOv5检测
       yolo_detection = self.ml_manager.object_detector(rgb_images)
       
       # 相机-激光雷达融合
       for camera_idx, camera in enumerate(self.rgb_camera):
           objects = o3d_camera_lidar_fusion(
               objects,
               yolo_detection.xyxy[camera_idx],  # 2D检测框
               self.lidar.data,                   # 3D点云
               projected_lidar,                   # 投影坐标
               self.lidar.sensor                  # 传感器标定参数
           )
       
       # 速度计算
       self.speed_retrieve(objects)
       return objects
   ```

### 3. 传感器类

#### CameraSensor（摄像头）

```python
class CameraSensor:
    """
    摄像头传感器管理
    
    属性
    ----------
    image : np.ndarray
        当前RGB图像，形状为 (H, W, 3)
    
    sensor : carla.Sensor
        CARLA摄像头actor
    
    frame : int
        当前帧编号
    
    timestamp : float
        时间戳
    """
    
    def __init__(self, vehicle, world, relative_position, global_position):
        """
        参数
        ----------
        relative_position : tuple
            相对位置 (x, y, z, yaw)
            例：(2.5, 0, 1.0, 0) 表示前方中间摄像头
        
        global_position : list 或 None
            全局位置（仅用于RSU）
        """
        # 创建RGB摄像头blueprint
        blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('fov', '100')  # 视场角100度
        
        # 挂载到车辆或设置全局位置
        self.sensor = world.spawn_actor(...)
        
        # 设置回调函数接收图像数据
        self.sensor.listen(
            lambda event: self._on_rgb_image_event(weak_self, event)
        )
```

#### LidarSensor（激光雷达）

```python
class LidarSensor:
    """
    激光雷达传感器管理
    
    属性
    ----------
    data : np.ndarray
        点云数据，形状为 (N, 4)
        每行: [x, y, z, intensity]
    
    o3d_pointcloud : open3d.geometry.PointCloud
        Open3D格式的点云对象
    
    sensor : carla.Sensor
        CARLA激光雷达actor
    """
    
    def __init__(self, vehicle, world, config_yaml, global_position):
        """
        参数
        ----------
        config_yaml : dict
            激光雷达配置：
            {
                'channels': 32,              # 通道数
                'range': 50,                 # 探测范围（米）
                'points_per_second': 100000,# 点云密度
                'rotation_frequency': 20,   # 旋转频率（Hz）
                'upper_fov': 10.0,          # 上视场角
                'lower_fov': -30.0,         # 下视场角
                'noise_stddev': 0.0         # 噪声标准差
            }
        """
        # 设置传感器属性
        blueprint.set_attribute('channels', str(config_yaml['channels']))
        blueprint.set_attribute('range', str(config_yaml['range']))
        # ...其他属性
        
        # 生成点云数据
        self.sensor.listen(
            lambda event: self._on_data_event(weak_self, event)
        )
```

### 4. 检测结果数据结构

#### ObstacleVehicle（检测到的车辆）

```python
class ObstacleVehicle(object):
    """
    检测到的车辆信息封装
    
    属性
    ----------
    location : [x, y, z]
        3D笛卡尔坐标（米）
    
    velocity : [vx, vy]
        2D速度矢量（m/s）
    
    confidence : float
        检测置信度 [0, 1]
    
    distance : float
        到自车的距离（米）
    
    angle : float
        相对角度（度）
    """
```

### 5. 自定义感知算法

创建文件 `opencda/customize/core/sensing/perception/my_detector.py`：

```python
class PerceptionManager(object):
    """自定义感知算法"""
    
    def __init__(self, vehicle, config_yaml, data_dumping=False):
        self.config = config_yaml
        self.ml_model = None  # 加载你的模型
        
    def detect(self, ego_pos):
        """
        实现自己的检测算法
        
        输入: ego_pos - 自车位置
        输出: objects字典
        """
        objects = {'vehicles': [], 'traffic_lights': []}
        
        # 1. 获取传感器数据
        rgb_images = self._get_camera_images()
        point_cloud = self._get_lidar_pointcloud()
        
        # 2. 自定义检测算法
        detections = self.ml_model.inference(rgb_images, point_cloud)
        
        # 3. 格式转换
        for det in detections:
            vehicle_info = {
                'position': [det.x, det.y, det.z],
                'confidence': det.conf,
                'velocity': [det.vx, det.vy]
            }
            objects['vehicles'].append(vehicle_info)
        
        return objects
```

---

## 定位模块接口

定位模块融合GNSS、IMU和Kalman滤波器，提供车辆的位置和速度估计。

### 1. 文件位置

```
opencda/core/sensing/localization/
├── localization_manager.py        # 定位管理器（主接口）
├── kalman_filter.py               # Kalman滤波器实现
├── coordinate_transform.py        # 坐标系变换
└── localization_debug_helper.py   # 调试工具
```

### 2. 核心类：LocalizationManager

#### 初始化

```python
class LocalizationManager(object):
    """
    定位模块管理器
    
    参数
    ----------
    vehicle : carla.Vehicle
        CARLA车辆对象
    
    config_yaml : dict
        定位模块配置：
        {
            'activate': bool,           # 是否启用定位算法
            'dt': 0.05,                 # 时间步长（秒）
            'gnss': {                   # GNSS传感器噪声
                'noise_alt_stddev': 0.001,
                'noise_lat_stddev': 1.0e-6,
                'noise_lon_stddev': 1.0e-6,
                'heading_direction_stddev': 0.1,
                'speed_stddev': 0.2
            },
            'debug_helper': {...}       # 调试选项
        }
    
    carla_map : carla.Map
        CARLA地图对象
    """
    
    def __init__(self, vehicle, config_yaml, carla_map):
        self.activate = config_yaml['activate']
        
        # 初始化传感器
        self.gnss = GnssSensor(vehicle, config_yaml['gnss'])
        self.imu = ImuSensor(vehicle)
        
        # Kalman滤波器
        self.kf = KalmanFilter(config_yaml['dt'])
        
        # 内部状态
        self._ego_pos = None        # 估计位置
        self._speed = 0             # 估计速度
        self._ego_pos_history = deque(maxlen=100)  # 位置历史
```

#### 核心方法：localize()

**接口定义**
```python
def localize(self):
    """
    执行一步定位处理
    
    处理流程:
    1. 如果disabled: 直接从CARLA获取位置/速度
    2. 如果enabled: 
       - 从GNSS获取WGS84坐标 (lat, lon, alt, timestamp)
       - 从IMU获取角加速度、线性加速度
       - 坐标系转换: WGS84 → ENU (地心坐标系)
       - Kalman滤波融合
       - 输出估计位置和速度
    
    更新的内部状态:
    - self._ego_pos : carla.Transform
    - self._speed : float (km/h)
    - self._ego_pos_history : deque
    """
```

**两种工作模式**

1. **deactivate_mode**（直接获取）
   ```python
   # 禁用定位算法时的简单实现
   self._ego_pos = self.vehicle.get_transform()
   self._speed = get_speed(self.vehicle)  # 从CARLA获取速度
   
   # 优点: 快速、确定性
   # 缺点: 不能模拟实际传感器噪声
   ```

2. **activate_mode**（传感器融合）
   ```python
   # 启用完整定位算法
   
   # 步骤1: 获取传感器测量值
   speed_true = get_speed(self.vehicle)
   speed_noise = self.add_speed_noise(speed_true)
   
   # GNSS: WGS84坐标 (latitude, longitude, altitude)
   x, y, z = geo_to_transform(
       self.gnss.lat,
       self.gnss.lon,
       self.gnss.alt,
       self.geo_ref.latitude,
       self.geo_ref.longitude,
       0.0
   )
   
   # 步骤2: 添加传感器噪声
   heading_angle = self.add_heading_direction_noise(
       rotation.yaw
   )  # 方向角噪声
   
   # 步骤3: Kalman滤波融合
   if len(self._ego_pos_history) == 0:
       # 初始化: 第一帧使用GNSS测量
       self.kf.run_step_init(
           x, y, 
           np.deg2rad(heading_angle), 
           speed_noise / 3.6
       )
   else:
       # 后续帧: 融合GNSS、IMU、速度
       x_kf, y_kf, heading_angle_kf, speed_kf = \
           self.kf.run_step(
               x, y,                          # GNSS测量
               np.deg2rad(heading_angle),     # 方向角
               speed_noise / 3.6,             # 速度测量
               self.imu.gyroscope[2]          # IMU陀螺仪(偏航角速度)
           )
   
   # 步骤4: 输出估计值
   self._ego_pos = carla.Transform(
       carla.Location(x=x_kf, y=y_kf, z=z),
       carla.Rotation(pitch=0, yaw=heading_angle_kf, roll=0)
   )
   self._speed = speed_kf * 3.6  # m/s → km/h
   
   # 步骤5: 保存历史轨迹
   self._ego_pos_history.append(self._ego_pos)
   self._timestamp_history.append(self.gnss.timestamp)
   ```

### 3. 传感器类

#### GnssSensor（GPS/GNSS）

```python
class GnssSensor(object):
    """
    GNSS传感器，提供WGS84坐标
    
    属性
    ----------
    sensor : carla.Sensor
        CARLA GNSS传感器actor
    
    lat : float
        纬度（度）
    
    lon : float
        经度（度）
    
    alt : float
        海拔高度（米）
    
    timestamp : float
        数据时间戳
    """
```

#### ImuSensor（IMU/惯导）

```python
class ImuSensor(object):
    """
    IMU传感器，提供加速度和陀螺仪数据
    
    属性
    ----------
    sensor : carla.Sensor
        CARLA IMU传感器actor
    
    accelerometer : (ax, ay, az)
        线性加速度 (m/s²)
    
    gyroscope : (gx, gy, gz)
        角速度 (rad/s)
    
    compass : float
        磁罗盘方向角
    """
```

### 4. Kalman滤波器接口

```python
class KalmanFilter(object):
    """
    扩展Kalman滤波器（EKF）实现
    
    状态向量: [x, y, vx, vy, yaw, ω]
    - (x,y): 位置
    - (vx,vy): 速度
    - yaw: 方向角
    - ω: 角速度
    """
    
    def __init__(self, dt):
        """
        参数
        ----------
        dt : float
            时间步长（秒）
        """
        self.dt = dt
        self.P = np.eye(6)  # 协方差矩阵
        self.Q = ...        # 过程噪声
        self.R = ...        # 测量噪声
    
    def run_step_init(self, x, y, yaw, speed):
        """
        初始化滤波器
        """
    
    def run_step(self, x_meas, y_meas, yaw_meas, speed_meas, gyro_z):
        """
        执行一步滤波
        
        参数
        ----------
        x_meas, y_meas : float
            位置测量值（米）
        yaw_meas : float
            方向角测量值（弧度）
        speed_meas : float
            速度测量值（m/s）
        gyro_z : float
            陀螺仪Z轴输出（角速度，rad/s）
        
        返回
        -------
        x_est, y_est, yaw_est, speed_est : float
            估计位置、方向角和速度
        """
```

### 5. 获取位置和速度

```python
def get_ego_pos(self):
    """
    获取自车位置
    
    返回
    -------
    ego_pos : carla.Transform
        包含位置(location)和姿态(rotation)
    """
    return self._ego_pos

def get_ego_spd(self):
    """
    获取自车速度
    
    返回
    -------
    speed : float
        速度（km/h）
    """
    return self._speed
```

### 6. 自定义定位算法

创建文件 `opencda/customize/core/sensing/localization/my_localizer.py`：

```python
class LocalizationManager(object):
    """自定义定位算法"""
    
    def __init__(self, vehicle, config_yaml, carla_map):
        self.vehicle = vehicle
        self.gnss = GnssSensor(vehicle, config_yaml['gnss'])
        self.imu = ImuSensor(vehicle)
        self.my_kf = MyCustomKalmanFilter()  # 你的滤波器
    
    def localize(self):
        """实现自己的定位算法"""
        # 获取传感器测量
        gnss_meas = (self.gnss.lat, self.gnss.lon, self.gnss.alt)
        imu_meas = (self.imu.accelerometer, self.imu.gyroscope)
        
        # 自定义滤波或融合算法
        fused_state = self.my_kf.fuse(gnss_meas, imu_meas)
        
        # 更新内部状态
        self._ego_pos = carla.Transform(...)
        self._speed = ...
```

---

## 决策模块接口

决策模块包括行为规划（BehaviorAgent）和轨迹规划（LocalPlanner）两部分。

### 1. 文件位置

```
opencda/core/plan/
├── behavior_agent.py              # 行为规划（主接口）
├── local_planner_behavior.py      # 轨迹规划
├── collision_check.py             # 碰撞检测
├── global_route_planner.py        # 全局路由规划
└── global_route_planner_dao.py    # 地图数据访问层
```

### 2. 行为规划：BehaviorAgent

#### 初始化

```python
class BehaviorAgent(object):
    """
    行为规划代理
    
    参数
    ----------
    vehicle : carla.Vehicle
        CARLA车辆对象
    
    carla_map : carla.Map
        CARLA地图
    
    config_yaml : dict
        行为配置：
        {
            'max_speed': 111,           # 最大速度(km/h)
            'tailgate_speed': 121,      # 跟尾速度
            'speed_lim_dist': 3,        # 速度限制距离
            'speed_decrease': 15,       # 减速幅度
            'safety_time': 4,           # TTC安全阈值(秒)
            'emergency_param': 0.4,     # 紧急制动参数
            'ignore_traffic_light': true,
            'overtake_allowed': true,
            'collision_time_ahead': 1.5,
            'overtake_counter_recover': 35,
            'sample_resolution': 4.5,
            'local_planner': {...},
            'debug': false
        }
    """
    
    def __init__(self, vehicle, carla_map, config_yaml):
        self.vehicle = vehicle
        self._map = carla_map
        
        # 速度相关参数
        self.max_speed = config_yaml['max_speed']
        self.tailgate_speed = config_yaml['tailgate_speed']
        
        # 安全相关参数
        self._collision_check = CollisionChecker(
            time_ahead=config_yaml['collision_time_ahead']
        )
        
        # 规划器
        self._global_planner = GlobalRoutePlanner(carla_map)
        self._local_planner = LocalPlanner(
            self, carla_map, config_yaml['local_planner']
        )
        
        # 状态变量
        self._ego_pos = None        # 当前位置
        self._ego_speed = 0         # 当前速度
        self.objects = {}           # 周围对象信息
```

#### 核心方法：update_information()

```python
def update_information(self, ego_pos, ego_speed, objects):
    """
    更新来自感知和定位模块的信息
    
    参数
    ----------
    ego_pos : carla.Transform
        自车位置（来自定位模块）
    
    ego_speed : float
        自车速度，km/h（来自定位模块）
    
    objects : dict
        周围对象信息（来自感知模块）
        {
            'vehicles': [ObstacleVehicle, ...],
            'traffic_lights': [TrafficLight, ...]
        }
    """
    # 更新状态
    self._ego_speed = ego_speed
    self._ego_pos = ego_pos
    self.break_distance = self._ego_speed / 3.6 * self.emergency_param
    
    # 更新本地规划器
    self.get_local_planner().update_information(ego_pos, ego_speed)
    
    # 更新对象列表（过滤白名单）
    self.objects = objects
    self.obstacle_vehicles = self.white_list_match(objects['vehicles'])
    
    # 更新调试信息
    self.debug_helper.update(ego_speed, self.ttc)
```

#### 核心方法：run_step()

**接口定义**
```python
def run_step(self, target_speed=None, 
             collision_detector_enabled=True,
             lane_change_allowed=True):
    """
    执行一步行为规划
    
    参数
    ----------
    target_speed : float 或 None
        手动指定的目标速度（km/h），如果为None使用默认值
    
    collision_detector_enabled : bool
        是否启用碰撞检测
    
    lane_change_allowed : bool
        是否允许变道
    
    返回
    -------
    target_speed : float
        下一时刻的目标速度（km/h）
        返回0表示应该停止
    
    target_location : carla.Location 或 None
        轨迹点的目标位置
        返回None表示应该停止
    """
```

**决策流程**

```
1. 初始化和状态更新
   ├─ 获取当前位置的路点信息
   ├─ 更新超车计数器
   └─ 更新目的地推送标志

2. 到达检查
   └─ if is_close_to_destination(): return (0, None)  # 到达目标

3. 交通灯管理
   └─ if traffic_light_manager() != 0: return (0, None)  # 红灯停止

4. 临时路由恢复
   └─ 当临时路由完成时，返回全局路由

5. 车道变更权限检查
   └─ 基于曲率、碰撞检测、超车状态等因素决定是否允许变道

6. 碰撞检测
   ├─ collision_manager()  # 检测前方碰撞
   ├─ hazard_flag: 是否有危险
   └─ obstacle_vehicle: 碰撞目标

7. 目的地推送（可选）
   └─ 当检测到可能无法进行计划的变道时，临时推送目的地

8. 行为选择
   ├─ if 危险（碰撞）: 
   │  ├─ 考虑超车行为
   │  └─ if 超车可行: 修改目的地
   │  └─ else: 跟车行为
   │
   ├─ if 跟车标志 and 距离 < break_distance:
   │  └─ return (0, None)  # 紧急停止
   │
   ├─ if 跟车标志 and 距离 < safety_distance:
   │  └─ target_speed = car_following_manager()  # 跟车速度
   │
   └─ else:
      └─ target_speed = max_speed - speed_lim_dist  # 正常速度

9. 轨迹规划
   └─ local_planner.run_step(rx, ry, rk, target_speed)
      返回: 目标速度、目标位置

10. 返回决策结果
    └─ (target_speed, target_location)
```

**关键决策函数**

| 函数 | 用途 | 输入 | 输出 |
|------|------|------|------|
| `collision_manager()` | 碰撞检测 | rx, ry, ryaw, waypoint | (hazard, vehicle, distance) |
| `overtake_management()` | 超车判断 | obstacle_vehicle | bool（是否可以超车） |
| `lane_change_management()` | 变道判断 | 无 | bool（变道是否安全） |
| `car_following_manager()` | 跟车控制 | vehicle, distance | target_speed |
| `traffic_light_manager()` | 交通灯 | waypoint | 0(通过) 或 1(停止) |
| `is_intersection()` | 路口检测 | objects, waypoint_buffer | bool |
| `set_destination()` | 设置目标 | start_loc, end_loc | 无 |

### 3. 轨迹规划：LocalPlanner

#### 初始化

```python
class LocalPlanner(object):
    """
    本地轨迹规划器，实现低层的路点追踪
    
    参数
    ----------
    agent : BehaviorAgent
        上层行为规划代理
    
    carla_map : carla.Map
        CARLA地图
    
    config : dict
        轨迹规划配置：
        {
            'buffer_size': 12,                # 路点缓冲区大小
            'min_dist': 3,                    # 移除路点最小距离
            'trajectory_update_freq': 15,     # 轨迹更新频率
            'waypoint_update_freq': 9,        # 路点更新频率
            'trajectory_dt': 0.20,            # 轨迹采样时间步长
            'debug': false,
            'debug_trajectory': false
        }
    """
    
    def __init__(self, agent, carla_map, config_yaml):
        self._vehicle = agent.vehicle
        self._map = carla_map
        
        # 路点管理
        self.waypoints_queue = deque(maxlen=20000)      # 全局路线
        self._waypoint_buffer = deque(maxlen=buffer_size) # 本地缓冲
        self._trajectory_buffer = deque(maxlen=30)      # 轨迹缓冲
        
        # 时间参数
        self.dt = config_yaml['trajectory_dt']  # 0.2秒采样一个轨迹点
```

#### 核心方法：run_step()

**接口定义**
```python
def run_step(self, rx, ry, rk, target_speed=None, 
             trajectory=None, following=False):
    """
    执行本地轨迹规划一步
    
    参数
    ----------
    rx, ry : list
        规划路径的x、y坐标列表
    
    rk : list
        路径曲率列表
    
    target_speed : float 或 None
        目标速度（km/h）
    
    trajectory : list 或 None
        预生成的轨迹（用于车队跟车）
    
    following : bool
        是否处于跟车状态
    
    返回
    -------
    speed : float
        下一时刻的目标速度（km/h）
    
    waypoint : carla.Location
        下一个轨迹目标点的位置
    """
```

**轨迹生成流程**

```
1. 全局路径生成 (generate_path)
   ├─ 基于当前位置和目的地
   ├─ 使用全局路由规划器计算
   └─ 输出: 离散化的路点和曲率

2. 轨迹采样 (generate_trajectory)
   ├─ 将路点转换为光滑曲线（Spline）
   ├─ 以固定时间间隔dt采样
   └─ 输出: 轨迹点序列 [(waypoint, speed), ...]

3. 路点缓冲区管理
   ├─ 移除距离自车太近的旧路点 (pop_buffer)
   └─ 补充新的前向路点 (buffer_filter)

4. 目标路点选择
   └─ _trajectory_buffer[1]为下一个目标点

5. 输出
   └─ 返回目标速度和位置
```

### 4. 全局路由规划

```python
class GlobalRoutePlanner(object):
    """
    基于CARLA地图的全局路由规划
    
    方法
    ----------
    trace_route(start_waypoint, end_waypoint):
        生成从起点到终点的路点序列
        
        返回: [(waypoint, RoadOption), ...]
        RoadOption: VOID, LEFT, RIGHT, STRAIGHT, 
                    LANEFOLLOW, CHANGELANELEFT, CHANGELANERIGHT
    """
```

### 5. 碰撞检测

```python
class CollisionChecker(object):
    """
    碰撞检测器
    
    方法
    ----------
    collision_circle_check(rx, ry, ryaw, vehicle, speed, carla_map):
        检测计划的轨迹是否与指定车辆碰撞
        
        使用圆形碰撞模型（安全圆形体积）
        
        返回: bool
            True = 无碰撞，False = 有碰撞风险
    
    adjacent_lane_collision_check(ego_loc, target_wpt, carla_map, 
                                  overtake=False):
        检测相邻车道的碰撞风险
        
        返回: (rx, ry, ryaw)
            相邻车道的规划路径
    """
```

### 6. 自定义行为规划算法

创建文件 `opencda/customize/core/plan/behavior_agent.py`：

```python
class BehaviorAgent(object):
    """自定义行为规划算法"""
    
    def __init__(self, vehicle, carla_map, config_yaml):
        self.vehicle = vehicle
        self._map = carla_map
        # ... 初始化
    
    def update_information(self, ego_pos, ego_speed, objects):
        """接收来自感知和定位的信息"""
        self._ego_pos = ego_pos
        self._ego_speed = ego_speed
        self.objects = objects
    
    def run_step(self, target_speed=None):
        """
        实现自己的决策逻辑
        
        例：基于规则的决策树
        """
        if self._ego_speed > 50:
            target_speed = 50
        elif self._detect_obstacle():
            target_speed = max(10, self._ego_speed - 5)
        else:
            target_speed = 100
        
        # 轨迹规划
        rx, ry, rk, ryaw = self._local_planner.generate_path()
        target_speed, target_location = self._local_planner.run_step(
            rx, ry, rk, target_speed
        )
        
        return target_speed, target_location
```

---

## 控制模块接口

控制模块将行为规划的输出（目标速度和位置）转换为车辆控制命令。

### 1. 文件位置

```
opencda/core/actuation/
├── control_manager.py             # 控制器管理器（主接口）
├── pid_controller.py              # PID控制器实现
└── __init__.py
```

### 2. 控制管理器：ControlManager

#### 初始化

```python
class ControlManager(object):
    """
    控制器管理器
    
    参数
    ----------
    control_config : dict
        控制配置：
        {
            'type': 'pid_controller',     # 控制器类型
            'args': {                     # 控制器参数
                'lat': {...},             # 横向参数
                'lon': {...},             # 纵向参数
                'dynamic': false,
                'dt': 0.05,
                'max_brake': 1.0,
                'max_throttle': 1.0,
                'max_steering': 0.3
            }
        }
    """
    
    def __init__(self, control_config):
        # 动态加载控制器
        controller_type = control_config['type']
        controller_class = getattr(
            importlib.import_module(
                f"opencda.core.actuation.{controller_type}"
            ),
            'Controller'
        )
        self.controller = controller_class(control_config['args'])
```

#### 核心方法

```python
def update_info(self, ego_pos, ego_speed):
    """
    更新车辆状态信息
    
    参数
    ----------
    ego_pos : carla.Transform
        自车位置和姿态
    
    ego_speed : float
        自车速度（km/h）
    """
    self.controller.update_info(ego_pos, ego_speed)

def run_step(self, target_speed, waypoint):
    """
    执行控制计算
    
    参数
    ----------
    target_speed : float
        目标速度（km/h）
    
    waypoint : carla.Location
        目标位置
    
    返回
    -------
    control : carla.VehicleControl
        车辆控制命令
        {
            'throttle': float,    # 油门 [0, 1]
            'brake': float,       # 制动 [0, 1]
            'steer': float,       # 转向 [-1, 1]
            'hand_brake': bool,
            'manual_gear_shift': bool,
            'gear': int
        }
    """
    return self.controller.run_step(target_speed, waypoint)
```

### 3. PID控制器：Controller

#### 初始化

```python
class Controller(object):
    """
    PID控制器
    
    参数
    ----------
    args : dict
        控制参数：
        {
            'lat': {                # 横向（转向）参数
                'k_p': 0.75,        # 比例系数
                'k_d': 0.02,        # 微分系数
                'k_i': 0.4          # 积分系数
            },
            'lon': {                # 纵向（加速/制动）参数
                'k_p': 0.37,
                'k_d': 0.024,
                'k_i': 0.032
            },
            'dynamic': false,       # 动态PID
            'dt': 0.05,             # 时间步长
            'max_brake': 1.0,
            'max_throttle': 1.0,
            'max_steering': 0.3
        }
    """
    
    def __init__(self, args):
        # 纵向PID参数
        self._lon_k_p = args['lon']['k_p']
        self._lon_k_d = args['lon']['k_d']
        self._lon_k_i = args['lon']['k_i']
        self._lon_ebuffer = deque(maxlen=10)  # 误差历史
        
        # 横向PID参数
        self._lat_k_p = args['lat']['k_p']
        self._lat_k_d = args['lat']['k_d']
        self._lat_k_i = args['lat']['k_i']
        self._lat_ebuffer = deque(maxlen=10)
        
        # 控制限制
        self.max_brake = args['max_brake']
        self.max_throttle = args['max_throttle']
        self.max_steering = args['max_steering']
        
        # 时间参数
        self.dt = args['dt']
        
        # 当前状态
        self.current_transform = None
        self.current_speed = 0.0
        self.past_steering = 0.0
```

#### 核心方法：run_step()

**接口定义**
```python
def run_step(self, target_speed, waypoint):
    """
    执行一步PID控制
    
    参数
    ----------
    target_speed : float
        目标速度（km/h）
    
    waypoint : carla.Location
        目标位置
    
    返回
    -------
    control : carla.VehicleControl
        控制命令
    """
```

**PID控制流程**

```
1. 紧急停止检查
   └─ if target_speed == 0 or waypoint is None:
      └─ return 紧急制动命令 (throttle=0, brake=1)

2. 纵向PID控制（速度）
   ├─ 计算速度误差: error = target_speed - current_speed
   ├─ PID计算:
   │  ├─ P项: K_p * error
   │  ├─ D项: K_d * (误差变化速率)
   │  └─ I项: K_i * (误差累积)
   ├─ 限幅: clip(-1, 1)
   └─ 输出: acceleration ∈ [-1, 1]

3. 横向PID控制（转向）
   ├─ 计算车身方向向量 v_vec
   ├─ 计算目标向量 w_vec（当前位置→目标位置）
   ├─ 计算偏角: angle = arccos(v_vec · w_vec)
   ├─ PID计算: steering = K_p * angle + K_d * dangle + K_i * ∫angle
   ├─ 限幅: clip(-max_steering, max_steering)
   └─ 平滑处理: 限制转向速率 ΔSteer ≤ 0.2

4. 加速度→油门/制动转换
   ├─ if acceleration ≥ 0:
   │  └─ throttle = min(acceleration, max_throttle), brake = 0
   └─ else:
      └─ throttle = 0, brake = min(|acceleration|, max_brake)

5. 返回控制命令
   └─ carla.VehicleControl(throttle, brake, steering)
```

**PID数学公式**

纵向PID:
```
u(t) = K_p * e(t) + K_d * de(t)/dt + K_i * ∫e(t)dt

其中:
- e(t) = target_speed - current_speed
- de/dt ≈ (e[t] - e[t-1]) / dt
- ∫e(t)dt ≈ sum(error_buffer) * dt
```

横向PID:
```
δ(t) = K_p * θ(t) + K_d * dθ(t)/dt + K_i * ∫θ(t)dt

其中:
- θ(t) = 偏航角误差 (车身方向 vs 目标方向)
- δ(t) = 转向角
```

### 4. 自定义控制器

创建文件 `opencda/customize/core/actuation/mpc_controller.py`：

```python
class Controller(object):
    """自定义模型预测控制（MPC）"""
    
    def __init__(self, args):
        # 初始化MPC模型和参数
        self.mpc_model = MPCModel()
        self.prediction_horizon = 5  # 预测步数
        self.control_horizon = 2     # 控制步数
        self.dt = args['dt']
    
    def update_info(self, ego_pos, ego_speed):
        """更新当前状态"""
        self.state = {
            'x': ego_pos.location.x,
            'y': ego_pos.location.y,
            'yaw': ego_pos.rotation.yaw,
            'speed': ego_speed
        }
    
    def run_step(self, target_speed, waypoint):
        """
        MPC控制器
        
        1. 建立预测模型
        2. 生成参考轨迹（目标速度 + 路点）
        3. 优化控制输入（最小化偏差）
        4. 返回第一时刻的控制命令
        """
        # 构造轨迹参考
        reference_trajectory = self._get_reference_traj(
            self.state, waypoint, target_speed
        )
        
        # MPC优化
        optimal_control = self.mpc_model.solve(
            self.state,
            reference_trajectory,
            self.prediction_horizon,
            self.control_horizon
        )
        
        # 返回当前控制输入
        throttle, steer = optimal_control[0]
        
        control = carla.VehicleControl()
        control.throttle = throttle
        control.brake = max(0, -throttle)  # 如果throttle为负则制动
        control.steer = steer
        return control
```

---

## 数据流与集成

### 1. 仿真主循环

```python
# 1. 每步仿真的数据流
def simulate_one_step():
    # Step 1: 感知和定位（数据采集）
    #=====================
    localization_manager.localize()
    ego_pos = localization_manager.get_ego_pos()
    ego_speed = localization_manager.get_ego_spd()
    
    objects = perception_manager.detect(ego_pos)
    
    
    # Step 2: 决策（生成目标）
    #=====================
    behavior_agent.update_information(ego_pos, ego_speed, objects)
    # 调用行为规划和轨迹规划
    target_speed, target_location = behavior_agent.run_step(
        target_speed=None,
        collision_detector_enabled=True
    )
    
    # Step 3: 控制（生成命令）
    #=====================
    controller.update_info(ego_pos, ego_speed)
    control_command = controller.run_step(target_speed, target_location)
    
    # Step 4: 执行（应用到车辆）
    #=====================
    vehicle.apply_control(control_command)
    
    # Step 5: 安全和V2X检查（可选）
    #=====================
    safety_manager.run_step()
    v2x_manager.update()
```

### 2. 时间同步

```
CARLA仿真框架:

┌──────────────────────────────────┐
│   CARLA World Tick (△t=0.05s)   │  每20ms一次
└──────────────────────────────────┘
         ↓
┌──────────────────────────────────┐
│  Sensor Callback                  │
│  - Camera ImageEvent              │  异步回调
│  - LiDAR PointCloudEvent          │  更新传感器数据
│  - GNSS LocationEvent             │
│  - IMU AccelerometerEvent         │
└──────────────────────────────────┘
         ↓
┌──────────────────────────────────┐
│  Control Step (synchronous)       │  同步执行
│  1. localization.localize()       │
│  2. perception.detect()           │
│  3. behavior_agent.run_step()     │
│  4. controller.run_step()         │
│  5. vehicle.apply_control()       │
└──────────────────────────────────┘
         ↓
┌──────────────────────────────────┐
│  Vehicle Physics Update           │  由CARLA处理
│  - Update vehicle position        │
│  - Update velocity                │
│  - Resolve collisions             │
└──────────────────────────────────┘
```

### 3. 消息传递和接口契约

| 模块 | 输出 | → | 输入 | 模块 |
|------|------|---|------|------|
| LocalizationMgr | ego_pos (Transform) | → | update_information | BehaviorAgent |
| LocalizationMgr | ego_speed (float, km/h) | → | update_information | BehaviorAgent |
| PerceptionMgr | objects (dict) | → | update_information | BehaviorAgent |
| BehaviorAgent | target_speed (float, km/h) | → | run_step | Controller |
| BehaviorAgent | target_location (Location) | → | run_step | Controller |
| BehaviorAgent | ego_pos (Transform) | → | update_info | Controller |
| BehaviorAgent | ego_speed (float, km/h) | → | update_info | Controller |
| Controller | VehicleControl | → | apply_control | Vehicle |

---

## 自定义算法开发指南

### 1. 开发框架

```
1. 创建自定义模块文件夹
   opencda/customize/core/{module}/

2. 创建与标准接口兼容的类
   - 继承原始类（可选）
   - 实现所有必需的公共方法

3. 在YAML中配置使用自定义实现
   controller:
     type: my_custom_controller  # 对应 my_custom_controller.py

4. 加载自定义模块
   - 系统会自动搜索 opencda.customize.core 路径
   - 无需修改原始代码
```

### 2. 开发感知算法

**实现步骤**

```python
# opencda/customize/core/sensing/perception/yolov8_detector.py

from opencda.core.sensing.perception.perception_manager import PerceptionManager
import torch
from ultralytics import YOLO

class PerceptionManager(PerceptionManager):
    """YOLOv8感知系统"""
    
    def __init__(self, vehicle, config_yaml, data_dumping=False):
        super().__init__(vehicle, config_yaml, data_dumping)
        # 加载YOLOv8模型替代YOLOv5
        self.model = YOLO('yolov8n.pt')  # nano模型
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    def detect(self, ego_pos):
        """使用YOLOv8进行检测"""
        objects = {'vehicles': [], 'traffic_lights': []}
        
        # 获取图像
        images = [cam.image for cam in self.rgb_camera]
        
        # YOLOv8推理
        results = self.model(images, device=self.device)
        
        # 处理检测结果
        for result in results:
            for box in result.boxes:
                if box.cls in [2, 5, 7]:  # car, bus, truck
                    objects['vehicles'].append({
                        'position': self._box_to_3d(box),
                        'confidence': float(box.conf),
                        'velocity': [0, 0]
                    })
        
        return objects
```

### 3. 开发定位算法

```python
# opencda/customize/core/sensing/localization/rtk_gnss.py

from opencda.core.sensing.localization.localization_manager import LocalizationManager
import numpy as np

class LocalizationManager(LocalizationManager):
    """RTK-GNSS高精度定位"""
    
    def localize(self):
        """RTK定位（厘米级精度）"""
        if self.activate:
            # RTK固定解
            x_rtk, y_rtk, z_rtk = self._get_rtk_solution()
            heading = self._get_rtk_heading()
            
            # 更新位置（RTK解已经包含厘米级精度）
            self._ego_pos = carla.Transform(
                carla.Location(x=x_rtk, y=y_rtk, z=z_rtk),
                carla.Rotation(yaw=heading)
            )
            self._speed = self._get_rtk_velocity()
        else:
            # 降级到标准GNSS
            super().localize()
```

### 4. 开发行为规划算法

```python
# opencda/customize/core/plan/my_behavior_planner.py

from opencda.core.plan.behavior_agent import BehaviorAgent

class BehaviorAgent(BehaviorAgent):
    """自定义行为规划：状态机"""
    
    def __init__(self, vehicle, carla_map, config_yaml):
        super().__init__(vehicle, carla_map, config_yaml)
        self.state = 'CRUISE'  # 巡航状态
    
    def run_step(self, target_speed=None):
        """基于状态机的决策"""
        ego_speed = self._ego_speed
        
        # 状态转移
        if self.state == 'CRUISE':
            if self._detect_obstacle():
                self.state = 'FOLLOWING'
            target_speed = 100
        
        elif self.state == 'FOLLOWING':
            if not self._detect_obstacle():
                self.state = 'CRUISE'
            target_speed = min(ego_speed - 5, 80)
        
        # 生成轨迹
        rx, ry, rk, ryaw = self._local_planner.generate_path()
        target_speed, target_location = self._local_planner.run_step(
            rx, ry, rk, target_speed
        )
        
        return target_speed, target_location
```

### 5. 开发控制器

```python
# opencda/customize/core/actuation/adaptive_pid.py

from opencda.core.actuation.pid_controller import Controller
import numpy as np

class Controller(Controller):
    """自适应PID控制器"""
    
    def run_step(self, target_speed, waypoint):
        """根据速度自动调整PID参数"""
        # 自适应PID增益
        speed_ratio = self.current_speed / self.max_speed
        
        # 低速时增加响应性
        if speed_ratio < 0.3:
            kp_lon = self._lon_k_p * 1.5
        else:
            kp_lon = self._lon_k_p
        
        # 纵向控制
        error = target_speed - self.current_speed
        acceleration = kp_lon * error + ...
        
        # 构造控制输出
        control = carla.VehicleControl()
        # ...设置throttle, brake, steer
        return control
```

### 6. 在YAML中配置

```yaml
# config.yaml
vehicle_base:
  sensing:
    perception:
      activate: true
      
  behavior:
    local_planner:
      debug: true

scenario:
  single_cav_list:
    - name: cav1
      spawn_position: [0, 0, 0, 0, 0, 0]
      destination: [500, 0, 0]
      perception:
        activate: true  # 使用自定义的YOLOv8
      behavior:
        local_planner:
          debug: true
```

### 7. 加载自定义模块

```python
# 系统自动加载机制
import importlib

# 加载自定义感知器
perception_module = importlib.import_module(
    "opencda.customize.core.sensing.perception.yolov8_detector"
)
PerceptionManager = getattr(perception_module, 'PerceptionManager')
```

---

## 总结与最佳实践

| 原则 | 说明 |
|------|------|
| **接口分离** | 感知 → 决策 → 控制，明确的输入输出 |
| **参数隔离** | 所有参数在YAML中定义，便于修改 |
| **模块独立** | 每个模块可独立开发和测试 |
| **时间同步** | 所有模块在同一仿真时刻运行 |
| **扩展性** | 通过继承原类实现自定义算法 |
| **降级机制** | activate=false时降级到简单实现 |
| **数据一致性** | 确保数据单位一致（速度km/h等） |
| **调试支持** | 每个模块都有debug选项和可视化工具 |

---

## 参考资源

- [感知模块源码](opencda/core/sensing/perception/perception_manager.py)
- [定位模块源码](opencda/core/sensing/localization/localization_manager.py)
- [行为规划源码](opencda/core/plan/behavior_agent.py)
- [轨迹规划源码](opencda/core/plan/local_planner_behavior.py)
- [控制模块源码](opencda/core/actuation/pid_controller.py)
- [完整命令参数文档](OpenCDA_Command_Parameters_Documentation.md)
- [OpenCDA官方文档](https://github.com/ucla-mobility/OpenCDA)

---
