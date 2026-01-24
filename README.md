# RC2026 Gazebo Classic 仿真场地功能包

![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange?logo=gazebo&logoColor=white)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)


![image](./assets/overview.png)

根据重邮开源的blender场地文件, 将场地导出为.dae格式, 并添加到gazebo classic中, KFS模型根据贴图由脚本自动生成
> 注意本场地功能包仅在ROS2 humble 以及 gazebo classic下测试通过, 其他ROS2版本未测试

# 场地模型说明
本项目使用了重庆邮电大学 HXC 战队提供的Robocon 2026比赛场地模型, 进行了部分删改

# 快速开始
1. 编译项目
```bash
colcon build --packages-select rc2026_field
source install/setup.bash
```

2. 启动仿真
```bash
ros2 launch rc2026_field rc2026_field_sim.launch.py
```

# TODO
- 功能增加: 场地每次加载自动随机按照规则在梅林随机放置KFS, 支持yaml简单配置
- 功能增加: 场地每次加载自动随机按照规则在武馆九宫格随机放置KFS, 支持yaml简单配置
    -  精简上传的代码, 然后改成config中进行配置, 精简坐标

# 调整说明

- 修正了RC2026场地梅林场地摆放错误
- 将武馆的九宫格边框贴上黑色亚光乙烯胶带
- 删除了广告牌与文字




## KFS 坐标分布图

以下网格展示了红蓝双方 KFS (Kung Fu Saplings) 的放置坐标 (x, y, z)。
**排列说明**: 图示**下方为入口** (Entry Side)，对应 ID 01-03。机器人通常从下方进入梅林区。
**ID 排序**: 从入口开始 (ID 01-03) 到 最深处 (ID 10-12)。

### 红方 (Red Team)
- 坐标基于全局坐标系 (Global Frame)。
- **X轴**: 垂直方向 (Top -> Bottom 对应 X: -1.4 -> 2.2)
- **Y轴**: 水平方向 (Left -> Right 对应 Y: 1.8 -> 4.2)
- **入口**: ID 01, 02, 03 (位于图示下方)

```text
      ^ X 负方向 (Deep)
      |
      |   [ ID 10 ]             [ ID 11 ]             [ ID 12 ]
      |   (-1.4, 1.8, 5.2)      (-1.4, 3.0, 5.2)      (-1.4, 4.2, 5.2)
      |
      |   [ ID 07 ]             [ ID 08 ]             [ ID 09 ]
      |   (-0.2, 1.8, 5.4)      (-0.2, 3.0, 5.4)      (-0.2, 4.2, 5.4)
      |
      |   [ ID 04 ]             [ ID 05 ]             [ ID 06 ]
      |   (1.0, 1.8, 5.6)       (1.0, 3.0, 5.6)       (1.0, 4.2, 5.6)
      |
      |   [ ID 01 ]             [ ID 02 ]             [ ID 03 ]   <-- 入口 (Entry)
      |   (2.2, 1.8, 5.8)       (2.2, 3.0, 5.8)       (2.2, 4.2, 5.8)
      v X 正方向 (Near)
      +-------------------------------------------------------------> Y
```

### 蓝方 (Blue Team)
- 坐标基于全局坐标系 (Global Frame)。
- **入口**: ID 01, 02, 03 (位于图示下方)

```text
      ^ X 负方向 (Deep)
      |
      |   [ ID 10 ]             [ ID 11 ]             [ ID 12 ]
      |   (-1.4, -4.2, 5.2)     (-1.4, -3.0, 5.2)     (-1.4, -1.8, 5.2)
      |
      |   [ ID 07 ]             [ ID 08 ]             [ ID 09 ]
      |   (-0.2, -4.2, 5.4)     (-0.2, -3.0, 5.4)     (-0.2, -1.8, 5.4)
      |
      |   [ ID 04 ]             [ ID 05 ]             [ ID 06 ]
      |   (1.0, -4.2, 5.6)      (1.0, -3.0, 5.6)      (1.0, -1.8, 5.6)
      |
      |   [ ID 01 ]             [ ID 02 ]             [ ID 03 ]   <-- 入口 (Entry)
      |   (2.2, -4.2, 5.8)      (2.2, -3.0, 5.8)      (2.2, -1.8, 5.8)
      v X 正方向 (Near)
      +-------------------------------------------------------------> Y
```