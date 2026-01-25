# RC2026 Gazebo Classic 仿真场地功能包

![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange?logo=gazebo&logoColor=white)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)


![image](./assets/overview.png)

根据重邮开源的blender场地文件, 将场地导出为.dae格式, 并添加到gazebo classic中, KFS模型根据贴图由脚本自动生成
> 注意本场地功能包仅在ROS2 humble 以及 gazebo classic下测试通过, 其他ROS2版本未测试

# 场地模型说明
本项目使用了重庆邮电大学 HXC 战队提供的Robocon 2026比赛场地模型, 进行了部分删改.

## 新版更新介绍 (v2.0)
- **动态 KFS 生成**:  不再使用静态 world 文件, 而是通过 ROS Node 动态加载和管理 KFS 模型。
- **场地控制 GUI**:  新增图形化控制界面, 支持裁判系统模拟。
- **配置化生成**:  支持通过 YAML 配置文件调整九宫格生成参数及库存数量。

# 快速开始
0. 安装依赖
```bash
pip install ttkbootstrap
```

1. 编译项目
```bash
colcon build --packages-select rc2026_field
source install/setup.bash
```

2. 启动仿真 
```bash
ros2 launch rc2026_field rc2026_field_sim.launch.py
```

# 场地控制 GUI 功能
启动仿真后会自动弹出控制面板 (`field_gui`), 主要功能包括:

1.  **系统控制**:
    *   **重置/生成**: 重新随机生成场上 KFS(梅林)。
    *   **弹丸计数**: 实时显示红/蓝方剩余发弹量。
    *   **全流程仿真模式**: 开启后九宫格可放置的KFS

2.  **队伍选择**:
    *   选择 **红队/蓝队** 后, 点击九宫格空位可放置对应队伍的 KFS。
    *   选择 **无** 时, 仅可查看状态或移除模型。

3.  **区域交互**:
    *   **梅林区**: 点击存在的 KFS 可将其移除。
    *   **九宫格**:
        *   点击 **空位**: 放置当前选定队伍的 KFS (需先选队)。
        *   点击 **已有模型**: 移除该模型。

# 关键配置参数
配置文件路径: `config/kfs_config.yaml`

| 参数项 | 说明 | 关键子项 |
| :--- | :--- | :--- |
| **inventory** | 库存配置 | `num_r1`(R1 kfs数), `num_r2_true`(r2 kfs数), `num_fake`(fake kfs数) |
| **grid** | 九宫格生成 | `base_x/y`(中心坐标), `pitch_x/z`(间距), `random_range_*`(随机偏移量) |
| **models** | 静态坐标 | 定义梅林区及库存区模型的默认绝对坐标 |

详细配置请参考配置文件内的中文注释。

# 调整说明

- 修正了RC2026场地梅林场地摆放错误
- 将武馆的九宫格边框贴上黑色亚光乙烯胶带
- 删除了广告牌与文字




