# API DOCUMENT
## 1. 场地控制与反馈

### 发送指令
向controller发送控制指令

*   **Topic**: `/simulation/gui_event`
*   **Type**: `std_msgs/msg/String`

#### 示例:

1.  **放置 KFS (九宫格)**
    ```json
    {
      "action": "place",
      "team": "red",      // "red" 或 "blue"
      "target": "grid_5"  // 目标九宫格位置 (grid_1 ~ grid_9)
    }
    ```
    *注意: 全流程模式下，必须先从梅林区移除 KFS 后才能放置。*

2.  **攻击 KFS (九宫格)**
    ```json
    {
      "action": "remove",
      "team": "red",      // 攻击方队伍
      "target": "grid_5"  // 目标位置
    }
    ```
    *效果: 消耗一份武器。不可攻击己方。*

3.  **拾取 KFS (梅林)**
    ```json
    {
      "action": "remove",
      "team": "red",
      "target": "red_meilin_1" // 梅林位置 (red_meilin_1 ~ 12)
    }
    ```
    *效果: 将该 KFS 收入"已拾取"列表，供全流程模式下的放置使用。*

4.  **切换仿真模式**
    ```json
    {
      "action": "toggle_mode",
      "value": true // true: 全流程模式 (需在梅林拾取KFS), false: 独立模式(无需验证是否拾取KFS) 
    }
    ```

### 获取状态 
获取当前场地的全局状态。

*   **Topic**: `/simulation/status`
*   **Type**: `std_msgs/msg/String`
*   **Format**: JSON 字符串

```bash
ros2 topic echo /simulation/status --field data --full-length
```

#### 返回数据示例:
```json
{
  "red_weapon_count": 5,        // 红方剩余武器
  "blue_weapon_count": 5,       // 蓝方剩余武器
  "full_simulation_mode": false, // 当前模式
  "placements": {               // 场地上的 KFS 占用情况
    "grid_5": "BlueTrueKFS01",
    "red_meilin_1": "RedR1KFS01"
  }
}
```

## 2. 系统复位

重置整个仿真环境，包括 KFS 随机布局和武器数量。

*   **Service**: `/simulation/reset_kfs`
*   **Type**: `std_srvs/srv/Trigger`

**调用方式**:
发送空请求即可。系统将重新随机生成梅林区布局并清空九宫格。

---

## 附录

### MODEL_ID 参考

| 队伍 | 类型 | 数量 | Model ID |
|------|------|------|----------|
| 红方 | R1 KFS | 3 | `RedR1KFS`, `RedR1KFS_2`, `RedR1KFS_3` |
| 红方 | True KFS | 15 | `RedTrueKFS01` ~ `RedTrueKFS15` |
| 红方 | Fake KFS | 15 | `RedFakeKFS01` ~ `RedFakeKFS15` |
| 蓝方 | R1 KFS | 3 | `BlueR1KFS`, `BlueR1KFS_2`, `BlueR1KFS_3` |
| 蓝方 | True KFS | 15 | `BlueTrueKFS01` ~ `BlueTrueKFS15` |
| 蓝方 | Fake KFS | 15 | `BlueFakeKFS01` ~ `BlueFakeKFS15` |

### 梅林布局图

从启动区视角看：

```

+----+----+----+
| 10 | 11 | 12 |   ← 靠近九宫格
+----+----+----+
|  7 |  8 |  9 |
+----+----+----+
|  4 |  5 |  6 |
+----+----+----+
|  1 |  2 |  3 |   ← 靠近启动区
+----+----+----+
```

> **注意**:
> - R1 KFS 只能放置在**边缘位置**: 1, 2, 3, 4, 6, 7, 9, 10, 11, 12
> - Fake KFS **禁止**放置在入口位置: 1, 2, 3