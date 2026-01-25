## 1. 场地控制与反馈

### 发送指令
向系统发送控制指令，放置或移除 KFS。

*   **Topic**: `/simulation/gui_event`
*   **Type**: `std_msgs/msg/String`
*   **Format**: JSON 字符串

#### 示例:

1.  **放置 KFS (九宫格) **
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

3.  **拾取 KFS (梅林区)**
    ```json
    {
      "action": "remove",
      "team": "red",
      "target": "red_meilin_1" // 梅林区位置 (red_meilin_1 ~ 12)
    }
    ```
    *效果: 将该 KFS 收入"已拾取"列表，供全流程模式下的放置使用。*

4.  **切换仿真模式**
    ```json
    {
      "action": "toggle_mode",
      "value": true // true: 全流程模式 (需拾取), false: 独立模式(无需验证是否拾取KFS) 
    }
    ```

### 获取状态 (Feedback)
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
