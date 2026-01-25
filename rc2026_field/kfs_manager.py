#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from gazebo_msgs.srv import SetEntityState
import math
from scipy.spatial.transform import Rotation
import os
import yaml
import json
import random
import time
from ament_index_python.packages import get_package_share_directory

class KFSManager(Node):
    """KFS 管理节点。

    负责管理场地上的 KFS 状态，包括初始化布局、处理 GUI 事件（放置、移除）以及重置仿真。

    Attributes:
        config_file: 配置文件路径。
        red_weapon_count_: 红方剩余武器数量。
        blue_weapon_count_: 蓝方剩余武器数量。
        full_simulation_mode_: 是否开启全流程仿真模式。
        placements_: 当前场地上的 KFS 布局记录 {location_desc: model_name}。
        picked_models_: 已被取走(回到初始位置待命)的KFS集合。
        destroyed_models_: 已被摧毁的KFS集合。
    """

    def __init__(self):
        super().__init__('kfs_manager')
        
        # 加载配置
        self.declare_parameter('config_path', '')
        config_path_param = self.get_parameter('config_path').get_parameter_value().string_value
        self.config_file = config_path_param

        self.get_logger().info(f"正在加载配置文件: {self.config_file}")
        self.load_config()

        # 状态初始化
        self.red_weapon_count_ = self.config.get('red_weapon_count', 3)
        self.blue_weapon_count_ = self.config.get('blue_weapon_count', 3)
        self.full_simulation_mode_ = self.config.get('full_simulation_mode', False)
        
        self.placements_ = {} 
        self.picked_models_ = set() 
        self.destroyed_models_ = set() 
        self.current_meilin_selection_ = {'red': [], 'blue': []} 

        # ROS 接口
        self.client_set_entity_state_ = self.create_client(SetEntityState, '/set_entity_state')
        self.srv_reset_ = self.create_service(Trigger, '/simulation/reset_kfs', self.handle_reset)
        self.sub_gui_event_ = self.create_subscription(String, '/simulation/gui_event', self.handle_gui_event, 10)
        self.pub_status_ = self.create_publisher(String, '/simulation/status', 10)

        self.create_timer(1.0, self.publish_status)

        self.get_logger().info("KFS 管理节点已启动")

    def load_config(self):
        """加载或重载 YAML 配置文件。"""
        with open(self.config_file, 'r') as f:
            self.config = yaml.safe_load(f)

    def publish_status(self):
        """发布当前仿真状态，供 GUI 显示。"""
        status = {
            "red_weapon_count": self.red_weapon_count_,
            "blue_weapon_count": self.blue_weapon_count_,
            "full_simulation_mode": self.full_simulation_mode_,
            "placements": self.placements_
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status_.publish(msg)

    def handle_reset(self, request, response) -> Trigger.Response:
        """处理重置服务请求。

        重置所有状态,重新生成随机布局,并将KFS移动到初始位置。

        Args:
            request: Trigger 请求。
            response: Trigger 响应。

        Returns:
            Trigger 响应结果。
        """
        self.get_logger().info("正在重置 KFS 仿真环境...")
        self.load_config()
        self.red_weapon_count_ = self.config.get('red_weapon_count', 5)
        self.blue_weapon_count_ = self.config.get('blue_weapon_count', 5)
        
        if not self.client_set_entity_state_.service_is_ready():
            self.get_logger().warn("Gazebo set_entity_state 服务未就绪")
        
        models_cfg = self.config.get('models', {})
        if not models_cfg:
            response.success = False
            response.message = "配置文件中未找到KFS定义"
            return response
            
        inventory = {'red': [], 'blue': []}
        for m in models_cfg.keys():
            if 'Red' in m: inventory['red'].append(m)
            elif 'Blue' in m: inventory['blue'].append(m)
        self.get_logger().info(f"库存统计: 红方={len(inventory['red'])}, 蓝方={len(inventory['blue'])}")

        # 生成新布局
        self.get_logger().info("生成随机布局中...")
        assignments = self.generate_new_layout(inventory)
        
        # 计算需要归位的KFS
        prev_models = set(self.placements_.values())
        new_models = set(item['model'] for item in assignments)
        models_to_home = prev_models - new_models
        
        if models_to_home:
            self.get_logger().debug(f"将 {len(models_to_home)} 个残留KFS移回初始位置...")
            for model_name in models_to_home:
                home = models_cfg.get(model_name)
                if home:
                    self.move_model(model_name, home[0], home[1], home[2])
                    time.sleep(0.02)
            time.sleep(0.3) 
        
        # 清空状态
        self.placements_.clear()
        self.picked_models_.clear()
        self.destroyed_models_.clear()
        
        # 移动新分配的KFS
        self.get_logger().info(f"正在放置 {len(assignments)} 个 KFS...")
        for item in assignments:
            self.move_model(item['model'], item['x'], item['y'], item['z'])
            time.sleep(0.02)
            
        self.placements_ = {item['desc']: item['model'] for item in assignments}
        
        # 记录本轮梅林区选中的KFS
        self.current_meilin_selection_ = {'red': [], 'blue': []}
        for item in assignments:
            m = item['model']
            if 'Red' in m: self.current_meilin_selection_['red'].append(m)
            elif 'Blue' in m: self.current_meilin_selection_['blue'].append(m)
        
        result_msg = f"重置完成，已重新放置 {len(assignments)} 个KFS。"
        self.get_logger().info(result_msg)
        response.success = True
        response.message = result_msg
        return response

    def handle_gui_event(self, msg):
        """处理来自 GUI 的事件消息。

        Args:
            msg: 包含 JSON 数据的 String 消息。
        """
        try:
            data = json.loads(msg.data)
            action = data.get('action')
            team = data.get('team', None) 
            
            if action == 'toggle_mode':
                self.full_simulation_mode_ = data.get('value', False)
                mode_str = "全流程仿真模式" if self.full_simulation_mode_ else "独立测试模式"
                self.get_logger().info(f"模式已切换: {mode_str}")
                self.publish_status()
                return

            if action == 'refresh_config':
                self.get_logger().info("收到刷新配置请求...")
                
                self.load_config()
                # 更新关键参数
                self.red_weapon_count_ = self.config.get('red_weapon_count', 3)
                self.blue_weapon_count_ = self.config.get('blue_weapon_count', 3)
                self.full_simulation_mode_ = self.config.get('full_simulation_mode', False)
                
                self.get_logger().info(f"配置已重载 (文件: {self.config_file})")
                self.publish_status()
                return

            if action == 'place':
                self._handle_place_action(data, team)

            elif action == 'remove':
                self._handle_remove_action(data, team)

        except Exception as e:
            self.get_logger().error(f"处理 GUI 事件时发生错误: {e}") 

    def _handle_place_action(self, data, team):
        """处理放置动作。"""
        target_grid_desc = data.get('target', '')
        
        if not team:
            self.get_logger().warn("请先选择队伍再进行放置")
            return
        if self.placements_.get(target_grid_desc):
            self.get_logger().warn(f"目标位置 {target_grid_desc} 已被占用")
            return
        
        model_name = None

        # 根据模式选择可用的KFS
        if self.full_simulation_mode_:
            # 模式 A: 全流程 - 只能从picked_models中选择
            candidates = []
            for m in self.picked_models_:
                if "Fake" in m: continue
                if team == 'red' and 'Red' in m: candidates.append(m)
                elif team == 'blue' and 'Blue' in m: candidates.append(m)
            
            if not candidates:
                 self.get_logger().warn(f"{team}: 没有可用的已拾取 KFS! 请先从梅林区拾取。")
                 return
            
            model_name = random.choice(candidates)
            self.picked_models_.remove(model_name)
            
        else:
            # 模式 B: 独立/调试 - 从本轮梅林区选中的KFS中选择
            potential_models = self.current_meilin_selection_.get(team, [])
            potential_models = [m for m in potential_models if m not in self.destroyed_models_]
            potential_models = [m for m in potential_models if "Fake" not in m]
            grid_placed_models = [v for k,v in self.placements_.items() if k.startswith('grid')]
            candidates = [m for m in potential_models if m not in grid_placed_models]
                          
            if not candidates:
                self.get_logger().warn(f"{team}: 没有可用的梅林 KFS 可放置到九宫格")
                return

            model_name = random.choice(candidates)
            
            # 自动处理源状态清理
            current_loc_key = next((k for k, v in self.placements_.items() if v == model_name), None)
            if current_loc_key:
                del self.placements_[current_loc_key]
                
            if model_name in self.picked_models_:
                self.picked_models_.remove(model_name)

        # 执行移动
        grid_idx = int(target_grid_desc.split('_')[-1])
        gx, gy, gz, gyaw = self.get_grid_pose(grid_idx)
        
        self.move_model(model_name, gx, gy, gz, gyaw)
        
        self.placements_[target_grid_desc] = model_name
        
        mode_str = "全流程" if self.full_simulation_mode_ else "独立"
        self.get_logger().info(f"已移动 {model_name} 至 {target_grid_desc} (模式: {mode_str})")

    def _handle_remove_action(self, data, team):
        """处理移除动作。"""
        target_desc = data.get('target', '')
        is_grid = target_desc.startswith('grid')
        
        # 移除九宫格上的物体需要消耗武器
        if is_grid:
             if not team:
                 self.get_logger().warn("请先选择队伍再进行攻击!")
                 return
             current_count = self.red_weapon_count_ if team == 'red' else self.blue_weapon_count_
             if current_count <= 0:
                 self.get_logger().warn(f"{team} 武器已耗尽!")
                 return

        model_name = self.placements_.get(target_desc)
        if not model_name: 
            self.get_logger().warn(f"目标位置 {target_desc} 为空")
            return

        # 逻辑处理
        if is_grid:

            is_red_model = 'Red' in model_name
            is_blue_model = 'Blue' in model_name
            if (team == 'red' and is_red_model) or (team == 'blue' and is_blue_model):
                 self.get_logger().warn(f"无法攻击己方 KFS ({model_name})!")
                 return
            
            # 判定摧毁
            self.destroyed_models_.add(model_name)
            
            # 扣除武器
            if team == 'red': self.red_weapon_count_ -= 1
            else: self.blue_weapon_count_ -= 1

        else:
            # 梅林区移除 
            if "Fake" in model_name:
                self.get_logger().warn(f"无法拾取FAKE KFS ({model_name})!")
                return
                
            self.picked_models_.add(model_name)
        
        # 移回初始位置 
        models_cfg = self.config.get('models', {})
        home_pos = models_cfg.get(model_name)
        
        if home_pos:
            self.move_model(model_name, home_pos[0], home_pos[1], home_pos[2])
            
            # 更新状态
            keys_to_remove = [k for k, v in self.placements_.items() if v == model_name]
            for k in keys_to_remove:
                del self.placements_[k]
                
            self.get_logger().info(f"已移除 {model_name}。位置: {target_desc}。")
        else:
            self.get_logger().error(f"未找到 {model_name} 的初始位置配置")

    def generate_new_layout(self, inventory):
        """生成随机 KFS 布局。
        
        - 每队 8 个 KFS 放在 12 个方块上
        - 3 个 R1 KFS(只能放在边缘方块 1,2,3,4,6,7,9,10,11,12)
        - 4 个 R2 True KFS(随机选择)
        - 1 个 Fake KFS(禁止放在入口方块 1,2,3)
        
        Args:
            inventory: 按队伍分类的KFS名称字典
            
        Returns:
            assignments 列表，每项包含 model, x, y, z, desc
        """
        assignments = []
        meilin_cfg = self.config['meilin']
        
        # 分类KFS工具函数
        def classify_models(model_list):
            r1 = [m for m in model_list if 'R1KFS' in m]
            true_kfs = [m for m in model_list if 'TrueKFS' in m]
            fake = [m for m in model_list if 'FakeKFS' in m]
            random.shuffle(r1)
            random.shuffle(true_kfs)
            random.shuffle(fake)
            return r1, true_kfs, fake
        
        red_r1, red_true, red_fake = classify_models(inventory['red'])
        blue_r1, blue_true, blue_fake = classify_models(inventory['blue'])
        
        # 梅林区生成常量
        R1_ALLOWED_BLOCKS = [1, 2, 3, 4, 6, 7, 9, 10, 11, 12]  # 边缘方块
        FAKE_FORBIDDEN_BLOCKS = [1, 2, 3]  # 入口方块
        
        def place_meilin_team(team, r1_list, true_list, fake_list, coords_map):
            """为单个队伍生成梅林区 KFS 放置方案。"""
            used_blocks = set()
            placements = []
            
            all_blocks = list(range(1, 13))
            random.shuffle(all_blocks)
            
            # 1. 放置 R1 KFS
            num_r1 = min(len(r1_list), 3)
            r1_candidates = [b for b in all_blocks if b in R1_ALLOWED_BLOCKS]
            random.shuffle(r1_candidates)
            for i in range(num_r1):
                if r1_candidates:
                    bid = r1_candidates.pop()
                    used_blocks.add(bid)
                    placements.append((r1_list[i], bid))
            
            # 2. 放置 Fake KFS
            num_fake = min(len(fake_list), 1)
            fake_candidates = [b for b in all_blocks 
                              if b not in FAKE_FORBIDDEN_BLOCKS and b not in used_blocks]
            random.shuffle(fake_candidates)
            for i in range(num_fake):
                if fake_candidates:
                    bid = fake_candidates.pop()
                    used_blocks.add(bid)
                    placements.append((fake_list[i], bid))
            
            # 3. 放置 True KFS
            num_true = min(len(true_list), 4)
            true_candidates = [b for b in all_blocks if b not in used_blocks]
            random.shuffle(true_candidates)
            for i in range(num_true):
                if true_candidates:
                    bid = true_candidates.pop()
                    used_blocks.add(bid)
                    placements.append((true_list[i], bid))
            
            # 转换为坐标
            for model, bid in placements:
                pos = coords_map.get(bid) or coords_map.get(str(bid))
                if pos:
                    assignments.append({
                        'model': model,
                        'x': pos[0], 'y': pos[1], 'z': pos[2],
                        'desc': f'{team}_meilin_{bid}'
                    })
                    self.get_logger().debug(f"{team} 放置: {model} -> 方块 {bid}")
                else:
                    self.get_logger().error(f"未找到 {team} 方块 {bid} 的坐标配置")
        
        place_meilin_team('red', red_r1, red_true, red_fake, meilin_cfg['red'])
        place_meilin_team('blue', blue_r1, blue_true, blue_fake, meilin_cfg['blue'])
        
        return assignments

    def get_grid_pose(self, index):
        """计算九宫格指定位置的坐标。

        Args:
            index: 九宫格索引 (1-9)。

        Returns:
            (x, y, z, yaw) 坐标元组。
        """
        g = self.config['grid']
        base_z = {
            0: g['base_z_bottom'],
            1: g['base_z_bottom'] + g['pitch_z'],
            2: g['base_z_bottom'] + g['pitch_z'] * 2
        }
        layout = {
            1: (0, 1), 2: (0, 0), 3: (0, -1),
            4: (1, 1), 5: (1, 0), 6: (1, -1),
            7: (2, 1), 8: (2, 0), 9: (2, -1)
        }
        
        if index not in layout: return 0,0,0,0
        
        row, col_offset = layout[index]
        cx = g['base_x'] + (col_offset * g['pitch_x'])
        cy = g['base_y']
        cz = base_z[row]
        
        # 随机扰动
        ox = random.uniform(-g['random_range_x'], g['random_range_x'])
        oy = random.uniform(-g['random_range_y'], g['random_range_y'])
        
        # 随机偏航角
        yaw_range_deg = g.get('random_range_yaw_deg', 0.0)
        yaw_rad = 0.0
        if yaw_range_deg > 0:
            yaw_deg = random.uniform(-yaw_range_deg, yaw_range_deg)
            yaw_rad = math.radians(yaw_deg)

        return cx + ox, cy + oy, cz, yaw_rad

    def move_model(self, model_name, x, y, z, yaw=0.0):
        """调用 Gazebo 服务移动KFS。

        Args:
            model_name: KFS名称。
            x, y, z: 目标位置坐标。
            yaw: 偏航角 (弧度)，默认为 0。
        """
        req = SetEntityState.Request()
        req.state.name = model_name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)

        r = Rotation.from_euler('xyz', [0, 0, yaw])
        q = r.as_quat()
        
        req.state.pose.orientation.x = float(q[0])
        req.state.pose.orientation.y = float(q[1])
        req.state.pose.orientation.z = float(q[2]) 
        req.state.pose.orientation.w = float(q[3]) 
        
        future = self.client_set_entity_state_.call_async(req)
        
        def done_callback(future):
            try:
                result = future.result()
                if result and result.success:
                    self.get_logger().debug(f"成功移动 {model_name}")
                else:
                    self.get_logger().warn(f"移动 {model_name} 失败: {result}")
            except Exception as e:
                self.get_logger().error(f"移动 {model_name} 时发生异常: {e}")
        
        future.add_done_callback(done_callback)

def main(args=None):
    rclpy.init(args=args)
    node = KFSManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
