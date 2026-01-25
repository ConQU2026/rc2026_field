#!/usr/bin/env python3
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import threading
import time
import signal
import sys
import yaml


class FieldGUI(Node):
    """场地控制 GUI 节点。

    提供图形化界面用于控制场上的 KFS 模型，包括重置、放置、移除以及模式切换。

    Attributes:
        root: ttkbootstrap Window 主窗口实例。
        pub_event_: 发布 GUI 事件的 ROS Publisher。
        sub_status_: 订阅场上状态的 ROS Subscriber。
        client_reset_: 调用重置服务的 ROS Client。
        status_data_: 本地缓存的场上状态数据。
        selected_team_: 当前选中的队伍 ('red', 'blue', 'none')。
        buttons_: 按钮控件字典 {desc: widget}。
    """

    def __init__(self, root):
        super().__init__('field_gui')
        self.root = root
        self.root.title("Robocon 2026 field Controller")
        
        # ROS 通信接口
        self.pub_event_ = self.create_publisher(String, '/simulation/gui_event', 10)
        self.sub_status_ = self.create_subscription(String, '/simulation/status', self.update_status, 10)
        self.client_reset_ = self.create_client(Trigger, '/simulation/reset_kfs')
        
        # 状态初始化
        self.status_data_ = {"red_weapon_count": 0, "blue_weapon_count": 0, "placements": {}, "current_seed": -1}
        self.selected_team_ = ttk.StringVar(value="none") 
        self.buttons_ = {}
        
        # 加载配置文件获取初始值
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if config_path:
            try:
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                self.status_data_['red_weapon_count'] = config.get('red_weapon_count', 3)
                self.status_data_['blue_weapon_count'] = config.get('blue_weapon_count', 3)
                self.status_data_['current_seed'] = config.get('meilin_seed', -1)
                self.get_logger().info(f"已加载初始配置: 种子={self.status_data_['current_seed']}")
            except Exception as e:
                self.get_logger().warn(f"加载配置失败: {e}")
        
        # 构建界面
        self.create_widgets()
        
    def create_widgets(self):
        """创建 GUI 组件布局。"""
        # 顶部控制面板
        control_frame = ttk.LabelFrame(self.root, text="系统控制")
        control_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Button(control_frame, text="重置 / 生成", command=self.call_reset, bootstyle=WARNING).pack(side="left", padx=5, pady=5)
        ttk.Button(control_frame, text="刷新配置", command=self.call_refresh_config, bootstyle=INFO).pack(side="left", padx=5, pady=5)
        
        r_init = self.status_data_.get('red_weapon_count', '--')
        b_init = self.status_data_.get('blue_weapon_count', '--')
        seed_init = self.status_data_.get('current_seed', -1)
        
        self.lbl_weapon_red = ttk.Label(control_frame, text=f"红方: {r_init}", font=("Helvetica", 12, "bold"), bootstyle=DANGER)
        self.lbl_weapon_red.pack(side="left", padx=10)
        
        self.lbl_weapon_blue = ttk.Label(control_frame, text=f"蓝方: {b_init}", font=("Helvetica", 12, "bold"), bootstyle=PRIMARY)
        self.lbl_weapon_blue.pack(side="left", padx=10)
        
        # 队伍选择
        team_frame = ttk.LabelFrame(self.root, text="队伍选择")
        team_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Radiobutton(team_frame, text="无", variable=self.selected_team_, value="none").pack(side="left", padx=10)
        ttk.Radiobutton(team_frame, text="红队", variable=self.selected_team_, value="red").pack(side="left", padx=10)
        ttk.Radiobutton(team_frame, text="蓝队", variable=self.selected_team_, value="blue").pack(side="left", padx=10)
        
        # 种子显示和复制按钮
        self.current_seed_value_ = seed_init
        ttk.Button(team_frame, text="复制种子", command=self.copy_seed, bootstyle=SECONDARY).pack(side="right", padx=5, pady=5)
        self.lbl_seed = ttk.Label(team_frame, text=f"种子: {seed_init}", font=("Helvetica", 10), bootstyle=INFO)
        self.lbl_seed.pack(side="right", padx=10)
        
        # 配置 / 模式
        self.is_full_sim = ttk.BooleanVar(value=False)
        ttk.Checkbutton(control_frame, text="全流程仿真模式", variable=self.is_full_sim, command=self.toggle_mode, bootstyle="round-toggle").pack(side="right", padx=5)
        
        # 选项卡区域
        tab_control = ttk.Notebook(self.root)
        self.tab_red = ttk.Frame(tab_control)
        self.tab_blue = ttk.Frame(tab_control)
        self.tab_grid = ttk.Frame(tab_control)
        
        tab_control.add(self.tab_red, text='红方梅林')
        tab_control.add(self.tab_blue, text='蓝方梅林')
        tab_control.add(self.tab_grid, text='九宫格')
        tab_control.pack(expand=1, fill="both")
        
        # 红方梅林 (4行 3列)
        self.build_button_grid(self.tab_red, 4, 3, "red_meilin", start_id=1)
        
        # 蓝方梅林 (4行 3列)
        self.build_button_grid(self.tab_blue, 4, 3, "blue_meilin", start_id=1)
        
        # 九宫格 (3x3)
        self.build_button_grid(self.tab_grid, 3, 3, "grid", start_id=1)

    def build_button_grid(self, parent, rows, cols, prefix, start_id=1):

        frame = ttk.Frame(parent)
        frame.pack(expand=True, fill="both", padx=10, pady=10)
        
        for r in range(rows):
            frame.grid_rowconfigure(r, weight=1)
            for c in range(cols):
                frame.grid_columnconfigure(c, weight=1)
                
                actual_row = rows - 1 - r  
                idx = start_id + (actual_row * cols + c)
                desc = f"{prefix}_{idx}"
                
                btn = ttk.Button(frame, text=f"空\n({idx})", bootstyle=SECONDARY)
                btn.configure(width=10) 
                btn.grid(row=r, column=c, padx=4, pady=4, sticky="nsew", ipady=10)
                
                btn.configure(command=lambda d=desc: self.on_item_click(d))
                
                self.buttons_[desc] = btn

    def on_item_click(self, desc):
        team = self.selected_team_.get()
        is_grid = desc.startswith("grid")
        
        if is_grid and team == "none":
            self.get_logger().warn("警告: 请先选择一支队伍，然后再操作九宫格!")
            return

        model = self.status_data_['placements'].get(desc)
        
        if model:
            # 该位置已有模型 -> 移除
            msg = {"action": "remove", "target": desc, "team": team if team != 'none' else None}
            self.pub_event_.publish(String(data=json.dumps(msg)))
        else:
            # 该位置为空 -> 放置 
            if is_grid:
                msg = {"action": "place", "target": desc, "team": team}
                self.pub_event_.publish(String(data=json.dumps(msg)))
            else:
                 self.get_logger().info(f"点击了空位: {desc} (梅林区无法手动放置)")

    def toggle_mode(self):
        """切换仿真模式（全流程/独立）。"""
        val = self.is_full_sim.get()
        msg = {"action": "toggle_mode", "value": val}
        self.pub_event_.publish(String(data=json.dumps(msg)))



    def call_reset(self):
        """调用重置服务。"""
        req = Trigger.Request()
        self.client_reset_.call_async(req)
        self.get_logger().info("已发送重置请求...")

    def call_refresh_config(self):
        """发送刷新配置事件。"""
        msg = {"action": "refresh_config"}
        self.pub_event_.publish(String(data=json.dumps(msg)))
        self.get_logger().info("已发送刷新配置请求...")

    def copy_seed(self):
        """复制当前种子到剪贴板。"""
        self.root.clipboard_clear()
        self.root.clipboard_append(str(self.current_seed_value_))
        self.get_logger().info(f"已复制种子: {self.current_seed_value_}")


    def update_status(self, msg):
        """根据 ROS 消息更新 GUI 状态。"""
        try:
            data = json.loads(msg.data)
            self.status_data_ = data
            
            # 同步模式状态
            if "full_simulation_mode" in data:
                if self.is_full_sim.get() != data["full_simulation_mode"]:
                    self.is_full_sim.set(data["full_simulation_mode"])
            
            # 更新武器显示
            r_count = data.get("red_weapon_count", 0)
            b_count = data.get("blue_weapon_count", 0)
            seed = data.get("current_seed", -1)
            self.lbl_weapon_red.config(text=f"红方: {r_count}")
            self.lbl_weapon_blue.config(text=f"蓝方: {b_count}")
            if seed != -1:
                self.current_seed_value_ = seed
                self.lbl_seed.config(text=f"种子: {seed}")
            
            placements = data.get("placements", {})
            
            # 更新所有按钮的显示
            for desc, btn in self.buttons_.items():
                model = placements.get(desc)
                if model:
                    display_text = model
                    if "R1" in model: display_text = "R1 KFS"
                    elif "True" in model: display_text = "R2 KFS"
                    elif "Fake" in model: display_text = "FAKE KFS"
                    
                    style_type = SECONDARY
                    if "Red" in model: style_type = DANGER
                    elif "Blue" in model: style_type = PRIMARY
                    
                    btn.config(text=f"{display_text}\n({desc.split('_')[-1]})", bootstyle=style_type)
                else:
                    # 空位状态
                    idx = desc.split('_')[-1]
                    btn.config(text=f"空\n({idx})", bootstyle=SECONDARY)
                    
        except Exception as e:
            self.get_logger().error(f"更新状态时出错: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    root = ttk.Window(themename="cosmo")
    app = FieldGUI(root)
    
    # 在独立线程中运行 ROS 循环，避免阻塞 GUI
    ros_thread = threading.Thread(target=lambda: rclpy.spin(app), daemon=True)
    ros_thread.start()
    
    def quit_app(*args):
        print("\n正在关闭应用程序...")
        try:
            root.destroy()
        except:
            pass
            
        try:
            rclpy.shutdown()
        except:
            pass
            
        sys.exit(0)

    root.protocol("WM_DELETE_WINDOW", quit_app)
    signal.signal(signal.SIGINT, quit_app)
    def check_signals():
        root.after(100, check_signals)

    root.after(100, check_signals)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        quit_app()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
