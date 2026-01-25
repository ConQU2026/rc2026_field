#!/usr/bin/env python3
import os
import random
import argparse
import math

# ============== 基础坐标配置 ==============
# 九宫格基准点 (2号位 - 底层中间)
# 坐标系: X=横向(Left=+X), Y=深度(Depth), Z=高度
BASE_X: float = -4.8    # 2号位X坐标
BASE_Y: float = 0.0     # 2号位Y坐标
BASE_Z_BOTTOM: float = 1.0  # 底层Z坐标

# ============== 尺寸配置 ==============
PITCH_X: float = 0.54   # 列间距 (横向)
PITCH_Z: float = 0.54   # 层间距 (高度)

# ============== 随机化配置 ==============
# 随机偏移范围 (半宽)
RANDOM_RANGE_X: float = 0.25   # X轴 (横向) ±0.25m
RANDOM_RANGE_Y: float = 0.13   # Y轴 (深度) ±0.13m


# ============== 输出文件配置 ==============
DEFAULT_OUTPUT_PATH: str = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
    "worlds/robocon2026_grid.world"
)

WORLD_HEADER = """<sdf version='1.7'>
  <world name='robocon2026_grid_world'>
    <light name='sun_main1' type='directional'>
      <pose>0 0 20 0 -0 0</pose>
      <diffuse>255 255 255 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>0.5 0.5 -1</direction>
      <cast_shadows>1</cast_shadows>
      <attenuation>
        <range>1000</range>
        <constant>0.01</constant>
        <linear>0.001</linear>
        <quadratic>0.00</quadratic>
      </attenuation>
    </light>
    <light name='sun_main2' type='directional'>
      <pose>0 0 20 0 -0 0</pose>
      <diffuse>255 255 255 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.5 -1</direction>
      <cast_shadows>1</cast_shadows>
      <attenuation>
        <range>1000</range>
        <constant>0.01</constant>
        <linear>0.001</linear>
        <quadratic>0.00</quadratic>
      </attenuation>
    </light>
    <light name='sun_main3' type='directional'>
      <pose>0 0 20 0 -0 0</pose>
      <diffuse>255 255 255 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>0.5 -0.5 -1</direction>
      <cast_shadows>1</cast_shadows>
      <attenuation>
        <range>1000</range>
        <constant>0.01</constant>
        <linear>0.001</linear>
        <quadratic>0.00</quadratic>
      </attenuation>
    </light>
    <light name='sun_main4' type='directional'>
      <pose>0 0 20 0 -0 0</pose>
      <diffuse>255 255 255 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 -0.5 -1</direction>
      <cast_shadows>1</cast_shadows>
      <attenuation>
        <range>1000</range>
        <constant>0.01</constant>
        <linear>0.001</linear>
        <quadratic>0.00</quadratic>
      </attenuation>
    </light>
    <scene>
      <ambient>0.2 0.2 0.2 1</ambient>
      <background>0.4 0.4 0.4 1</background>
      <shadows>1</shadows>
    </scene>
    <scene>
      <ambient>0.2 0.2 0.2 1</ambient>
      <background>0.4 0.4 0.4 1</background>
      <shadows>1</shadows>
    </scene>
    <include>
      <uri>model://robocon2026_world</uri>
      <name>robocon2026_field</name>
      <pose>0 0 0.01 0 -0 0</pose>
    </include>
"""

WORLD_FOOTER = """
  </world>
</sdf>
"""

# Include 模板
INCLUDE_TEMPLATE = """    <include>
      <uri>model://{model_name}</uri>
      <name>{instance_name}</name>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
    </include>"""


def get_kfs_pose(index: int):
    """
    根据序号(1-9)获取KFS的随机坐标
    """
    # Z轴基准 (底层)
    base_z_levels = {
        0: BASE_Z_BOTTOM,                # Row 1 (1,2,3)
        1: BASE_Z_BOTTOM + PITCH_Z,      # Row 2 (4,5,6)
        2: BASE_Z_BOTTOM + PITCH_Z * 2   # Row 3 (7,8,9)
    }
    
    # 映射表: Index -> (Row, Col_Offset)
    # Row: 0=Bottom, 1=Mid, 2=Top
    # Col_Offset: +1 (Left/Pos1), 0 (Center/Pos2), -1 (Right/Pos3)
    # 依据: "1在2左边, x比2大" -> Left对应 +X方向偏移
    layout = {
        1: (0, 1), 2: (0, 0), 3: (0, -1),
        4: (1, 1), 5: (1, 0), 6: (1, -1),
        7: (2, 1), 8: (2, 0), 9: (2, -1)
    }
    
    if index not in layout:
        return None
        
    row, col_offset = layout[index]
    
    # Calculate Center (基准中心)
    center_x = BASE_X + (col_offset * PITCH_X)
    center_y = BASE_Y 
    center_z = base_z_levels[row]
    
    # Apply Random (应用随机偏移)
    offset_x = random.uniform(-RANDOM_RANGE_X, RANDOM_RANGE_X)
    offset_y = random.uniform(-RANDOM_RANGE_Y, RANDOM_RANGE_Y)
    
    final_x = center_x + offset_x
    final_y = center_y + offset_y
    
    return (final_x, final_y, center_z)


def check_win(positions: set) -> bool:
    """
    检查是否存在三连珠 (行、列、对角线)
    """
    winning_lines = [
        {1, 2, 3}, {4, 5, 6}, {7, 8, 9},  # Rows
        {1, 4, 7}, {2, 5, 8}, {3, 6, 9},  # Cols
        {1, 5, 9}, {3, 5, 7}              # Diagonals
    ]
    
    for line in winning_lines:
        if line.issubset(positions):
            return True
    return False


def generate_team_inventory(team: str) -> list:
    """
    生成队伍的可用KFS库存
    规则: 3个 R1KFS + 4个 TrueKFS (从01-15中随机选4个)
    总计: 7个
    """
    inventory = []
    
    # 1. 添加 3个 R1KFS
    for _ in range(3):
        inventory.append(f"{team}R1KFS")
        
    # 2. 添加 4个 TrueKFS 
    true_ids = random.sample(range(1, 16), 4)
    for tid in true_ids:
        inventory.append(f"{team}TrueKFS{tid:02d}")
    random.shuffle(inventory)
    return inventory


def generate_world_file(output_path: str, seed: int = None, red_count: int = 4, blue_count: int = 4):
    """
    生成World文件 (带验证逻辑和库存限制)
    """
    if seed is not None:
        random.seed(seed)
        print(f"Using random seed: {seed}")

    # 1. 验证数量限制
    # 每个队伍最多只有7个KFS (3 R1 + 4 True)
    MAX_KFS_PER_TEAM = 7
    if red_count > MAX_KFS_PER_TEAM:
        print(f"Error: Red count ({red_count}) exceeds max inventory ({MAX_KFS_PER_TEAM}).")
        return
    if blue_count > MAX_KFS_PER_TEAM:
        print(f"Error: Blue count ({blue_count}) exceeds max inventory ({MAX_KFS_PER_TEAM}).")
        return

    total_kfs = red_count + blue_count
    if total_kfs > 9:
        print(f"Error: Total KFS ({total_kfs}) exceeds grid size (9).")
        return

    # 2. 准备库存 (从有限池中抽取)
    # 为红方和蓝方分别生成完整的7个候选池，然后从中取出所需的数量
    red_full_inventory = generate_team_inventory("Red")
    blue_full_inventory = generate_team_inventory("Blue")
    
    # 截取所需数量的模型列表
    red_models = red_full_inventory[:red_count]
    blue_models = blue_full_inventory[:blue_count]

    # 3. 尝试生成满足条件的布局
    max_attempts = 1000
    valid_placement = False
    red_positions = set()
    blue_positions = set()
    
    for attempt in range(max_attempts):
        all_indices = list(range(1, 10))
        selected_indices = random.sample(all_indices, total_kfs)
        
        # 分配红蓝位置
        current_red = set(selected_indices[:red_count])
        current_blue = set(selected_indices[red_count:])
        
        # 验证赢棋条件 (双方都不能赢)
        if not check_win(current_red) and not check_win(current_blue):
            red_positions = current_red
            blue_positions = current_blue
            valid_placement = True
            break
            
    if not valid_placement:
        print(f"Error: Could not generate a non-winning placement after {max_attempts} attempts.")
        print("Try reducing KFS counts or check if constraints are impossible.")
        return

    print(f"Placement generated successfully (Attempts: {attempt + 1})")
    print(f"Red Pos: {red_positions}, Blue Pos: {blue_positions}")

    includes = []
    
    print("-" * 60)
    print(f"{'ID':<4} {'Pos Desc':<12} {'Team':<6} {'Model':<20} {'X':<8} {'Y':<8} {'Z':<8}")
    print("-" * 60)
    instance_counter = {}

    assignments = []
    
    red_pos_list = list(red_positions)
    blue_pos_list = list(blue_positions)
    
    # 随机打乱位置以避免模型顺序与位置ID相关联
    random.shuffle(red_pos_list)
    random.shuffle(blue_pos_list)
    
    for i, pos_id in enumerate(red_pos_list):
        assignments.append((pos_id, "Red", red_models[i]))
        
    for i, pos_id in enumerate(blue_pos_list):
        assignments.append((pos_id, "Blue", blue_models[i]))
        
    # 按位置ID排序，方便打印
    assignments.sort(key=lambda x: x[0])

    for i, team, model_name in assignments:
        pos = get_kfs_pose(i)
        if not pos:
            continue
            
        x, y, z = pos
        
        # 处理 Instance Name
        if model_name not in instance_counter:
            instance_counter[model_name] = 1
            instance_name = f"{model_name}" 
            # 加上grid后缀以防万一
            instance_name = f"{model_name}_grid_{i}" 
        else:
            instance_counter[model_name] += 1
            instance_name = f"{model_name}_grid_{i}_{instance_counter[model_name]}"

        # 描述
        layout = {
            1: "Bot-Left", 2: "Bot-Mid", 3: "Bot-Right",
            4: "Mid-Left", 5: "Mid-Mid", 6: "Mid-Right",
            7: "Top-Left", 8: "Top-Mid", 9: "Top-Right"
        }
        desc = layout.get(i, "Unknown")
        
        print(f"{i:<4} {desc:<12} {team:<6} {model_name:<20} {x:<8.3f} {y:<8.3f} {z:<8.3f}")

        # 添加到 XML
        includes.append(INCLUDE_TEMPLATE.format(
            model_name=model_name,
            instance_name=instance_name,
            x=x,
            y=y,
            z=z
        ))

    # 写入文件
    world_content = WORLD_HEADER + "\n".join(includes) + WORLD_FOOTER
    
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(world_content)

    print("-" * 60)
    print(f"World file generated: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='RenderGrid: Generate 9-grid KFS World (No-Win Constraints)')
    parser.add_argument('-o', '--output', type=str, default=DEFAULT_OUTPUT_PATH,
                        help='Output world file path')
    parser.add_argument('-s', '--seed', type=int, default=None,
                        help='Random seed')
    parser.add_argument('--red', type=int, default=4,
                        help='Number of Red KFS (Default: 4)')
    parser.add_argument('--blue', type=int, default=4,
                        help='Number of Blue KFS (Default: 4)')
    
    args = parser.parse_args()
    
    generate_world_file(args.output, args.seed, args.red, args.blue)



if __name__ == "__main__":
    main()
