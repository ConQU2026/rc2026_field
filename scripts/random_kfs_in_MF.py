#!/usr/bin/env python3
"""
ROBOCON 2026 "武林探秘" World Generator
根据官方规则PDF生成随机KFS放置的world文件

支持红方和蓝方KFS生成，支持yaw旋转调整方向
"""

import random
import argparse
import math
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import json

# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              配置参数区域                                      ║
# ║  所有可配置参数都在这里，方便调整                                                ║
# ╚══════════════════════════════════════════════════════════════════════════════╝
import os

# ============== 生成开关 ==============
GENERATE_RED: bool = True       # 是否生成红方KFS
GENERATE_BLUE: bool = True      # 是否生成蓝方KFS


# ============== KFS数量配置 ==============
# 根据规则：每队半场梅林区共放置8个KFS
# - R1 KFS: 3个 (只能放在3x4矩形边缘，贴有大赛Logo)
# - R2 KFS: 4个 (可以放在任意方块，从TrueKFS01-15随机选4个)
# - 假KFS: 1个 (禁止放在入口方块10,11,12，从FakeKFS01-15随机选1个)
NUM_R1_KFS: int = 3             # R1 KFS数量
NUM_R2_KFS: int = 4             # R2 真KFS数量
NUM_FAKE_KFS: int = 1           # 假KFS数量

# R1 KFS只能放置的方块（3x4矩形边缘，排除中间的5和8）
R1_ALLOWED_BLOCKS: List[int] = [1, 2, 3, 4, 6, 7, 9, 10, 11, 12]

# 禁止放置假KFS的方块（入口方块）
FAKE_KFS_FORBIDDEN_BLOCKS: List[int] = [1, 2, 3]


# ============== 树林方块配置 (24个固定坐标) ==============
# 坐标单位：米 (x, y, z) - 最终世界坐标
# 这些坐标已经包含了所有的全局偏移和旋转，生成时直接使用

# 红方树林方块 (12个)
RED_FOREST_BLOCKS: Dict[int, Tuple[float, float, float]] = {
    1: (2.2000, 1.8000, 5.8000),
    2: (2.2000, 3.0000, 5.8000),
    3: (2.2000, 4.2000, 5.8000),
    4: (1.0000, 1.8000, 5.6000),
    5: (1.0000, 3.0000, 5.6000),
    6: (1.0000, 4.2000, 5.6000),
    7: (-0.2000, 1.8000, 5.4000),
    8: (-0.2000, 3.0000, 5.4000),
    9: (-0.2000, 4.2000, 5.4000),
    10: (-1.4000, 1.8000, 5.2000),
    11: (-1.4000, 3.0000, 5.2000),
    12: (-1.4000, 4.2000, 5.2000),
}

# 蓝方树林方块 (12个)
BLUE_FOREST_BLOCKS: Dict[int, Tuple[float, float, float]] = {
    1: (2.2000, -4.2000, 5.8000),
    2: (2.2000, -3.0000, 5.8000),
    3: (2.2000, -1.8000, 5.8000),
    4: (1.0000, -4.2000, 5.6000),
    5: (1.0000, -3.0000, 5.6000),
    6: (1.0000, -1.8000, 5.6000),
    7: (-0.2000, -4.2000, 5.4000),
    8: (-0.2000, -3.0000, 5.4000),
    9: (-0.2000, -1.8000, 5.4000),
    10: (-1.4000, -4.2000, 5.2000),
    11: (-1.4000, -3.0000, 5.2000),
    12: (-1.4000, -1.8000, 5.2000),
}

# ============== 队伍旋转配置 ==============
# 用于设定生成物体的yaw，不再用于计算位置
RED_TEAM_YAW: float = 0.0
BLUE_TEAM_YAW: float = math.pi


# ============== 单独方块偏移配置 ==============
# 红方: "red_1", "red_2", ...
# 蓝方: "blue_1", "blue_2", ...
BLOCK_OFFSETS: Dict[str, Tuple[float, float, float]] = {
    # 示例: "red_1": (0.05, -0.02, 0.01),
    # 示例: "blue_1": (0.05, -0.02, 0.01),
}

# ============== 未使用KFS存放位置 ==============
STORAGE_AREA_X: float = -8.0
STORAGE_AREA_Y_START: float = -4.0
STORAGE_AREA_Y_STEP: float = 0.4
STORAGE_AREA_Z: float = 0.5

# ============== 输出文件配置 ==============
# 默认输出路径
DEFAULT_OUTPUT_PATH: str = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "worlds/robocon2026_random_MF.world")


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              数据结构定义                                      ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

@dataclass
class KFSPlacement:
    """KFS放置信息"""
    model_name: str
    x: float
    y: float
    z: float
    yaw: float              # yaw角度
    block_id: Optional[int]
    is_storage: bool


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              辅助函数                                          ║
# ╚══════════════════════════════════════════════════════════════════════════════╗

def rotate_point(x: float, y: float, yaw: float) -> Tuple[float, float]:
    """
    绕原点(0,0)旋转点坐标

    Args:
        x, y: 原始坐标
        yaw: 旋转角度（弧度），正值为逆时针

    Returns:
        旋转后的(x, y)坐标
    """
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    new_x = x * cos_yaw - y * sin_yaw
    new_y = x * sin_yaw + y * cos_yaw
    return (new_x, new_y)


def get_storage_position(index: int, category: str) -> Tuple[float, float, float]:
    """获取存放区位置"""
    category_x_offset = {
        "red_true": 0.0,
        "red_fake": 0.5,
        "blue_true": 1.0,
        "blue_fake": 1.5,
    }
    
    x = STORAGE_AREA_X + category_x_offset.get(category, 0)
    y = STORAGE_AREA_Y_START + index * STORAGE_AREA_Y_STEP
    z = STORAGE_AREA_Z
    return (x, y, z)


def get_block_position_for_team(
    block_id: int,
    team: str,
) -> Tuple[float, float, float]:
    """获取指定队伍方块的最终位置"""
    if team == "red":
        base_pos = RED_FOREST_BLOCKS.get(block_id, (0, 0, 0))
    else:
        base_pos = BLUE_FOREST_BLOCKS.get(block_id, (0, 0, 0))

    # 获取方块特定偏移 (如果还需要微调的话)
    offset_key = f"{team}_{block_id}"
    block_offset = BLOCK_OFFSETS.get(offset_key, (0, 0, 0))

    final_x = base_pos[0] + block_offset[0]
    final_y = base_pos[1] + block_offset[1]
    final_z = base_pos[2] + block_offset[2]

    return (final_x, final_y, final_z)


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              KFS放置生成器                                     ║
# ╚══════════════════════════════════════════════════════════════════════════════╗


def generate_team_kfs_placement(
    team: str,
    seed: Optional[int] = None
) -> Dict[str, List[KFSPlacement]]:
    """
    生成单个队伍的KFS随机放置方案
    
    规则：
    - R1 KFS (3个): 只能放在3x4边缘方块，使用 RedR1KFS/BlueR1KFS 模型
    - R2 KFS (4个): 可以放在任意方块，从 TrueKFS01-15 随机选4个
    - 假KFS (1个): 禁止放在入口方块10,11,12，从 FakeKFS01-15 随机选1个
    """
    if team == "red":
        yaw = RED_TEAM_YAW
        team_prefix = "Red"
    else:
        yaw = BLUE_TEAM_YAW
        team_prefix = "Blue"

    placements: Dict[str, List[KFSPlacement]] = {
        f"{team}_r1": [],
        f"{team}_true": [],
        f"{team}_fake": [],
    }

    used_blocks = set()

    # 所有12个方块
    all_blocks = list(range(1, 13))
    random.shuffle(all_blocks)

    # 随机选择要使用的TrueKFS编号（从01-15中随机选4个）
    true_kfs_numbers = random.sample(range(1, 16), NUM_R2_KFS)
    # 随机选择要使用的FakeKFS编号（从01-15中随机选1个）
    fake_kfs_numbers = random.sample(range(1, 16), NUM_FAKE_KFS)

    # ========== 1. 放置R1 KFS (3个，只能放在3x4边缘方块) ==========
    # R1 KFS使用 RedR1KFS/BlueR1KFS 模型，同一模型放3次需要不同的实例名
    # 从边缘方块中随机选择NUM_R1_KFS个
    r1_candidates = [b for b in all_blocks if b in R1_ALLOWED_BLOCKS]
    random.shuffle(r1_candidates)

    for i in range(NUM_R1_KFS):
        if r1_candidates:
            block_id = r1_candidates.pop()
            used_blocks.add(block_id)
            pos = get_block_position_for_team(block_id, team)

            placement = KFSPlacement(
                model_name=f"{team_prefix}R1KFS",  # 模型名
                x=pos[0],
                y=pos[1],
                z=pos[2],
                yaw=yaw,
                block_id=block_id,
                is_storage=False
            )
            placements[f"{team}_r1"].append(placement)

    # ========== 2. 放置假KFS (1个，禁止放在入口方块10,11,12) ==========
    # 注意：这里需要从剩余的可用方块中选择，并排除禁止的方块
    fake_candidates = [b for b in all_blocks if b not in FAKE_KFS_FORBIDDEN_BLOCKS and b not in used_blocks]
    random.shuffle(fake_candidates)

    for i in range(NUM_FAKE_KFS):
        if fake_candidates:
            block_id = fake_candidates.pop()
            used_blocks.add(block_id)
            pos = get_block_position_for_team(block_id, team)

            placement = KFSPlacement(
                model_name=f"{team_prefix}FakeKFS{fake_kfs_numbers[i]:02d}",
                x=pos[0],
                y=pos[1],
                z=pos[2],
                yaw=yaw,
                block_id=block_id,
                is_storage=False
            )
            placements[f"{team}_fake"].append(placement)

    # ========== 3. 放置R2 KFS (4个，可以放在任意剩余方块) ==========
    # R2 KFS从TrueKFS01-15中随机选择4个
    r2_candidates = [b for b in all_blocks if b not in used_blocks]
    random.shuffle(r2_candidates)

    for i in range(NUM_R2_KFS):
        if r2_candidates:
            block_id = r2_candidates.pop()
            used_blocks.add(block_id)
            pos = get_block_position_for_team(block_id, team)

            placement = KFSPlacement(
                model_name=f"{team_prefix}TrueKFS{true_kfs_numbers[i]:02d}",
                x=pos[0],
                y=pos[1],
                z=pos[2],
                yaw=yaw,
                block_id=block_id,
                is_storage=False
            )
            placements[f"{team}_true"].append(placement)

    return placements

def generate_all_kfs_placement(
    seed: Optional[int] = None
) -> Dict[str, List[KFSPlacement]]:
    """生成红蓝双方KFS的随机放置方案"""
    if seed is not None:
        random.seed(seed)

    placements: Dict[str, List[KFSPlacement]] = {}

    # 生成红方
    if GENERATE_RED:
        placements.update(generate_team_kfs_placement("red", None))

    # 生成蓝方
    if GENERATE_BLUE:
        placements.update(generate_team_kfs_placement("blue", None))

    return placements


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              World文件生成                                     ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

WORLD_HEADER = """<sdf version='1.7'>
  <world name='robocon2026_world_scene'>
    <model name='ground_plane'>
      <static>1</static>
    </model>
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

INCLUDE_TEMPLATE = """    <include>
      <uri>model://{model_name}</uri>
      <name>{instance_name}</name>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
    </include>"""


def generate_world_file(placements: Dict[str, List[KFSPlacement]], output_path: str):
    """生成world文件"""
    includes = []

    # 所有可能的类别
    all_categories = [
        "red_r1", "red_true", "red_fake",
        "blue_r1", "blue_true", "blue_fake"
    ]

    # 用于跟踪同一模型的实例编号
    instance_counter: Dict[str, int] = {}

    for category in all_categories:
        if category in placements:
            for p in placements[category]:
                # 为同一模型的多个实例生成不同的name
                if p.model_name not in instance_counter:
                    instance_counter[p.model_name] = 1
                    instance_name = p.model_name
                else:
                    instance_counter[p.model_name] += 1
                    instance_name = f"{p.model_name}_{instance_counter[p.model_name]}"

                includes.append(INCLUDE_TEMPLATE.format(
                    model_name=p.model_name,
                    instance_name=instance_name,
                    x=p.x,
                    y=p.y,
                    z=p.z,
                    yaw=p.yaw
                ))

    world_content = WORLD_HEADER + "\n".join(includes) + WORLD_FOOTER

    with open(output_path, 'w') as f:
        f.write(world_content)

    print(f"World file generated: {output_path}")


def print_placement_summary(placements: Dict[str, List[KFSPlacement]]):
    """打印放置摘要"""
    print("\n" + "=" * 70)
    print("KFS Placement Summary")
    print("=" * 70)

    # 所有可能的类别
    all_categories = [
        ("red_r1", "Red R1 KFS"),
        ("red_true", "Red R2 True KFS"),
        ("red_fake", "Red Fake KFS"),
        ("blue_r1", "Blue R1 KFS"),
        ("blue_true", "Blue R2 True KFS"),
        ("blue_fake", "Blue Fake KFS"),
    ]

    for category, name in all_categories:
        if category not in placements or not placements[category]:
            continue

        print(f"\n{name}:")
        on_field = [p for p in placements[category] if not p.is_storage]
        in_storage = [p for p in placements[category] if p.is_storage]

        print(f"  场地上: {len(on_field)}个, 存放区: {len(in_storage)}个")

        for p in placements[category]:
            if p.is_storage:
                location = "存放区"
            elif p.block_id:
                location = f"方块{p.block_id}"
            else:
                location = "起始区"
            print(f"    {p.model_name}: {location} ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) yaw={math.degrees(p.yaw):.1f}°")

    print("\n" + "-" * 70)
    total_on_field = sum(1 for cat in placements.values() for p in cat if not p.is_storage)
    total_in_storage = sum(1 for cat in placements.values() for p in cat if p.is_storage)
    print(f"场地上KFS总数: {total_on_field}")
    print(f"存放区KFS总数: {total_in_storage}")


def export_placement_json(placements: Dict[str, List[KFSPlacement]], output_path: str):
    """导出放置方案为JSON格式"""
    data = {}
    for category, items in placements.items():
        data[category] = [
            {
                "model_name": p.model_name,
                "x": p.x,
                "y": p.y,
                "z": p.z,
                "yaw": p.yaw,
                "yaw_deg": math.degrees(p.yaw),
                "block_id": p.block_id,
                "is_storage": p.is_storage
            }
            for p in items
        ]

    json_path = output_path.replace('.world', '_placement.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"Placement JSON exported: {json_path}")


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              主程序                                            ║
# ╚══════════════════════════════════════════════════════════════════════════════╝


def main():
    parser = argparse.ArgumentParser(
        description='ROBOCON 2026 World Generator (Red & Blue)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 scripts/randomWorld.py                          # 使用默认设置生成红蓝双方
  python3 scripts/randomWorld.py -s 42 -v                 # 使用种子42，显示详细信息
  python3 scripts/randomWorld.py --no-red                 # 只生成蓝方
  python3 scripts/randomWorld.py --no-blue                # 只生成红方
        """
    )

    parser.add_argument('-o', '--output', type=str, default=DEFAULT_OUTPUT_PATH,
                        help='输出world文件路径')
    parser.add_argument('-s', '--seed', type=int, default=None,
                        help='随机种子')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='显示详细放置信息')
    parser.add_argument('--json', action='store_true',
                        help='同时导出JSON格式')

    # 生成开关
    parser.add_argument('--no-red', action='store_true',
                        help='不生成红方KFS')
    parser.add_argument('--no-blue', action='store_true',
                        help='不生成蓝方KFS')

    args = parser.parse_args()

    # 处理生成开关
    global GENERATE_RED, GENERATE_BLUE
    GENERATE_RED = not args.no_red
    GENERATE_BLUE = not args.no_blue

    teams = []
    if GENERATE_RED:
        teams.append("Red")
    if GENERATE_BLUE:
        teams.append("Blue")

    print(f"Generating KFS placement ({' & '.join(teams)})...")
    placements = generate_all_kfs_placement(
        seed=args.seed
    )

    generate_world_file(placements, args.output)

    if args.json:
        export_placement_json(placements, args.output)

    if args.verbose:
        print_placement_summary(placements)

    print("\n" + "-" * 70)
    print("Configuration:")
    print(f"  Seed: {args.seed if args.seed else 'random'}")
    if GENERATE_RED:
        print(f"  Red yaw: {math.degrees(RED_TEAM_YAW):.1f}° ({RED_TEAM_YAW:.4f} rad)")
    if GENERATE_BLUE:
        print(f"  Blue yaw: {math.degrees(BLUE_TEAM_YAW):.1f}° ({BLUE_TEAM_YAW:.4f} rad)")
    print(f"  Output: {args.output}")
    print("\nDone!")


if __name__ == "__main__":
    main()
