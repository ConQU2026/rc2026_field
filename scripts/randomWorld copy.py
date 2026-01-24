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

# ============== 生成开关 ==============
GENERATE_RED: bool = True       # 是否生成红方KFS
GENERATE_BLUE: bool = True      # 是否生成蓝方KFS

# ============== 红方全局偏移配置 ==============
RED_GLOBAL_OFFSET_X: float = -2.4    # 红方X轴全局偏移 (米)
RED_GLOBAL_OFFSET_Y: float = 3.0     # 红方Y轴全局偏移 (米)
RED_GLOBAL_OFFSET_Z: float = 5.0     # 红方Z轴全局偏移 (米)

# ============== 蓝方全局偏移配置 ==============
# 蓝方梅花林在红方对面，需要单独配置
BLUE_GLOBAL_OFFSET_X: float = -2.4    # 蓝方X轴全局偏移 (米)
BLUE_GLOBAL_OFFSET_Y: float = -3.0    # 蓝方Y轴全局偏移 (米)
BLUE_GLOBAL_OFFSET_Z: float = 5.0    # 蓝方Z轴全局偏移 (米)

# ============== 红方旋转配置 ==============
# 以场地中心(0,0)为旋转中心，绕Z轴旋转
# yaw角度，单位：弧度。正值为逆时针旋转
# 常用值: 0=不旋转, math.pi/2=90度, math.pi=180度, -math.pi/2=-90度
RED_GLOBAL_YAW: float = 0.0         # 红方全局yaw旋转角度 (弧度)

# ============== 蓝方旋转配置 ==============
BLUE_GLOBAL_YAW: float = math.pi    # 蓝方全局yaw旋转角度 (弧度)，默认旋转180度

# 兼容旧变量名
GLOBAL_OFFSET_X: float = RED_GLOBAL_OFFSET_X
GLOBAL_OFFSET_Y: float = RED_GLOBAL_OFFSET_Y
GLOBAL_OFFSET_Z: float = RED_GLOBAL_OFFSET_Z
GLOBAL_YAW: float = RED_GLOBAL_YAW

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
FAKE_KFS_FORBIDDEN_BLOCKS: List[int] = [10, 11, 12]

# ============== 树林方块配置 ==============
# 方块排列（4行 x 3列 = 12个方块）：
#
#     红方梅林区 (俯视图)          方块编号
#     ==================          ========
#     第四行(入口): □ □ □          10, 11, 12  (假KFS禁止)
#     第三行:       □ □ □          7,  8,  9
#     第二行:       □ □ □          4,  5,  6
#     第一行:       □ □ □          1,  2,  3
#
# 方块间距配置（可调整）
BLOCK_SPACING_X: float = 1.2    # X方向（前后）间距
BLOCK_SPACING_Y: float = 1.2    # Y方向（左右）间距

# 红方梅林区起始X坐标（入口位置）
RED_START_X: float = 1.0

# 各行高度配置（Z坐标）
ROW_HEIGHTS: List[float] = [0.20, 0.40, 0.60, 0.80]  # 第1-4行高度

# 红方树林方块 (X > 0)
# 4行 x 3列
RED_FOREST_BLOCKS: Dict[int, Tuple[float, float, float]] = {
    # 第一行 (3个)
    1:  (RED_START_X + 0*BLOCK_SPACING_X, -1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[0]),
    2:  (RED_START_X + 0*BLOCK_SPACING_X,  0.0*BLOCK_SPACING_Y, ROW_HEIGHTS[0]),
    3:  (RED_START_X + 0*BLOCK_SPACING_X,  1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[0]),
    # 第二行 (3个)
    4:  (RED_START_X + 1*BLOCK_SPACING_X, -1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[1]),
    5:  (RED_START_X + 1*BLOCK_SPACING_X,  0.0*BLOCK_SPACING_Y, ROW_HEIGHTS[1]),
    6:  (RED_START_X + 1*BLOCK_SPACING_X,  1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[1]),
    # 第三行 (3个)
    7:  (RED_START_X + 2*BLOCK_SPACING_X, -1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[2]),
    8:  (RED_START_X + 2*BLOCK_SPACING_X,  0.0*BLOCK_SPACING_Y, ROW_HEIGHTS[2]),
    9:  (RED_START_X + 2*BLOCK_SPACING_X,  1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[2]),
    # 第四行 (入口，3个) - 假KFS禁止放置
    10: (RED_START_X + 3*BLOCK_SPACING_X, -1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[3]),
    11: (RED_START_X + 3*BLOCK_SPACING_X,  0.0*BLOCK_SPACING_Y, ROW_HEIGHTS[3]),
    12: (RED_START_X + 3*BLOCK_SPACING_X,  1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[3]),
}

# ============== 蓝方树林方块配置 ==============
# 蓝方梅林区起始X坐标（入口位置，与红方对称）
BLUE_START_X: float = -1.0

# 蓝方树林方块 (X < 0)，与红方镜像对称
# 4行 x 3列
BLUE_FOREST_BLOCKS: Dict[int, Tuple[float, float, float]] = {
    # 第一行 (3个)
    1:  (BLUE_START_X - 0*BLOCK_SPACING_X,  1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[0]),
    2:  (BLUE_START_X - 0*BLOCK_SPACING_X,  0.0*BLOCK_SPACING_Y, ROW_HEIGHTS[0]),
    3:  (BLUE_START_X - 0*BLOCK_SPACING_X, -1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[0]),
    # 第二行 (3个)
    4:  (BLUE_START_X - 1*BLOCK_SPACING_X,  1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[1]),
    5:  (BLUE_START_X - 1*BLOCK_SPACING_X,  0.0*BLOCK_SPACING_Y, ROW_HEIGHTS[1]),
    6:  (BLUE_START_X - 1*BLOCK_SPACING_X, -1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[1]),
    # 第三行 (3个)
    7:  (BLUE_START_X - 2*BLOCK_SPACING_X,  1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[2]),
    8:  (BLUE_START_X - 2*BLOCK_SPACING_X,  0.0*BLOCK_SPACING_Y, ROW_HEIGHTS[2]),
    9:  (BLUE_START_X - 2*BLOCK_SPACING_X, -1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[2]),
    # 第四行 (入口，3个) - 假KFS禁止放置
    10: (BLUE_START_X - 3*BLOCK_SPACING_X,  1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[3]),
    11: (BLUE_START_X - 3*BLOCK_SPACING_X,  0.0*BLOCK_SPACING_Y, ROW_HEIGHTS[3]),
    12: (BLUE_START_X - 3*BLOCK_SPACING_X, -1.0*BLOCK_SPACING_Y, ROW_HEIGHTS[3]),
}

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
DEFAULT_OUTPUT_PATH: str = "/home/inkc/inkc/sws/src/sim/rc2026_field/worlds/robocon2026_random.world"


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
# ╚══════════════════════════════════════════════════════════════════════════════╝

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


def get_block_position(
    block_id: int,
    offset_x: float = GLOBAL_OFFSET_X,
    offset_y: float = GLOBAL_OFFSET_Y,
    offset_z: float = GLOBAL_OFFSET_Z,
    yaw: float = GLOBAL_YAW
) -> Tuple[float, float, float]:
    """获取方块的最终位置（应用偏移和旋转后）"""
    base_pos = RED_FOREST_BLOCKS.get(block_id, (0, 0, 0))

    # 获取方块特定偏移
    offset_key = f"red_{block_id}"
    block_offset = BLOCK_OFFSETS.get(offset_key, (0, 0, 0))

    # 计算基础位置（未旋转）
    base_x = base_pos[0] + block_offset[0]
    base_y = base_pos[1] + block_offset[1]
    base_z = base_pos[2] + offset_z + block_offset[2]

    # 应用旋转（绕原点）
    rotated_x, rotated_y = rotate_point(base_x, base_y, yaw)

    # 应用全局偏移
    final_x = rotated_x + offset_x
    final_y = rotated_y + offset_y

    return (final_x, final_y, base_z)


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
    offset_x: float,
    offset_y: float,
    offset_z: float,
    yaw: float
) -> Tuple[float, float, float]:
    """获取指定队伍方块的最终位置（应用偏移和旋转后）"""
    if team == "red":
        base_pos = RED_FOREST_BLOCKS.get(block_id, (0, 0, 0))
    else:
        base_pos = BLUE_FOREST_BLOCKS.get(block_id, (0, 0, 0))

    # 获取方块特定偏移
    offset_key = f"{team}_{block_id}"
    block_offset = BLOCK_OFFSETS.get(offset_key, (0, 0, 0))

    # 计算基础位置（未旋转）
    base_x = base_pos[0] + block_offset[0]
    base_y = base_pos[1] + block_offset[1]
    base_z = base_pos[2] + offset_z + block_offset[2]

    # 应用旋转（绕原点）
    rotated_x, rotated_y = rotate_point(base_x, base_y, yaw)

    # 应用全局偏移
    final_x = rotated_x + offset_x
    final_y = rotated_y + offset_y

    return (final_x, final_y, base_z)


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              KFS放置生成器                                     ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def generate_team_kfs_placement(
    team: str,
    seed: Optional[int] = None,
    offset_x: float = None,
    offset_y: float = None,
    offset_z: float = None,
    yaw: float = None
) -> Dict[str, List[KFSPlacement]]:
    """
    生成单个队伍的KFS随机放置方案

    规则：
    - 总共8个KFS放置在12个方块中
    - R1 KFS (3个): 只能放在3x4边缘方块，使用 RedR1KFS/BlueR1KFS 模型
    - R2 KFS (4个): 可以放在任意方块，从 TrueKFS01-15 随机选4个
    - 假KFS (1个): 禁止放在入口方块10,11,12，从 FakeKFS01-15 随机选1个
    """
    # 使用队伍对应的默认值
    if team == "red":
        offset_x = offset_x if offset_x is not None else RED_GLOBAL_OFFSET_X
        offset_y = offset_y if offset_y is not None else RED_GLOBAL_OFFSET_Y
        offset_z = offset_z if offset_z is not None else RED_GLOBAL_OFFSET_Z
        yaw = yaw if yaw is not None else RED_GLOBAL_YAW
        team_prefix = "Red"
    else:
        offset_x = offset_x if offset_x is not None else BLUE_GLOBAL_OFFSET_X
        offset_y = offset_y if offset_y is not None else BLUE_GLOBAL_OFFSET_Y
        offset_z = offset_z if offset_z is not None else BLUE_GLOBAL_OFFSET_Z
        yaw = yaw if yaw is not None else BLUE_GLOBAL_YAW
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
            pos = get_block_position_for_team(block_id, team, offset_x, offset_y, offset_z, yaw)

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
    fake_candidates = [b for b in all_blocks if b not in FAKE_KFS_FORBIDDEN_BLOCKS and b not in used_blocks]
    random.shuffle(fake_candidates)

    for i in range(NUM_FAKE_KFS):
        if fake_candidates:
            block_id = fake_candidates.pop()
            used_blocks.add(block_id)
            pos = get_block_position_for_team(block_id, team, offset_x, offset_y, offset_z, yaw)

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
            pos = get_block_position_for_team(block_id, team, offset_x, offset_y, offset_z, yaw)

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


def generate_kfs_placement(
    seed: Optional[int] = None,
    offset_x: float = GLOBAL_OFFSET_X,
    offset_y: float = GLOBAL_OFFSET_Y,
    offset_z: float = GLOBAL_OFFSET_Z,
    yaw: float = GLOBAL_YAW
) -> Dict[str, List[KFSPlacement]]:
    """生成所有KFS的随机放置方案（兼容旧接口，只生成红方）"""
    if seed is not None:
        random.seed(seed)

    return generate_team_kfs_placement("red", None, offset_x, offset_y, offset_z, yaw)


def generate_all_kfs_placement(
    seed: Optional[int] = None,
    red_offset_x: float = None,
    red_offset_y: float = None,
    red_offset_z: float = None,
    red_yaw: float = None,
    blue_offset_x: float = None,
    blue_offset_y: float = None,
    blue_offset_z: float = None,
    blue_yaw: float = None,
) -> Dict[str, List[KFSPlacement]]:
    """生成红蓝双方KFS的随机放置方案"""
    if seed is not None:
        random.seed(seed)

    placements: Dict[str, List[KFSPlacement]] = {}

    # 生成红方
    if GENERATE_RED:
        red_placements = generate_team_kfs_placement(
            "red", None, red_offset_x, red_offset_y, red_offset_z, red_yaw
        )
        placements.update(red_placements)

    # 生成蓝方
    if GENERATE_BLUE:
        blue_placements = generate_team_kfs_placement(
            "blue", None, blue_offset_x, blue_offset_y, blue_offset_z, blue_yaw
        )
        placements.update(blue_placements)

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
  python3 tmp.py                          # 使用默认设置生成红蓝双方
  python3 tmp.py -s 42 -v                 # 使用种子42，显示详细信息
  python3 tmp.py --red-yaw 90             # 红方旋转90度
  python3 tmp.py --blue-yaw 180           # 蓝方旋转180度
  python3 tmp.py --no-red                 # 只生成蓝方
  python3 tmp.py --no-blue                # 只生成红方
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

    # 红方偏移参数
    parser.add_argument('--red-x', type=float, default=RED_GLOBAL_OFFSET_X,
                        help=f'红方X轴偏移 (默认: {RED_GLOBAL_OFFSET_X})')
    parser.add_argument('--red-y', type=float, default=RED_GLOBAL_OFFSET_Y,
                        help=f'红方Y轴偏移 (默认: {RED_GLOBAL_OFFSET_Y})')
    parser.add_argument('--red-z', type=float, default=RED_GLOBAL_OFFSET_Z,
                        help=f'红方Z轴偏移 (默认: {RED_GLOBAL_OFFSET_Z})')
    parser.add_argument('--red-yaw', type=float, default=math.degrees(RED_GLOBAL_YAW),
                        help=f'红方yaw角度（度） (默认: {math.degrees(RED_GLOBAL_YAW)})')

    # 蓝方偏移参数
    parser.add_argument('--blue-x', type=float, default=BLUE_GLOBAL_OFFSET_X,
                        help=f'蓝方X轴偏移 (默认: {BLUE_GLOBAL_OFFSET_X})')
    parser.add_argument('--blue-y', type=float, default=BLUE_GLOBAL_OFFSET_Y,
                        help=f'蓝方Y轴偏移 (默认: {BLUE_GLOBAL_OFFSET_Y})')
    parser.add_argument('--blue-z', type=float, default=BLUE_GLOBAL_OFFSET_Z,
                        help=f'蓝方Z轴偏移 (默认: {BLUE_GLOBAL_OFFSET_Z})')
    parser.add_argument('--blue-yaw', type=float, default=math.degrees(BLUE_GLOBAL_YAW),
                        help=f'蓝方yaw角度（度） (默认: {math.degrees(BLUE_GLOBAL_YAW)})')

    # 兼容旧参数
    parser.add_argument('--global-x', type=float, default=None,
                        help='(兼容旧版) 全局X轴偏移，同时应用于红蓝双方')
    parser.add_argument('--global-y', type=float, default=None,
                        help='(兼容旧版) 全局Y轴偏移，同时应用于红蓝双方')
    parser.add_argument('--global-z', type=float, default=None,
                        help='(兼容旧版) 全局Z轴偏移，同时应用于红蓝双方')
    parser.add_argument('--yaw', type=float, default=None,
                        help='(兼容旧版) 全局yaw角度，同时应用于红蓝双方')

    args = parser.parse_args()

    # 处理生成开关
    global GENERATE_RED, GENERATE_BLUE
    GENERATE_RED = not args.no_red
    GENERATE_BLUE = not args.no_blue

    # 处理兼容旧参数
    red_x = args.global_x if args.global_x is not None else args.red_x
    red_y = args.global_y if args.global_y is not None else args.red_y
    red_z = args.global_z if args.global_z is not None else args.red_z
    red_yaw = math.radians(args.yaw) if args.yaw is not None else math.radians(args.red_yaw)

    blue_x = args.global_x if args.global_x is not None else args.blue_x
    blue_y = args.global_y if args.global_y is not None else args.blue_y
    blue_z = args.global_z if args.global_z is not None else args.blue_z
    blue_yaw = math.radians(args.yaw) if args.yaw is not None else math.radians(args.blue_yaw)

    teams = []
    if GENERATE_RED:
        teams.append("Red")
    if GENERATE_BLUE:
        teams.append("Blue")

    print(f"Generating KFS placement ({' & '.join(teams)})...")
    placements = generate_all_kfs_placement(
        seed=args.seed,
        red_offset_x=red_x,
        red_offset_y=red_y,
        red_offset_z=red_z,
        red_yaw=red_yaw,
        blue_offset_x=blue_x,
        blue_offset_y=blue_y,
        blue_offset_z=blue_z,
        blue_yaw=blue_yaw,
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
        print(f"  Red offset: ({red_x}, {red_y}, {red_z})")
        print(f"  Red yaw: {math.degrees(red_yaw):.1f}° ({red_yaw:.4f} rad)")
    if GENERATE_BLUE:
        print(f"  Blue offset: ({blue_x}, {blue_y}, {blue_z})")
        print(f"  Blue yaw: {math.degrees(blue_yaw):.1f}° ({blue_yaw:.4f} rad)")
    print(f"  Output: {args.output}")
    print("\nDone!")


if __name__ == "__main__":
    main()
