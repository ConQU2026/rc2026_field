#!/usr/bin/env python3
"""
ROBOCON 2026 "武林探秘" Combined World Generator
- Generates Meilin Forest KFS (Red & Blue)
- Generates Grid KFS (Red & Blue) using a subset of Meilin KFS
- Outputs a single world file
"""
import os
import random
import argparse
import math
import json
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional, Set

# ============== Meilin Forest Configuration ==============
GENERATE_RED: bool = True
GENERATE_BLUE: bool = True

NUM_R1_KFS: int = 3
NUM_R2_KFS: int = 4
NUM_FAKE_KFS: int = 1
R1_ALLOWED_BLOCKS: List[int] = [1, 2, 3, 4, 6, 7, 9, 10, 11, 12]
FAKE_KFS_FORBIDDEN_BLOCKS: List[int] = [1, 2, 3]

RED_FOREST_BLOCKS: Dict[int, Tuple[float, float, float]] = {
    1: (2.2000, 1.8000, 5.8000), 2: (2.2000, 3.0000, 5.8000), 3: (2.2000, 4.2000, 5.8000),
    4: (1.0000, 1.8000, 5.6000), 5: (1.0000, 3.0000, 5.6000), 6: (1.0000, 4.2000, 5.6000),
    7: (-0.2000, 1.8000, 5.4000), 8: (-0.2000, 3.0000, 5.4000), 9: (-0.2000, 4.2000, 5.4000),
    10: (-1.4000, 1.8000, 5.2000), 11: (-1.4000, 3.0000, 5.2000), 12: (-1.4000, 4.2000, 5.2000),
}
BLUE_FOREST_BLOCKS: Dict[int, Tuple[float, float, float]] = {
    1: (2.2000, -4.2000, 5.8000), 2: (2.2000, -3.0000, 5.8000), 3: (2.2000, -1.8000, 5.8000),
    4: (1.0000, -4.2000, 5.6000), 5: (1.0000, -3.0000, 5.6000), 6: (1.0000, -1.8000, 5.6000),
    7: (-0.2000, -4.2000, 5.4000), 8: (-0.2000, -3.0000, 5.4000), 9: (-0.2000, -1.8000, 5.4000),
    10: (-1.4000, -4.2000, 5.2000), 11: (-1.4000, -3.0000, 5.2000), 12: (-1.4000, -1.8000, 5.2000),
}
BLOCK_OFFSETS: Dict[str, Tuple[float, float, float]] = {}
RED_TEAM_YAW: float = 0.0
BLUE_TEAM_YAW: float = math.pi

# ============== Grid Configuration ==============
BASE_X: float = -4.8
BASE_Y: float = 0.0
BASE_Z_BOTTOM: float = 1.0
PITCH_X: float = 0.54
PITCH_Z: float = 0.54
RANDOM_RANGE_X: float = 0.25
RANDOM_RANGE_Y: float = 0.13

# ============== Output Configuration ==============
DEFAULT_OUTPUT_PATH: str = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
    "worlds/robocon2026_random_all.world"
)


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              Data Structures                                   ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

@dataclass
class WorldPlacement:
    """Generic Placement Object"""
    model_name: str
    instance_name: str
    x: float
    y: float
    z: float
    yaw: float

@dataclass
class MeilinPlacement(WorldPlacement):
    block_id: Optional[int]
    is_storage: bool

@dataclass
class GridPlacement(WorldPlacement):
    pos_id: int
    team: str  # "Red" or "Blue"

# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              Meilin Logic                                      ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def get_block_position_for_team(block_id: int, team: str) -> Tuple[float, float, float]:
    if team == "red":
        base_pos = RED_FOREST_BLOCKS.get(block_id, (0, 0, 0))
    else:
        base_pos = BLUE_FOREST_BLOCKS.get(block_id, (0, 0, 0))

    offset_key = f"{team}_{block_id}"
    block_offset = BLOCK_OFFSETS.get(offset_key, (0, 0, 0))
    return (base_pos[0] + block_offset[0], base_pos[1] + block_offset[1], base_pos[2] + block_offset[2])

def generate_meilin_for_team(team: str) -> Tuple[List[MeilinPlacement], List[str]]:
    """Generates Meilin placements and returns the valid inventory (3 R1 + 4 True) for the grid."""
    if team == "red":
        yaw = RED_TEAM_YAW
        team_prefix = "Red"
    else:
        yaw = BLUE_TEAM_YAW
        team_prefix = "Blue"

    placements: List[MeilinPlacement] = []
    inventory: List[str] = [] # Stores model names (e.g., RedR1KFS, RedTrueKFS01)

    used_blocks = set()
    all_blocks = list(range(1, 13))
    random.shuffle(all_blocks)

    true_kfs_numbers = random.sample(range(1, 16), NUM_R2_KFS)
    fake_kfs_numbers = random.sample(range(1, 16), NUM_FAKE_KFS)

    # 1. R1 KFS (3)
    r1_candidates = [b for b in all_blocks if b in R1_ALLOWED_BLOCKS]
    random.shuffle(r1_candidates)
    for _ in range(NUM_R1_KFS):
        if r1_candidates:
            block_id = r1_candidates.pop()
            used_blocks.add(block_id)
            pos = get_block_position_for_team(block_id, team)
            model_name = f"{team_prefix}R1KFS"
            
            placements.append(MeilinPlacement(
                model_name=model_name,
                instance_name=f"{model_name}_meilin_{block_id}", # Init, will be refined if duplicate
                x=pos[0], y=pos[1], z=pos[2], yaw=yaw,
                block_id=block_id, is_storage=False
            ))
            inventory.append(model_name)

    # 2. Fake KFS (1)
    fake_candidates = [b for b in all_blocks if b not in FAKE_KFS_FORBIDDEN_BLOCKS and b not in used_blocks]
    random.shuffle(fake_candidates)
    if fake_candidates:
        block_id = fake_candidates.pop()
        used_blocks.add(block_id)
        pos = get_block_position_for_team(block_id, team)
        model_name = f"{team_prefix}FakeKFS{fake_kfs_numbers[0]:02d}"
        
        placements.append(MeilinPlacement(
            model_name=model_name,
            instance_name=f"{model_name}_meilin_{block_id}",
            x=pos[0], y=pos[1], z=pos[2], yaw=yaw,
            block_id=block_id, is_storage=False
        ))
        # Note: Fake KFS is NOT added to inventory for Grid

    # 3. R2 True KFS (4)
    r2_candidates = [b for b in all_blocks if b not in used_blocks]
    random.shuffle(r2_candidates)
    for i in range(NUM_R2_KFS):
        if r2_candidates:
            block_id = r2_candidates.pop()
            used_blocks.add(block_id)
            pos = get_block_position_for_team(block_id, team)
            model_name = f"{team_prefix}TrueKFS{true_kfs_numbers[i]:02d}"
            
            placements.append(MeilinPlacement(
                model_name=model_name,
                instance_name=f"{model_name}_meilin_{block_id}",
                x=pos[0], y=pos[1], z=pos[2], yaw=yaw,
                block_id=block_id, is_storage=False
            ))
            inventory.append(model_name)

    return placements, inventory

# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              Grid Logic                                        ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def get_grid_pose(index: int) -> Optional[Tuple[float, float, float]]:
    base_z_levels = {
        0: BASE_Z_BOTTOM,
        1: BASE_Z_BOTTOM + PITCH_Z,
        2: BASE_Z_BOTTOM + PITCH_Z * 2
    }
    layout = {
        1: (0, 1), 2: (0, 0), 3: (0, -1),
        4: (1, 1), 5: (1, 0), 6: (1, -1),
        7: (2, 1), 8: (2, 0), 9: (2, -1)
    }
    if index not in layout:
        return None
    row, col_offset = layout[index]
    
    center_x = BASE_X + (col_offset * PITCH_X)
    center_y = BASE_Y 
    center_z = base_z_levels[row]
    
    offset_x = random.uniform(-RANDOM_RANGE_X, RANDOM_RANGE_X)
    offset_y = random.uniform(-RANDOM_RANGE_Y, RANDOM_RANGE_Y)
    
    return (center_x + offset_x, center_y + offset_y, center_z)

def check_win(positions: Set[int]) -> bool:
    winning_lines = [
        {1, 2, 3}, {4, 5, 6}, {7, 8, 9},
        {1, 4, 7}, {2, 5, 8}, {3, 6, 9},
        {1, 5, 9}, {3, 5, 7}
    ]
    for line in winning_lines:
        if line.issubset(positions):
            return True
    return False

def generate_grid_placements(red_inventory: List[str], blue_inventory: List[str], 
                             red_count: int = 4, blue_count: int = 4) -> List[GridPlacement]:
    """Generates Grid placements using the provided inventories."""
    placements: List[GridPlacement] = []
    
    # Subset selection
    if len(red_inventory) < red_count:
        print(f"Warning: Red inventory ({len(red_inventory)}) < requested grid count ({red_count})")
        red_count = len(red_inventory)
    if len(blue_inventory) < blue_count:
        print(f"Warning: Blue inventory ({len(blue_inventory)}) < requested grid count ({blue_count})")
        blue_count = len(blue_inventory)
        
    red_models = random.sample(red_inventory, red_count)
    blue_models = random.sample(blue_inventory, blue_count)
    
    total_kfs = red_count + blue_count
    
    max_attempts = 1000
    valid_placement = False
    red_positions = set()
    blue_positions = set()
    
    for _ in range(max_attempts):
        all_indices = list(range(1, 10))
        selected_indices = random.sample(all_indices, total_kfs)
        
        current_red = set(selected_indices[:red_count])
        current_blue = set(selected_indices[red_count:])
        
        if not check_win(current_red) and not check_win(current_blue):
            red_positions = current_red
            blue_positions = current_blue
            valid_placement = True
            break
            
    if not valid_placement:
        print("Error: Could not generate valid grid placement.")
        return []
        
    assignments = []
    red_pos_list = list(red_positions)
    blue_pos_list = list(blue_positions)
    random.shuffle(red_pos_list)
    random.shuffle(blue_pos_list)
    
    for i, pos_id in enumerate(red_pos_list):
        assignments.append((pos_id, "Red", red_models[i]))
    for i, pos_id in enumerate(blue_pos_list):
        assignments.append((pos_id, "Blue", blue_models[i]))
        
    assignments.sort(key=lambda x: x[0])
    
    for pos_id, team, model_name in assignments:
        pos = get_grid_pose(pos_id)
        if pos:
            placements.append(GridPlacement(
                model_name=model_name,
                instance_name=f"{model_name}_grid_{pos_id}", # Init, refined later
                x=pos[0], y=pos[1], z=pos[2], yaw=0.0,
                pos_id=pos_id, team=team
            ))
            
    return placements

# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              World Generation                                  ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

WORLD_HEADER = """<sdf version='1.7'>
  <world name='robocon2026_combined_world'>
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


def write_world_file(placements: List[WorldPlacement], output_path: str):
    includes = []
    
    # Check for duplicate instance names just in case
    existing_names = set()

    for p in placements:
        # Use the pre-calculated instance_name if available and unique
        original_name = p.instance_name
        final_name = original_name
        
        counter = 1
        while final_name in existing_names:
            final_name = f"{original_name}_{counter}"
            counter += 1
        
        existing_names.add(final_name)
        p.instance_name = final_name
            
        includes.append(INCLUDE_TEMPLATE.format(
            model_name=p.model_name,
            instance_name=p.instance_name,
            x=p.x, y=p.y, z=p.z, yaw=p.yaw
        ))

    content = WORLD_HEADER + "\n".join(includes) + WORLD_FOOTER
    
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(content)
    print(f"World file generated: {output_path}")

# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              Main                                              ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def main():
    parser = argparse.ArgumentParser(description='ROBOCON 2026 Combined World Generator')
    parser.add_argument('-o', '--output', type=str, default=DEFAULT_OUTPUT_PATH,
                        help='Output world file path')
    parser.add_argument('-s', '--seed', type=int, default=None,
                        help='Random seed')
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)
        print(f"Using Seed: {args.seed}")

    all_placements: List[WorldPlacement] = []
    

    # 1. Generate Meilin
    print("Generating Meilin Forest Placements...")
    red_meilin_placements, red_inventory = generate_meilin_for_team("red")
    blue_meilin_placements, blue_inventory = generate_meilin_for_team("blue")
    
    all_placements.extend(red_meilin_placements)
    all_placements.extend(blue_meilin_placements)
    
    # Print detailed counts
    def print_team_summary(team_name, placements, inventory):
        r1_count = sum(1 for p in placements if "R1KFS" in p.model_name)
        fake_count = sum(1 for p in placements if "FakeKFS" in p.model_name)
        true_count = sum(1 for p in placements if "TrueKFS" in p.model_name)
        print(f"  {team_name} Side: {len(placements)} Total ({r1_count} R1, {true_count} True, {fake_count} Fake)")
        print(f"  {team_name} Grid Inventory (No Fake): {len(inventory)} items {inventory}")

    print_team_summary("Red", red_meilin_placements, red_inventory)
    print_team_summary("Blue", blue_meilin_placements, blue_inventory)

    # 2. Generate Grid
    print("Generating Grid Placements (Subset of Meilin)...")
    grid_placements = generate_grid_placements(red_inventory, blue_inventory)
    all_placements.extend(grid_placements)
    
    # 3. Write World
    write_world_file(all_placements, args.output)
    
    print("Done!")

if __name__ == "__main__":
    main()
