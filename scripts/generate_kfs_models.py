import os
import shutil

# Base paths
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
MODELS_DIR = os.path.join(ROOT_DIR, 'models')
IMG_DIR = os.path.join(ROOT_DIR, 'KFS_imgs')

# Templates
MODEL_CONFIG_TEMPLATE = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Generated</name>
    <email>generated@example.com</email>
  </author>
  <description>
    KFS Model generated from {image_file}
  </description>
</model>
"""

# START: Use mesh for visual to control UV mapping
MODEL_SDF_TEMPLATE = """<?xml version='1.0' encoding='utf-8'?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.083</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>
          <iyy>0.083</iyy> <iyz>0.0</iyz>
          <izz>0.083</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.35 0.35 0.35</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{name}/meshes/model.obj</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://{name}/materials/scripts</uri>
            <uri>model://{name}/materials/textures</uri>
            <name>RoboCon/{name}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MATERIAL_SCRIPT_TEMPLATE = """material RoboCon/{name}
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture {texture_filename}
        tex_address_mode clamp
        scale 1.0 1.0
      }}
    }}
  }}
}}
"""

def get_cube_obj_content(size=0.35):
    """
    Generate OBJ content for a cube with specific UV mapping.
    Sides: Standard orientation.
    Top/Bottom: Rotated 90 degrees clockwise relative to standard.
    """
    h = size / 2.0
    
    # Vertices
    # V0: Front-Bottom-Left
    # ...
    # Let's define manual vertices to ensure faces are correct
    # Front Face (+X in Gazebo? No, usually OBJ is Y-up or Z-up depending on importer.)
    # Gazebo uses Z-Up, Y-Left, X-Forward logic effectively.
    # But OBJ is just geometry.
    # Let's define the cube centered at 0.
    
    # Vertices
    vertices = [
        # Bottom (-Z)
        (-h, -h, -h), # 1
        ( h, -h, -h), # 2
        ( h,  h, -h), # 3
        (-h,  h, -h), # 4
             
        # Top (+Z)
        (-h, -h,  h), # 5
        ( h, -h,  h), # 6
        ( h,  h,  h), # 7
        (-h,  h,  h), # 8
    ]

    # UVs
    # 1: (0,0) BL
    # 2: (1,0) BR
    # 3: (1,1) TR
    # 4: (0,1) TL
    uvs = [
        (0.0, 0.0),
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 1.0)
    ]
    
    # Faces list of (vertex_idx, uv_idx, normal_idx)
    # Normals: 1:+Z, 2:-Z, 3:+Y, 4:-Y, 5:+X, 6:-X
    # For box:
    # Top (+Z), Bottom (-Z), Front (-Y?), Back (+Y?), Left (+X?), Right (-X?)
    # Let's assume standard Gazebo alignment where you look at Front.
    
    lines = []
    lines.append("# KFS Cube Custom UV")
    lines.append(f"o KFS_Cube")
    
    for v in vertices:
        lines.append(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}")
        
    for uv in uvs:
        lines.append(f"vt {uv[0]:.6f} {uv[1]:.6f}")

    # Normals
    normals = [
        ( 0,  0,  1), # 1 Top
        ( 0,  0, -1), # 2 Bottom
        ( 0, -1,  0), # 3 Front
        ( 0,  1,  0), # 4 Back
        ( 1,  0,  0), # 5 Right
        (-1,  0,  0)  # 6 Left
    ]
    for n in normals:
        lines.append(f"vn {n[0]:.1f} {n[1]:.1f} {n[2]:.1f}")

    # Face Construction
    # f v/vt/vn
    # Indices are 1-based
    
    # Standard Mapping: 
    # BL(1)/uv1, BR(2)/uv2, TR(3)/uv3, TL(4)/uv4
    
    # Rotated 90 CW Mapping:
    # Top-Left vertex gets Bottom-Left UV (conceptually rotating texture right)
    # Current: TL(uv4) -> TR(uv3) -> BR(uv2) -> BL(uv1)
    # Rotated 90 CW: 
    #   Vertex that WAS TL now gets BL UV (0,0) ? No.
    #   Image rotates CW. Top of Image (1,1..0,1) moves to Right of Face.
    #   So Face Right edge (TR, BR) should map to Image Top (TL, TR).
    #   TR vertex maps to Image TL (0,1) [uv4]
    #   BR vertex maps to Image TR (1,1) [uv3]
    #   BL vertex maps to Image BR (1,0) [uv2]
    #   TL vertex maps to Image BL (0,0) [uv1]
    
    # Faces:
    
    # 3. Front Face (-Y) 
    # Vertices: 1, 2, 6, 5 (Bottom-Left, Bottom-Right, Top-Right, Top-Left)
    # Standard orientation
    lines.append("f 1/1/3 2/2/3 6/3/3 5/4/3")

    # 4. Back Face (+Y)
    # Vertices: 2, 3, 7, 6 ? No.
    # Back is 2, 1? No.
    # Let's trace CCW from outside.
    # 2(BR), 3(TR of back?), 7(TL of back?), 6(BL of back?)
    # Vertices: 3, 4, 8, 7
    lines.append("f 3/1/4 4/2/4 8/3/4 7/4/4")
    
    # 5. Right Face (+X)
    # Vertices: 2, 3, 7, 6
    lines.append("f 2/1/5 3/2/5 7/3/5 6/4/5")
    
    # 6. Left Face (-X)
    # Vertices: 4, 1, 5, 8
    lines.append("f 4/1/6 1/2/6 5/3/6 8/4/6")
    
    # --- Special Faces (Rotated) ---
    
    # 1. Top Face (+Z)
    # Vertices: 5(FL), 6(FR), 7(BR), 8(BL)
    # Standard: 5/1 6/2 7/3 8/4
    # Rotated 90 CW:
    # 5(FL=TL relative), 6(FR=TR), 7(BR), 8(BL)?
    # Let's map carefully.
    # 5 [Front-Left] -> Needs UV1 (0,0) for standard?
    # If we want 90 CW rotation:
    # 5 uses UV4 (0,1)?
    # 6 uses UV1 (0,0)?
    # 7 uses UV2 (1,0)?
    # 8 uses UV3 (1,1)?
    # Let's check: 
    #   5->0,1 (TL of image)
    #   6->0,0 (BL of image)
    #   Wait, 5 is FL. 6 is FR.
    #   Vector 5->6 (Left to Right) maps to Image Top to Bottom?
    #   0,1 -> 0,0. Left of face = Top of Image? No. 
    #   Let's just shift UVs by -1 (or +3).
    #   Standard: v1/vt1 v2/vt2 v3/vt3 v4/vt4
    #   Shifted:  v1/vt2 v2/vt3 v3/vt4 v4/vt1 (90 deg CCW?)
    #   Shifted:  v1/vt4 v2/vt1 v3/vt2 v4/vt3 (90 deg CW?)
    #   Let's try Shifted +1 (vt2, vt3, vt4, vt1)
    #   If v1 maps to vt2(1,0 BR), v2 maps to vt3(1,1 TR).
    #   Bottom of face maps to Right of image.
    #   So Image is rotated 90 CCW.
    #   We want CW.
    #   So v1 maps to vt4(0,1 TL), v2 maps to vt1(0,0 BL).
    #   Bottom of face maps to Left of image.
    #   Image is rotated 90 CW.
    #   So we use mapping: v1/4 v2/1 v3/2 v4/3
    lines.append("f 5/4/1 6/1/1 7/2/1 8/3/1")
    
    # 2. Bottom Face (-Z)
    # Vertices: 4, 3, 2, 1
    # Rotated 90 CW
    lines.append("f 4/4/2 3/1/2 2/2/2 1/3/2")
    
    return "\n".join(lines)


def generate_models():
    if not os.path.exists(IMG_DIR):
        print(f"Error: Image directory not found at {IMG_DIR}")
        return

    files = [f for f in os.listdir(IMG_DIR) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
    print(f"Found {len(files)} image files.")
    
    # Sort files to ensure deterministic order (optional but good practice)
    files.sort()

    for filename in files:
        # Model name will be the filename without extension (e.g., 'BlueFakeKFS01')
        model_name = os.path.splitext(filename)[0]
        
        # Paths for the new model
        model_path = os.path.join(MODELS_DIR, model_name)
        materials_scripts_path = os.path.join(model_path, 'materials', 'scripts')
        materials_textures_path = os.path.join(model_path, 'materials', 'textures')
        meshes_path = os.path.join(model_path, 'meshes')
        
        # Create directories
        os.makedirs(materials_scripts_path, exist_ok=True)
        os.makedirs(materials_textures_path, exist_ok=True)
        os.makedirs(meshes_path, exist_ok=True)
        
        # 1. model.config
        with open(os.path.join(model_path, 'model.config'), 'w') as f:
            f.write(MODEL_CONFIG_TEMPLATE.format(name=model_name, image_file=filename))
            
        # 2. model.sdf (Using Mesh)
        with open(os.path.join(model_path, 'model.sdf'), 'w') as f:
            f.write(MODEL_SDF_TEMPLATE.format(name=model_name))
            
        # 3. material script
        material_filename = f"{model_name}.material"
        with open(os.path.join(materials_scripts_path, material_filename), 'w') as f:
            f.write(MATERIAL_SCRIPT_TEMPLATE.format(name=model_name, texture_filename=filename))
            
        # 4. Copy texture
        src_img = os.path.join(IMG_DIR, filename)
        dst_img = os.path.join(materials_textures_path, filename)
        shutil.copy2(src_img, dst_img)
        
        # 5. Generate OBJ file
        obj_content = get_cube_obj_content()
        with open(os.path.join(meshes_path, 'model.obj'), 'w') as f:
            f.write(obj_content)
        
        print(f"Generated model: {model_name}")

if __name__ == "__main__":
    generate_models()
