import os
import shutil

# Base paths
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(BASE_DIR, 'models')
IMG_DIR = os.path.join(MODELS_DIR, 'img')

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
          <box><size>0.35 0.35 0.35</size></box>
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
        tex_address_mode border
        tex_border_colour 0.208 0.227 0.553 1.0
        scale 1.0 1.0
      }}
    }}
  }}
}}
"""

def generate_models():
    if not os.path.exists(IMG_DIR):
        print(f"Error: Image directory not found at {IMG_DIR}")
        return

    files = [f for f in os.listdir(IMG_DIR) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
    print(f"Found {len(files)} image files.")

    for filename in files:
        # Model name will be the filename without extension (e.g., 'BlueFakeKFS01')
        model_name = os.path.splitext(filename)[0]
        
        # Paths for the new model
        model_path = os.path.join(MODELS_DIR, model_name)
        materials_scripts_path = os.path.join(model_path, 'materials', 'scripts')
        materials_textures_path = os.path.join(model_path, 'materials', 'textures')
        
        # Create directories
        os.makedirs(materials_scripts_path, exist_ok=True)
        os.makedirs(materials_textures_path, exist_ok=True)
        
        # 1. model.config
        with open(os.path.join(model_path, 'model.config'), 'w') as f:
            f.write(MODEL_CONFIG_TEMPLATE.format(name=model_name, image_file=filename))
            
        # 2. model.sdf
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
        
        print(f"Generated model: {model_name}")

if __name__ == "__main__":
    generate_models()
