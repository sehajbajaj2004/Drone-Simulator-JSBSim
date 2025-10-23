import jsbsim
import os

# Get JSBSim root directory
root_dir = jsbsim.get_default_root_dir()
print("\n" + "="*60)
print("JSBSim Root Directory Found:")
print("="*60)
print(root_dir)
print("="*60)

# Check if directory exists
if os.path.exists(root_dir):
    print("✓ Directory exists!")
    
    # List contents
    print("\nCurrent contents:")
    for item in os.listdir(root_dir):
        item_path = os.path.join(root_dir, item)
        if os.path.isdir(item_path):
            print(f"  📁 {item}/")
        else:
            print(f"  📄 {item}")
else:
    print("✗ Directory does not exist!")

# Show required structure
print("\n" + "="*60)
print("REQUIRED FOLDER STRUCTURE:")
print("="*60)
print(f"{root_dir}/")
print("├── aircraft/")
print("│   └── quadcopter/")
print("│       └── quadcopter.xml")
print("├── engine/")
print("│   ├── electric_motor.xml")
print("│   └── prop_8x4.xml")
print("└── systems/")
print("="*60)