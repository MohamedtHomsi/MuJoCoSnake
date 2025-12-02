"""
Simple viewer for the new CAD screw models
Displays NEW_TESTER_REV_A .stl and NEW_TESTER_REV_B.stl as static models
"""

import os
os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco
import mujoco.viewer as viewer

def main():
    # Build a simple static model to view the new screws
    xml_model = """
<mujoco model="new_cad_viewer">
    <compiler angle="degree"/>
    <option gravity="0 0 0"/>
    
    <asset>
        <mesh name="screw_left" file="NEW_SIMPLIFY_TESTER_REV_A.stl" scale="0.01 0.01 0.01"/>
        <mesh name="screw_right" file="NEW_TESTER_REV_B.stl" scale="0.01 0.01 0.01"/>
        
        <!-- Checkerboard texture for ground -->
        <texture name="checker" type="2d" builtin="checker" width="512" height="512" 
                 rgb1="0.95 0.95 0.95" rgb2="0.75 0.75 0.75"/>
        <material name="checkermat" texture="checker" texrepeat="20 20" reflectance="0.1"/>
    </asset>
    
    <worldbody>
        <light diffuse="1 1 1" pos="0 0 3" dir="0 0 -1"/>
        <geom type="plane" size="10 10 0.1" material="checkermat"/>
        
        <!-- Base body with free joint to allow positioning -->
        <body name="assembly_base" pos="0 0 0.4">
            <joint name="base_free" type="free"/>
            
            <!-- Hinge pin connecting the two screws -->
            <geom name="hinge_pin" type="capsule" fromto="-0.1 0 0 0.1 0 0" size="0.02" rgba="0.5 0.5 0.5 1"/>
            
            <!-- Left screw (blue) - connected via hinge joint -->
            <body name="left_screw" pos="-0.55 0 0">
                <joint name="left_hinge" type="hinge" axis="1 0 0" pos="0 0 0" limited="false" damping="0.01"/>
                <geom type="mesh" mesh="screw_left" 
                      euler="0 -90 0" 
                      rgba="0.2 0.5 1.0 1"
                      contype="0" conaffinity="0" mass="0.1"/>
            </body>
            
            <!-- Right screw (red) - connected via hinge joint -->
            <body name="right_screw" pos="0 0 0">
                <joint name="right_hinge" type="hinge" axis="1 0 0" pos="0 0 0" limited="false" damping="0.01"/>
                <geom type="mesh" mesh="screw_right" 
                      euler="0 90 0" 
                      rgba="1.0 0.2 0.2 1"
                      contype="0" conaffinity="0" mass="0.1"/>
            </body>
        </body>
    </worldbody>
</mujoco>
    """
    
    print("Loading new CAD models...")
    print("  Left (blue): NEW_SIMPLIFY_TESTER_REV_A.stl (189k triangles)")
    print("  Right (red): NEW_TESTER_REV_B.stl (128k triangles)")
    m = mujoco.MjModel.from_xml_string(xml_model)
    d = mujoco.MjData(m)
    
    print(f"✓ Model loaded successfully!")
    print(f"  Bodies: {m.nbody}")
    print(f"  Geoms: {m.ngeom}")
    print(f"  Meshes: {m.nmesh}")
    print(f"  Vertices: {m.mesh_vert.shape[0] if m.nmesh > 0 else 0}")
    
    # Forward kinematics to update visualization
    mujoco.mj_forward(m, d)
    
    print("\nLaunching viewer...")
    print("Controls:")
    print("  - Click and drag to rotate view")
    print("  - Right-click and drag to pan")
    print("  - Scroll to zoom")
    print("  - Press Ctrl+R to reset view")
    print("\nNote: Models are static (no physics/movement)")
    
    # Launch passive viewer - models won't move since there's no gravity and no joints
    with viewer.launch_passive(m, d) as v:
        # Set camera to see big models
        v.cam.distance = 15
        v.cam.elevation = -20
        v.cam.azimuth = 135
        v.cam.lookat[:] = [0, 0, 0]
        
        while v.is_running():
            # No stepping needed for static display, just sync the viewer
            v.sync()

if __name__ == "__main__":
    main()

