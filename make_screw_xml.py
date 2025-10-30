import math
import os
from pathlib import Path

# ====== Tunables ======
L = 0.40  # Length of each screw along X-axis
R = 0.050  # Core radius
# Increase rib radius for larger contact area (better grip)
rib_r = 0.028
# Steepen the helix (smaller pitch) to increase axial component of contact force
pitch = 0.07
# Increase number of parallel ribs for continuous contact
rib_count = 4
# Increase segments for smoother contact path
segments = 80
center_gap = 0.01  # Gap between screws (end-to-end configuration)
# Terrain grooves (to resist Y slip, allow X travel)
groove_pitch = 0.06   # spacing between groove centers along Y
ridge_width  = 0.010  # ridge thickness along Y
ridge_height = 0.006  # ridge height (meters)
num_ridges   = 7      # total ridges on each side (placed symmetrically)
core_mass = 0.40
base_clear = 0.0005  # Minimal clearance so ribs touch mat
# Friction: ground (backup), mat (compliant surface ribs bite into)
ground_mu = "2.5 0.01 0.002"
mat_mu = "6.0 0.01 0.002"  # Higher friction to promote bite and reduce slip
# ======================

z_base = R + rib_r + base_clear

def helix_capsules(x0, x1, handedness=+1, phase=0.0):
    """Generate helix along X-axis (screws point in X direction, end-to-end)"""
    dx = (x1 - x0) / segments
    for i in range(segments):
        xa = x0 + dx * i
        xb = x0 + dx * (i + 1)
        aa = handedness * (2*math.pi * (xa / pitch)) + phase
        ab = handedness * (2*math.pi * (xb / pitch)) + phase
        ya, za = R*math.cos(aa), R*math.sin(aa)
        yb, zb = R*math.cos(ab), R*math.sin(ab)
        yield xa, ya, za, xb, yb, zb

def screw_xml(name, joint, color, handedness, xoffset, fin_color):
    """Generate a screw oriented along X-axis (end-to-end configuration)"""
    # Core is visual only - NO CONTACT so only ribs engage ground
    core_color = ' '.join([str(float(c)*0.7) for c in color.split()[:3]] + ['1'])
    geoms = [f'''\
      <geom name="core_{name}" type="cylinder" fromto="-{L} 0 0  {L} 0 0"
            size="{R*0.85}" rgba="{core_color}" mass="{core_mass}"
            contype="0" conaffinity="0"/>''']  # No collision on core!
    
    # Helical ribs - ONLY these make contact
    for k in range(rib_count):
        phase = 2*math.pi * k / rib_count
        for xa, ya, za, xb, yb, zb in helix_capsules(-L, L, handedness, phase):
            geoms.append(
                f'''      <geom type="capsule" fromto="{xa} {ya} {za}  {xb} {yb} {zb}"
                           size="{rib_r}" rgba="{color}"
                           friction="3.5 0.01 0.002" mass="0.003"/>'''
            )
    # End caps (visual only - no collision)
    geoms.append(f'''      <geom type="sphere" pos="-{L} 0 0" size="{R*0.8}" rgba="{color}"
                           mass="0.01" contype="0" conaffinity="0"/>''')
    geoms.append(f'''      <geom type="sphere" pos="{L} 0 0" size="{R*0.8}" rgba="{color}"
                           mass="0.01" contype="0" conaffinity="0"/>''')

    return f'''
    <body name="{name}" pos="{xoffset} 0 0">
      <!-- Unlimited hinge about +X (screws rotate about X-axis → thrust in Y) -->
      <joint name="{joint}" type="hinge" axis="1 0 0" pos="0 0 0"
             limited="false" damping="0.001" frictionloss="0.0"/>
{chr(10).join(geoms)}
    </body>'''

enable_ridges = os.getenv("ENABLE_RIDGES", "0").strip() not in {"0", "false", "False", "off", ""}

xml = f'''<mujoco model="screw_snake">
  <compiler angle="degree" coordinate="local" autolimits="true" inertiafromgeom="true"/>
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast" iterations="100" solver="Newton"/>

  <default>
    <geom rgba="0.8 0.8 0.8 1" condim="6" friction="6.0 0.01 0.002"
          solref="0.003 1" solimp="0.96 0.96 0.01"/>
    <joint armature="0.001" damping="0.05"/>
    <motor ctrllimited="false"/>   <!-- velocity actuators -->
  </default>

  <worldbody>
    <!-- Hard ground plane (backup) -->
    <geom name="ground" type="plane" size="10 10 0.2" rgba="0.20 0.20 0.20 1" friction="{ground_mu}"/>
    
    <!-- Compliant soft mat for ribs to bite into -->
    <body pos="0 0 0.002">
      <geom name="mat" type="box" size="8 8 0.002" rgba="0.28 0.28 0.28 1"
            friction="{mat_mu}" solref="0.004 2.0" solimp="0.90 0.95 0.001"/>
    </body>

    <!-- Ridges: enable for X-only travel; disable for Y travel to avoid hopping -->
    {('\n'.join([f"    <geom name=\"ridge_{i}\" type=\"box\" size=\"8 {ridge_width/2:.4f} {ridge_height/2:.4f}\" pos=\"0 {i*groove_pitch:.4f} {0.002 + ridge_height/2:.4f}\" rgba=\"0.22 0.22 0.22 1\" friction=\"{mat_mu}\" solref=\"0.003 2.0\" solimp=\"0.92 0.97 0.002\"/>" for i in range(-num_ridges, num_ridges+1)])) if enable_ridges else ''}

    <body name="snake_base" pos="0 0 {z_base}">
      <!-- Allow translation in X, Y, Z (no rotations) so we can demonstrate X vs Y motion -->
      <joint name="slide_x" type="slide" axis="1 0 0" limited="false" damping="0.2"/>
      <joint name="slide_y" type="slide" axis="0 1 0" limited="false" damping="0.8"/>
      <joint name="slide_z" type="slide" axis="0 0 1" limited="false" damping="2.0"/>

      <!-- mid chassis (no contact) - heavier for more normal force -->
      <geom name="chassis" type="box" size="0.05 0.06 0.02" pos="0 0 0.03"
            rgba="0.1 0.15 0.2 1" mass="1.0" contype="0" conaffinity="0"/>

      <!-- CENTER OBJECT: Connects screws, adds mass/inertia -->
      <!-- Note: Y motion blocked by removing slide_y joint, not by friction -->
      <geom name="center_keel" type="box" size="0.02 0.08 0.04" pos="0 0 0"
            rgba="0.3 0.3 0.35 1" mass="0.20"/>

      <!-- Left half: RIGHT-HAND (red), extends along X -->
      {screw_xml("screw_right","jr","0.85 0.35 0.35 1", +1, -(L + center_gap/2), "0.95 0.1 0.1 1")}

      <!-- Right half: LEFT-HAND (blue), extends along X -->
      {screw_xml("screw_left","jl","0.30 0.65 0.95 1",   -1, +(L + center_gap/2), "0.1 0.6 1.0 1")}
    </body>
  </worldbody>

  <!-- Velocity motors: ctrl = target rad/s (qvel target) -->
  <actuator>
    <velocity name="motor_r" joint="jr" kv="150"/>
    <velocity name="motor_l" joint="jl" kv="150"/>
  </actuator>

  <visual>
    <map znear="0.005" zfar="100"/>
  </visual>
</mujoco>
'''
Path("snake_screws.xml").write_text(xml)
print("Wrote snake_screws.xml (ribs-only contact + compliant mat for forward thrust).")
