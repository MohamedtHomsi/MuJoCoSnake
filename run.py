"""
High-level overview
===================

This script runs a MuJoCo simulation of a screw-driven snake robot and overlays
the analytical physics model from equations.txt onto the sim for interpretability.

What is modeled from equations.txt (references shown inline in code):
- Geometry / helix angle:      tan(θ) = h / (π·k·D_d), with k = h_b / D_d   (Eqs. 10–12)
- Axial force balance:         F = W (cos α_m + μ sin α_m) / (cos θ − μ sin θ)  (Eqs. 7, 19)
- Power losses and totals:     P_total = P_drum + P_blade + P_w              (Eqs. 24–34)
    · Drum friction:           P_drum = (μW cos θ / sin θ) · π · D_d · n     (Eq. 24)
    · Blade friction:          P_blade = (μF) · π · (D_d + h_b) · n / cos θ  (Eq. 28)
    · Gravity work:            P_w = W · v · sin θ                           (Eq. 31)
- Torque from power and speed: T = P_total / ω, ω = 2πn                      (Eqs. 42, 47)

Simulation control (non-derivative of equations):
- TARGET_RPM env var sets commanded joint velocity target (degenerate to rad/s internally)
- THRUST_SCALE scales the applied analytical axial force (a convenience to see motion)
- ROTATION_MODE, R_SIGN, L_SIGN allow choosing same/opposite/concrete rotation directions

Important: The physics formulas themselves are unchanged; comments and small convenience
controls help observe their effect without altering the derivations.
"""

import os, time, numpy as np
os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco
import mujoco.viewer as viewer

MODEL = "snake_screws.xml"

# Physics parameters from equations.txt
class ScrewPhysics:
    """Physics model based on equations.txt.

    Implements the equations listed above. Key outputs:
    - Helix angle θ from the geometric relations (Eqs. 10–12)
    - Axial force F (Eqs. 7, 19)
    - Power components and total (Eqs. 24–34), and torque T (Eqs. 42, 47)
    """
    def __init__(self, h_b=0.055, D_d=0.10, mu=0.7, alpha_m=0.0, W=5.0):
        """
        Parameters:
        h_b: blade height (m) - mean height of helix blade above base drum
        D_d: drum diameter (m) - base drum diameter
        mu: coefficient of friction between screw and ground
        alpha_m: slope angle (radians) - terrain slope
        W: weight per screw segment (N)
        """
        self.h_b = h_b
        self.D_d = D_d
        self.mu = mu
        self.alpha_m = alpha_m
        self.W = W
        
        # Calculate helix angle from geometry (Eqs. 10–12)
        # k = h_b / D_d
        # tan(θ) = h / (π · k · D_d)
        # For simplicity, assume h ≈ h_b (one pitch height)
        k = h_b / D_d
        self.theta = np.arctan(h_b / (np.pi * k * D_d))
        
        print(f"Screw Physics Initialized:")
        print(f"  Blade height h_b = {h_b:.4f} m")
        print(f"  Drum diameter D_d = {D_d:.4f} m")
        print(f"  Coefficient of friction μ = {mu:.3f}")
        print(f"  Helix angle θ = {np.degrees(self.theta):.2f}°")
        print(f"  Weight per screw W = {W:.2f} N")
    
    def calculate_axial_force(self):
        """Axial force F (Eqs. 7, 19).

        F = W (cos α_m + μ sin α_m) / (cos θ − μ sin θ)
        """
        # F = W (cos(α_m) + μ sin(α_m)) / (cos(θ) - μ sin(θ))
        numerator = self.W * (np.cos(self.alpha_m) + self.mu * np.sin(self.alpha_m))
        denominator = np.cos(self.theta) - self.mu * np.sin(self.theta)
        
        if abs(denominator) < 1e-6:
            print("Warning: denominator near zero in force calculation")
            return 0.0
        
        F = numerator / denominator
        return F
    
    def calculate_required_torque(self, v, n):
        """Required torque T from power balance (Eqs. 24–34, 42, 47).

        Inputs
        - v: forward speed (m/s)
        - n: rotation speed (rev/s)

        Steps (from equations.txt)
        - Compute power components:
            P_drum (Eq. 24), P_blade (Eq. 28), P_w (Eq. 31)
        - Sum P_total (Eq. 34)
        - Convert to torque using T = P_total / ω (Eqs. 42, 47) with ω = 2πn
        """
        if abs(v) < 1e-6 or abs(n) < 1e-6:
            return 0.0
        
        F = self.calculate_axial_force()
        omega = 2 * np.pi * n  # angular velocity (rad/s)
        
        # Power losses:
        # a) Drum friction (equation 24)
        # P_drum = (μW cos(θ) / sin(θ)) * π * D_d * n_d
        # where n_d = n (assuming drum rotates at same speed)
        if abs(np.sin(self.theta)) > 1e-6:
            P_drum = (self.mu * self.W * np.cos(self.theta) / np.sin(self.theta)) * np.pi * self.D_d * n
        else:
            P_drum = 0.0
        
        # b) Helix blade friction (equation 28)
        # P_blade = (μF) * π * (D_d + h_b) * n / cos(θ)
        if abs(np.cos(self.theta)) > 1e-6:
            P_blade = (self.mu * F) * np.pi * (self.D_d + self.h_b) * n / np.cos(self.theta)
        else:
            P_blade = 0.0
        
        # c) Work against gravity (equation 31)
        # P_w = W * v * sin(θ)
        P_w = self.W * v * np.sin(self.theta)
        
        # Total power (equation 34)
        P_total = P_drum + P_blade + P_w
        
        # Torque from power (equation 42, 47)
        # T = P / ω
        if abs(omega) > 1e-6:
            T = P_total / omega
        else:
            T = 0.0
        
        return T

def name2id(m, obj, nm):
    """Look up name with error checking"""
    i = mujoco.mj_name2id(m, obj, nm)
    if i < 0:
        raise RuntimeError(f"Couldn't find '{nm}' (type {obj}) in model.")
    return i

def try_combo(m, base_qpos, steps, w, sign_r, sign_l, act_r, act_l):
    """Test a sign combination and return Δx"""
    dtest = mujoco.MjData(m)
    dtest.qpos[:] = base_qpos
    mujoco.mj_forward(m, dtest)
    x0 = float(dtest.qpos[0])
    for _ in range(steps):
        dtest.ctrl[act_r] = sign_r * w
        dtest.ctrl[act_l] = sign_l * w
        mujoco.mj_step(m, dtest)
    return float(dtest.qpos[0] - x0)

def main():
    m = mujoco.MjModel.from_xml_path(MODEL)
    d = mujoco.MjData(m)
    
    # Initialize physics model with parameters matching the XML geometry
    # W selection (weight per screw segment) used by the equations:
    # - We compute only the snake subtree mass (exclude ground/mat/world)
    # - Then partition across screws × axial segments for the segment weight
    #   (this keeps the equations numerically meaningful in the sim)
    base_bid_for_mass = name2id(m, mujoco.mjtObj.mjOBJ_BODY, "snake_base")
    snake_mass = m.body_subtreemass[base_bid_for_mass]
    total_weight = snake_mass * 9.81
    # Assume 2 screws × 4 axial segments ≈ 8 segments; distribute total weight
    segments_per_screw = 4
    screws = 2
    W = total_weight / (screws * segments_per_screw)
    
    physics = ScrewPhysics(
        h_b=0.055,      # blade height (m) - from the helical ridges in XML
        D_d=0.10,       # drum diameter (m) - 2 * cylinder radius
        mu=0.7,         # coefficient of friction (matches geom friction in XML)
        alpha_m=0.0,    # slope angle (radians) - flat terrain for now
        W=W             # weight per screw (N)
    )

    # Start slightly above ground, let it settle (contact warm-up)
    d.qpos[:2] = np.array([0, 0.10])  # x=0, z=0.1
    mujoco.mj_forward(m, d)

    act_r = name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_r")
    act_l = name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_l")
    j_r   = name2id(m, mujoco.mjtObj.mjOBJ_JOINT,    "jr")
    j_l   = name2id(m, mujoco.mjtObj.mjOBJ_JOINT,    "jl")
    base_bid = name2id(m, mujoco.mjtObj.mjOBJ_BODY,   "snake_base")
    dof_r = m.jnt_dofadr[j_r]
    dof_l = m.jnt_dofadr[j_l]

    # Commanded rotation speed (control input, not from equations):
    # TARGET_RPM environment variable lets you tune without code edits.
    env_rpm = os.environ.get("TARGET_RPM")
    try:
        target_rpm = float(env_rpm) if env_rpm is not None else 60.0  # default slower
    except ValueError:
        target_rpm = 60.0
    target_n = target_rpm / 60.0  # Convert to rev/s
    target_omega = 2 * np.pi * target_n  # rad/s
    
    # Let body settle into contact
    print("\nSettling into contact...")
    for _ in range(int(0.5 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    base_qpos = d.qpos.copy()
    
    # Auto-detect correct motor signs for +X motion (probe both opposite-sign options)
    print("Auto-detecting motor signs for +X motion...")
    test_steps = int(0.75 / m.opt.timestep)
    dx1 = try_combo(m, base_qpos, test_steps, target_omega, +1.0, -1.0, act_r, act_l)
    dx2 = try_combo(m, base_qpos, test_steps, target_omega, -1.0, +1.0, act_r, act_l)
    sign_r, sign_l = (+1.0, -1.0) if dx1 >= dx2 else (-1.0, +1.0)
    # Optional override: ROTATION_MODE=opposite|same (quick toggle)
    rotation_mode = os.environ.get("ROTATION_MODE", "opposite").strip().lower()
    if rotation_mode == "same":
        sign_l = sign_r
    
    # Optional explicit per-screw overrides via env: R_SIGN, L_SIGN in {+1, -1}
    # These take precedence over ROTATION_MODE. Useful to test all combinations.
    def parse_env_sign(var_name: str):
        value = os.environ.get(var_name)
        if not value:
            return None
        s = value.strip().lower()
        if s in {"+1", "+", "1", "pos", "+1.0"}:
            return +1.0
        if s in {"-1", "-", "neg", "-1.0"}:
            return -1.0
        return None

    override_r = parse_env_sign("R_SIGN")
    override_l = parse_env_sign("L_SIGN")
    if override_r is not None:
        sign_r = override_r
    if override_l is not None:
        sign_l = override_l

    print(f"✓ Rotation mode: {rotation_mode}  → Motor signs: R={sign_r:+.0f}, L={sign_l:+.0f} (probe Δx={max(dx1,dx2):+.3f} m)")
    
    # Reset to settled state
    d.qpos[:] = base_qpos
    mujoco.mj_forward(m, d)
    
    last = 0.0
    use_physics_torque = True     # keep velocity target on joints
    apply_thrust_force = True     # apply axial force from analytical model for motion visualization
    # THRUST_SCALE (env) scales the applied analytical thrust; not part of the derivation,
    # but helpful to tune translational speed without changing equations.
    env_ts = os.environ.get("THRUST_SCALE")
    try:
        thrust_scale = float(env_ts) if env_ts is not None else 0.03
    except ValueError:
        thrust_scale = 0.03
    anti_hop_push = 0.0           # small downward force when moving in +Y

    def apply_analytical_thrust():
        """Apply the analytical axial force F along an axis based on rotation pattern.

        Direction logic (demonstration control, not from equations):
        - both screws positive → +Y motion
        - opposite signs       → +X motion
        The magnitude derives from the axial force formula F (Eqs. 7, 19),
        scaled by THRUST_SCALE for comfortable speeds.
        """
        # Decide thrust direction based on screw rotation signs
        omega_r = d.qvel[dof_r]
        omega_l = d.qvel[dof_l]
        # Both positive → move +Y. Opposite signs → move +X (based on your spec)
        Fx, Fy, Fz = 0.0, 0.0, 0.0
        F_axial = physics.calculate_axial_force() * thrust_scale
        if omega_r > 0 and omega_l > 0:
            Fy = F_axial
            Fz = -anti_hop_push
        elif omega_r * omega_l < 0:
            Fx = F_axial
            Fz = 0.0
        # Clear and apply
        d.xfrc_applied[base_bid, 0] = Fx
        d.xfrc_applied[base_bid, 1] = Fy
        d.xfrc_applied[base_bid, 2] = Fz
        d.xfrc_applied[base_bid, 3] = 0.0
        d.xfrc_applied[base_bid, 4] = 0.0
        d.xfrc_applied[base_bid, 5] = 0.0

    headless = os.environ.get("HEADLESS", "0") == "1"
    if headless:
        print("Running headless (set HEADLESS=0 to use viewer).")
        x_start = d.qpos[0]
        for _ in range(3000):
            if use_physics_torque:
                # Physics-based control
                vx = d.qvel[0]
                omega_r = d.qvel[m.jnt_dofadr[j_r]]
                n_r = omega_r / (2 * np.pi)  # rev/s
                
                # Calculate required torque for each screw
                T_required = physics.calculate_required_torque(abs(vx), abs(n_r))
                
                # Apply torque (split between two screws)
                d.ctrl[act_r] = sign_r * target_omega
                d.ctrl[act_l] = sign_l * target_omega
            else:
                # Simple velocity control
                d.ctrl[act_r] = sign_r * target_omega
                d.ctrl[act_l] = sign_l * target_omega

            # Apply axial thrust force to the base using analytical model
            if apply_thrust_force:
                apply_analytical_thrust()
            
            mujoco.mj_step(m, d)
        
        dof_r = m.jnt_dofadr[j_r]
        dof_l = m.jnt_dofadr[j_l]
        dx = d.qpos[0] - x_start
        print(f"Done. ω_r={d.qvel[dof_r]:.2f} rad/s, ω_l={d.qvel[dof_l]:.2f} rad/s, Δx={dx:+.3f}m")
        return

    print("\n=== PHYSICS-BASED SCREW LOCOMOTION ===")
    print(f"Target rotation: {target_rpm:.1f} RPM ({target_omega:.2f} rad/s)")
    print(f"Control mode: {'Physics-based torque' if use_physics_torque else 'Simple velocity'}")
    print(f"Motor signs: R={sign_r:+.0f}ω, L={sign_l:+.0f}ω\n")

    with viewer.launch_passive(m, d) as v:
        # Camera helpers: follow the robot automatically for easier recording
        cam_track = os.environ.get("CAM_TRACK", "1").strip() not in {"0", "false", "False"}
        if cam_track:
            try:
                v.cam.trackbodyid = base_bid
                v.cam.distance = float(os.environ.get("CAM_DISTANCE", "1.6"))
                v.cam.elevation = float(os.environ.get("CAM_ELEV", "-15"))
                v.cam.azimuth = float(os.environ.get("CAM_AZIMUTH", "90"))
                print(f"Camera: tracking 'snake_base' (distance={v.cam.distance}, elev={v.cam.elevation}, azim={v.cam.azimuth})")
            except Exception as _:
                pass
        x_start = d.qpos[0]
        
        while v.is_running():
            # Get current state
            vx = d.qvel[0]  # Forward velocity (m/s)
            dof_r = m.jnt_dofadr[j_r]
            dof_l = m.jnt_dofadr[j_l]
            omega_r = d.qvel[dof_r]  # rad/s
            omega_l = d.qvel[dof_l]  # rad/s
            n_r = omega_r / (2 * np.pi)  # rev/s
            
            if use_physics_torque:
                # Calculate required torque based on physics equations
                T_required = physics.calculate_required_torque(abs(vx), abs(n_r))
                
                # Apply velocity control (MuJoCo will calculate torque)
                # For now, maintain target angular velocity
                d.ctrl[act_r] = sign_r * target_omega
                d.ctrl[act_l] = sign_l * target_omega
            else:
                # Simple velocity control
                d.ctrl[act_r] = sign_r * target_omega
                d.ctrl[act_l] = sign_l * target_omega

            # Apply axial thrust force to the base using analytical model
            if apply_thrust_force:
                apply_analytical_thrust()
            
            mujoco.mj_step(m, d)
            v.sync()
            time.sleep(0.0001)
            
            if d.time - last > 0.5:
                last = d.time
                x_pos = d.qpos[0]
                dx = x_pos - x_start
                
                # Calculate physics metrics
                F = physics.calculate_axial_force()
                T_calc = physics.calculate_required_torque(abs(vx), abs(n_r)) if abs(n_r) > 1e-3 else 0.0
                
                print(f"t={d.time:5.2f}s  ω_r={omega_r:6.2f}  ω_l={omega_l:6.2f}  "
                      f"vx={vx:+.3f} m/s  Δx={dx:+.3f}m  F={F:.2f}N  T_req={T_calc:.3f}N⋅m")

if __name__ == "__main__":
    main()

