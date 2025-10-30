# Physics Implementation Documentation

## Overview
This document explains how the physics equations from `equations.txt` have been implemented in the screw vehicle simulation.

## Physics Parameters

The `ScrewPhysics` class in `run.py` implements all the equations from your physics model:

### Geometric Parameters (from equations 10-16)
- **h_b**: Blade height (m) - mean height of helix blade above base drum
  - Default: 0.055 m (matching the helical ridge geometry in XML)
- **D_d**: Drum diameter (m) - base drum diameter
  - Default: 0.10 m (2 × cylinder radius of 0.05 m)
- **k**: Geometric ratio = h_b / D_d
- **θ**: Helix angle (radians) calculated from: tan(θ) = h / (π × k × D_d)

### Friction and Forces
- **μ**: Coefficient of friction between screw and ground
  - Default: 0.7 (matching geom friction in XML)
- **α_m**: Slope angle (radians) - terrain incline
  - Default: 0.0 (flat terrain)
- **W**: Weight per screw segment (N)
  - Calculated from model: W = total_mass × 9.81

## Equation Implementation

### 1. Axial Force Balance (Equations 7 & 19)
```
F = W (cos(α_m) + μ sin(α_m)) / (cos(θ) - μ sin(θ))
```
**Implementation**: `ScrewPhysics.calculate_axial_force()`
- Calculates the axial force required to overcome friction and support weight
- Used in power loss calculations

### 2. Helix Angle (Equations 10 & 12)
```
tan(θ) = h / (π × k × D_d)
k = h_b / D_d
```
**Implementation**: `ScrewPhysics.__init__()`
- Automatically calculated during initialization
- Determines the mechanical advantage of the screw thread

### 3. Power Loss Components (Equations 24-34)

#### a) Drum Friction (Equation 24)
```
P_drum = (μW cos(θ) / sin(θ)) × π × D_d × n_d
```
**Implementation**: Lines 75-78 in `calculate_required_torque()`
- Models power loss from drum base sliding on ground
- n_d = rotation speed (rev/s)

#### b) Helix Blade Friction (Equation 28)
```
P_blade = (μF) × π × (D_d + h_b) × n / cos(θ)
```
**Implementation**: Lines 81-85 in `calculate_required_torque()`
- Models power loss from blade friction at mean height
- Uses the calculated axial force F

#### c) Work Against Gravity (Equation 31)
```
P_w = W × v × sin(θ)
```
**Implementation**: Lines 88-89 in `calculate_required_torque()`
- Models work done lifting the vehicle along helix angle
- v = forward velocity (m/s)

### 4. Total Power and Torque (Equations 34, 42, 47)
```
P_total = P_drum + P_blade + P_w
ω = 2πn / 60  (convert RPM to rad/s)
T = P_total / ω
```
**Implementation**: Lines 91-100 in `calculate_required_torque()`
- Sums all power losses
- Converts to required torque based on angular velocity

### 5. Non-dimensionalized Power (Equations 38-39)
```
γ = total power / (Wv)
γ = [A cos(θ) / sin(θ)] × [μ cos(θ) / (1 - μ tan(θ))] + sin(θ)
```
**Note**: This non-dimensional form is implicitly included in the power calculations above.

## Simulation Features

### Real-time Physics Display
The simulation outputs physics-based metrics every 0.5 seconds:
- **ω_r, ω_l**: Angular velocities of right and left screws (rad/s)
- **vx**: Forward velocity (m/s)
- **Δx**: Total displacement (m)
- **F**: Calculated axial force (N)
- **T_req**: Required torque based on physics equations (N⋅m)

### Control Modes
Set `use_physics_torque = True` (line 171) to use physics-based torque calculation.
- When True: Torque is calculated from the physics equations
- When False: Simple velocity control without physics

### Parameter Tuning
You can adjust physics parameters in `main()` (lines 131-137):
```python
physics = ScrewPhysics(
    h_b=0.055,      # Blade height
    D_d=0.10,       # Drum diameter
    mu=0.7,         # Friction coefficient
    alpha_m=0.0,    # Slope angle (radians)
    W=W             # Weight (automatically calculated)
)
```

### Target Rotation Speed
Set rotation speed at line 148:
```python
target_rpm = 60.0  # Target rotation speed in RPM
```

## Model-Equation Alignment

### Geometry Matching
The XML model (`snake_screws.xml`) geometry matches the physics parameters:
- Cylinder radius: 0.05 m → D_d = 0.10 m diameter
- Helical ridge size: 0.02 m → approximates h_b = 0.055 m blade height
- Friction values: 1.0-3.5 → matches μ = 0.7 friction coefficient

### Actuator Configuration
- Two velocity actuators: `motor_r` and `motor_l`
- Gain kv = 80 (tunable for desired torque response)
- Counter-rotating screws for forward motion

## Running the Simulation

### With Viewer (default)
```bash
python3 run.py
```

### Headless Mode
```bash
HEADLESS=1 python3 run.py
```

## Validation Checklist

The implementation correctly models:
- ✅ Helix angle calculation from geometry
- ✅ Axial force from friction and weight
- ✅ Drum friction power loss
- ✅ Blade friction power loss
- ✅ Gravitational work power loss
- ✅ Total torque requirement from power/velocity relationship
- ✅ Real-time display of physics metrics

## Future Enhancements

Potential improvements:
1. Implement torque control actuators instead of velocity control
2. Add terrain slope variation (α_m parameter)
3. Test on inclined planes
4. Add towing load simulation
5. Optimize blade geometry for different terrain types
6. Compare predicted vs. actual torque values

## References

See `equations.txt` for the complete physics derivation and assumptions.

