# Environment Selection

You can now simulate the snake in different ground environments!

## Available Environments

### Normal (Default)
- **Visual**: Blue checker pattern with cross marks
- **Friction**: High (6.0) - hard surface
- **Physics**: High grip, optimal performance

```bash
mjpython run.py
# or explicitly:
GROUND=normal mjpython run.py
```

### Sand
- **Visual**: Tan/beige grainy texture
- **Friction**: Lower (2.5) - loose sand
- **Physics**: Reduced grip, more slippage

```bash
GROUND=sand mjpython run.py
```

## Combining with Other Options

You can combine environment selection with rotation modes:

```bash
# Sand environment with opposite rotation (X-motion)
GROUND=sand ROTATION_MODE=opposite mjpython run.py

# Sand environment with same rotation (Y-motion, positive)
GROUND=sand R_SIGN=1 L_SIGN=1 mjpython run.py

# Sand environment with reverse Y-motion
GROUND=sand R_SIGN=-1 L_SIGN=-1 mjpython run.py
```

## Physics Differences

| Parameter | Normal | Sand | Explanation |
|-----------|--------|------|-------------|
| **Friction** (slide/roll/torsion) | 6.0/0.01/0.002 | 0.9/0.12/0.05 | Sand: lower slide, higher roll/torsion |
| **solimp** (contact stiffness) | 0.96/0.96/0.01 | 0.8/0.8/0.01 | Sand: softer, more compliant surface |
| **solref** (contact dynamics) | 0.003/1 | 0.003/1 | Same: quick contact response |
| **Physics μ** | 0.7 | 0.45 | Sand: reduced effective friction |

### What These Parameters Mean:

**Friction [slide, roll, torsion]:**
- **Slide (0.9)**: Lower than hard surface - sand grains can shift
- **Roll (0.12)**: Much higher - object sinks and plows through sand
- **Torsion (0.05)**: Much higher - rotating in place displaces sand

**solimp [dmin, dwidth, width]:**
- **0.8 vs 0.96**: Softer contact - sand compresses under weight
- Creates a "sinking" effect as screws push into the surface

**Result:** Expect slower movement, more wheel slip, and visible "digging" behavior on sand!

