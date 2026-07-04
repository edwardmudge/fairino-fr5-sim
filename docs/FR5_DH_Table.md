# FR5 Standard DH Parameters

Reference: Huang et al. (2026). All lengths in **mm**, angles in **radians**.

## DH Parameter Table

| Link | a (mm) | α (rad) | d (mm) | θ offset |
|------|--------|---------|--------|----------|
| 1 | 0 | +π/2 | 152 | 0 |
| 2 | −425 | 0 | 0 | 0 |
| 3 | −395 | 0 | 0 | 0 |
| 4 | 0 | +π/2 | 102 | 0 |
| 5 | 0 | −π/2 | 102 | 0 |
| 6 | 0 | 0 | 100 | 0 |

## Standard DH Homogeneous Transform

From frame {i−1} to frame {i}:

```
        ┌                                              ┐
        │ cos θ    −sin θ·cos α    sin θ·sin α    a·cos θ │
T_i =   │ sin θ     cos θ·cos α   −cos θ·sin α    a·sin θ │
        │   0          sin α          cos α           d     │
        │   0            0              0             1     │
        └                                              ┘
```

Where:
- **a** = link length (mm)
- **α** = link twist (rad)
- **d** = joint offset (mm)
- **θ** = joint angle (rad) = `deg2rad(joint_angle_deg) + offset`

## FK Chain

```
T_0_6 = T_0_1 · T_1_2 · T_2_3 · T_3_4 · T_4_5 · T_5_6
```

Each `T_0_i` gives link i's frame relative to the base frame.

## Zero-Position Verification

When `joints = [0, 0, 0, 0, 0, 0]` degrees:

```
T_0_6 position = [-820.0, -202.0, 50.0] mm
```

Use this to verify your FK implementation is correct (confirmed by manual chain multiplication).
