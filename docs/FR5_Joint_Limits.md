# FR5 Joint Limits

Measured from the real teach pendant (2026-04-25 verified).

## Physical Joint Limits

| Joint | Min (°) | Max (°) | Note |
|-------|---------|---------|------|
| J1 | −174 | +174 | |
| J2 | −264 | +84 | Asymmetric! |
| J3 | −159 | +159 | |
| J4 | −264 | +84 | Asymmetric! |
| J5 | −174 | +174 | |
| J6 | −174 | +174 | |

## Home Position

```
Home = [0, 0, 0, 0, 90, 0] degrees
```

J5 = 90° makes the tool point straight down (−Z direction).
When J1–J4 and J6 are all zero, J5 = 90° rotates the flange from horizontal to vertical — this is the standard "tool pointing down" configuration used in industrial robotics.

## Practical Slider Ranges

For ImGui sliders during development, a safe working range is:

```
J1: [-170, 170]
J2: [-130, 80]    (avoid deep negative for collision safety)
J3: [-155, 155]
J4: [-170, 80]
J5: [-170, 170]
J6: [-170, 170]
```
