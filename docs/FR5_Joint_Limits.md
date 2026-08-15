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

> **Sliders only, since roadmap 7.2.** These govern `gui_panel.JOINT_LIMITS` and
> nothing else. Every **solver** call — the toolpath precompute, the manual IK
> panel, and the exchange spec's joint-limit rejection row — passes
> `geometry_backend.PHYSICAL_JOINT_LIMITS`, the table above.
>
> Until 7.2 the solver borrowed this constant, so it rejected poses the arm can
> physically reach: J2 by ~134° and J4 by ~94°. Measured effect of the split —
> **425 valid IK branches vs 207** over an 80-pose sample.
>
> The two are not interchangeable, and the narrowing here is still deliberate:
> J2's shallow floor is a hand-rolled collision proxy, and dragging a slider has
> no continuity or clearance check behind it. Note that this project has **no
> mesh-vs-mesh collision anywhere**, so that proxy is doing more work than its
> comment suggests. See `settled.md` S1.44.
