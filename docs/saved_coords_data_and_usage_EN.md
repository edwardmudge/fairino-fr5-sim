# FR5 Saved Coordinates: Data Description and Usage

> Audience: Collaborators who need to reproduce the FR5 coordinate system in a pure simulation framework
>
> Data source: `assets/calib/active_tcp_wobj.json` (MultiHeadTCPStore, Single Source of Truth)
>
> Collected: 2026-05-28, from the real robot (IP: 192.168.58.2, mode: real)

---

## 1. Data Overview

The currently saved coordinate data includes **1 User Frame (WObj)** and **3 TCPs (Tool Center Points)**:

### 1.1 User Frame (Workpiece Coordinate System)

| Field | Value |
|------|-----|
| user_index | 1 |
| X (mm) | 649.456 |
| Y (mm) | 133.762 |
| Z (mm) | 322.778 |
| Rx (deg) | -0.369 |
| Ry (deg) | 0.329 |
| Rz (deg) | -89.080 |

**Meaning**: The User Frame defines the pose of the G-code workpiece coordinate system relative to the robot's Base Frame (base center). All XYZ coordinates in G-code are defined relative to this User Frame.

### 1.2 TCP Offsets (Tool Center Point Offsets)

| tool_index | X (mm) | Y (mm) | Z (mm) | Rx (deg) | Ry (deg) | Rz (deg) | Description |
|------------|--------|--------|--------|----------|----------|----------|------|
| 0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | Default tool (no offset = flange origin) |
| 1 | -134.777 | 96.448 | 106.334 | 86.647 | -13.136 | 60.612 | Print head 1 |
| 2 | -134.777 | 96.448 | 106.334 | 86.647 | -13.136 | 60.612 | Print head 2 (currently the same as head 1) |

**Meaning**: The TCP offset defines the offset of the tool nozzle tip relative to the Flange Frame. The flange face is the standard reference plane at the end of the robot's 6th axis.

---

## 2. Coordinate System Architecture

```
    Base Frame (robot base center = world coordinate origin)
       |
       |--- DH kinematic chain (6 axes) ---> Flange Frame
       |                              |
       |                              +-- T_flange_to_tool --> Tool Frame (TCP, nozzle tip)
       |
       +-- T_base_to_user ----------> User Frame (workpiece coordinate system)
                                         |
                                         +-- G-code coordinates defined here
```

**Key relationships**:
- `T_tool_base = T_base_flange @ T_flange_to_tool` — Tool pose in the world coordinate system
- `p_base = T_base_to_user @ p_user` — Workpiece coordinates converted to world coordinates
- The G-code target pose is defined in the User Frame; the IK solver converts it internally to the Base Frame

---

## 3. Converting a 6D Pose to a 4x4 Homogeneous Transformation Matrix

All 6D poses `[x, y, z, rx, ry, rz]` are converted to a 4x4 matrix using the same convention:

```python
import numpy as np

def pose_to_matrix(x, y, z, rx, ry, rz):
    """
    6D pose -> 4x4 homogeneous transform.
    
    x, y, z: mm
    rx, ry, rz: degrees
    Rotation order: R = Rz(rz) @ Ry(ry) @ Rx(rx)
    Equivalent to: intrinsic XYZ / extrinsic ZYX
    """
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    rx_r, ry_r, rz_r = np.radians([rx, ry, rz])

    Rx = np.array([[1,            0,             0],
                   [0,  np.cos(rx_r), -np.sin(rx_r)],
                   [0,  np.sin(rx_r),  np.cos(rx_r)]])
    Ry = np.array([[ np.cos(ry_r), 0, np.sin(ry_r)],
                   [0,             1,            0],
                   [-np.sin(ry_r), 0, np.cos(ry_r)]])
    Rz = np.array([[np.cos(rz_r), -np.sin(rz_r), 0],
                   [np.sin(rz_r),  np.cos(rz_r), 0],
                   [0,             0,            1]])
    T[:3, :3] = Rz @ Ry @ Rx
    return T
```

**Note**: This rotation convention is consistent with the return format of the Fairino SDK `GetTCPOffset()` / `GetWObjOffset()`. If your simulation framework also defines a User Frame and TCP Frame, confirm whether the rotation order is the same — if not, you will need to convert the rotation representation.

---

## 4. How to Use This Data in a Pure Simulation Framework

### 4.1 Constructing the Transformation Matrices

```python
# User Frame (WObj) — workpiece coordinate system
T_base_to_user = pose_to_matrix(649.456, 133.762, 322.778, -0.369, 0.329, -89.080)

# TCP offset for tool 1 — print head offset relative to the flange
T_flange_to_tool1 = pose_to_matrix(-134.777, 96.448, 106.334, 86.647, -13.136, 60.612)

# TCP offset for tool 0 — no offset (flange origin)
T_flange_to_tool0 = np.eye(4)
```

### 4.2 G-code Coordinates -> World Coordinates (Base Frame)

The target pose `[gx, gy, gz, grx, gry, grz]` in G-code is defined in the User Frame:

```python
# G-code target pose (User Frame)
target_user = pose_to_matrix(gx, gy, gz, grx, gry, grz)

# Convert to Base Frame
target_base = T_base_to_user @ target_user
```

### 4.3 FK: Joint Angles -> TCP World Pose

```python
# Given joint angles joints_deg = [j1, j2, j3, j4, j5, j6]

# Step 1: DH forward kinematics -> flange pose in the Base Frame
T_base_flange = forward_kinematics(joints_deg)  # computed from DH parameters

# Step 2: Flange -> TCP
T_base_tcp = T_base_flange @ T_flange_to_tool1

# Step 3 (if conversion to User Frame is needed): 
T_user_tcp = np.linalg.inv(T_base_to_user) @ T_base_tcp
```

### 4.4 IK: TCP Target Pose -> Joint Angles

```python
# IK input: target TCP pose in the User Frame
target_user_6d = [gx, gy, gz, grx, gry, grz]

# Step 1: User Frame -> Base Frame
T_target_base = T_base_to_user @ pose_to_matrix(*target_user_6d)

# Step 2: TCP -> Flange (remove the TCP offset)
T_target_flange = T_target_base @ np.linalg.inv(T_flange_to_tool1)

# Step 3: Solve IK on T_target_flange -> joints
joints = your_ik_solver(T_target_flange)
```

---

## 5. FR5 DH Parameters (Standard DH)

If you need to reproduce FK/IK in a pure simulation, here is the FR5's Standard DH table:

| Joint | a (mm) | alpha (rad) | d (mm) |
|-------|--------|-------------|--------|
| 1 | 0.0 | pi/2 | 152.0 |
| 2 | -425.0 | 0.0 | 0.0 |
| 3 | -395.0 | 0.0 | 0.0 |
| 4 | 0.0 | pi/2 | 102.0 |
| 5 | 0.0 | -pi/2 | 102.0 |
| 6 | 0.0 | 0.0 | 100.0 |

Single-axis transformation matrix (Standard DH):

```python
def dh_matrix(a, alpha, d, theta):
    """Standard DH transform for one joint."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,      ca,      d],
        [0.0,     0.0,     0.0,    1.0],
    ])
```

FK computation:
```python
def forward_kinematics(joints_deg):
    """6 joint angles (degrees) -> T_base_flange (4x4)."""
    T = np.eye(4)
    for i in range(6):
        theta = np.deg2rad(joints_deg[i])
        T = T @ dh_matrix(DH_A[i], DH_ALPHA[i], DH_D[i], theta)
    return T
```

### Joint Limits (Teach Pendant Ground Truth, Verified 2026-04-25)

| Joint | Min (deg) | Max (deg) | Notes |
|-------|-----------|-----------|------|
| J1 | -174 | 174 | |
| J2 | -264 | 84 | Asymmetric! |
| J3 | -159 | 159 | |
| J4 | -264 | 84 | Asymmetric! |
| J5 | -174 | 174 | |
| J6 | -174 | 174 | |

---

## 6. Frame4 Correction (Historical Reference, Not Currently Used)

> **Important: for External IK data exchange, use the standard DH in §5 (d4=102) directly — do not apply the correction in this section.** This section is kept only as a historical record; it describes the behavior of the old SDK 3.9.4, deprecated after migrating to SDK 3.8.7 on 2026-05-03.

In the old SDK 3.9.4, the exposed flange was not equal to the raw DH6 endpoint. A correction used to be inserted between Joint 4 and Joint 5:

```
T_base_flange = T_0_4 × T_frame4_corr × A5 × A6
```

Where `T_frame4_corr` used to be a `[0, +28, 0] mm` translation in the frame4 local coordinate system, equivalent to d4 = 130mm.

**Current status (after 2026-05-03)**: The system has migrated to SDK 3.8.7, using the standard d4=102mm, with the correction now zero. The above is kept only as a historical record.

**For data exchange: use the standard DH parameters from §5 (d4=102) directly, with no correction applied.**

---

## 7. Numerical Verification Method

Compare the FK results from your simulation against the following reference values (error should be < 0.01mm):

**Test case**: zero position `joints = [0, 0, 0, 0, 0, 0]`

```python
T_flange_zero = forward_kinematics([0, 0, 0, 0, 0, 0])
# Using standard DH (d4=102):
# T_flange_zero[:3, 3] = [-820.0, -202.0, 50.0] mm (base frame)
#
# After adding the TCP tool=1 offset:
# T_tcp1_zero[:3, 3] = [-954.777, -308.334, 146.448] mm (base frame)
```

Suggestion: in your simulation framework, use FK to compute the zero-position joint angles, confirm the flange position is `[-820, -202, 50]`, and only then connect the path. If it does not match, check the DH parameter table and the rotation convention.

---

## 8. Quick Reference Card

```
┌─────────────────────────────────────────────────────┐
│ FR5 Saved Coordinates Quick Reference               │
├─────────────────────────────────────────────────────┤
│ User Frame (WObj, user_index=1):                    │
│   [649.456, 133.762, 322.778, -0.369, 0.329, -89.080] │
│                                                     │
│ TCP tool=0:  [0, 0, 0, 0, 0, 0]  (flange origin)   │
│ TCP tool=1:  [-134.777, 96.448, 106.334,            │
│               86.647, -13.136, 60.612]              │
│ TCP tool=2:  (same as tool=1)                       │
│                                                     │
│ 6D format:   [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] │
│ Rotation:    R = Rz(rz) @ Ry(ry) @ Rx(rx)          │
│              (intrinsic XYZ / extrinsic ZYX)        │
│                                                     │
│ DH:          Standard DH, 6-axis                    │
│ d4:          102mm (current; 130mm historical only) │
│ Source:      assets/calib/active_tcp_wobj.json      │
│ Acquired:    2026-05-28 from real robot             │
└─────────────────────────────────────────────────────┘
```
