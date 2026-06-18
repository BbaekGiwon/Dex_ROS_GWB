#!/usr/bin/env python3
"""Query FR3 forward kinematics from 7 joint values.

This script reuses the FR3 kinematic parameters stored in this repository:
`franka_kistar_description/robots/fr3/kinematics.yaml`.

By default it returns the pose of the tool origin (joint8 in the YAML), expressed
in the base frame `fr3_link0`, as:
  x y z qx qy qz qw
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


JOINT_NAMES = [f"fr3_joint{i}" for i in range(1, 8)]


def _repo_root_from_here() -> Path:
    return Path(__file__).resolve().parents[1]


def _default_kinematics_path() -> Path:
    return (
        _repo_root_from_here()
        / "isaac-ros"
        / "kistar_ws"
        / "src"
        / "franka_kistar_description"
        / "robots"
        / "fr3"
        / "kinematics.yaml"
    )


def _default_joint_limits_path() -> Path:
    return (
        _repo_root_from_here()
        / "isaac-ros"
        / "kistar_ws"
        / "src"
        / "franka_kistar_description"
        / "robots"
        / "fr3"
        / "joint_limits.yaml"
    )


def _matmul4(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> List[List[float]]:
    return [
        [
            sum(a[row][k] * b[k][col] for k in range(4))
            for col in range(4)
        ]
        for row in range(4)
    ]


def _rpy_to_rot(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = [
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ]
    ry = [
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ]
    rz = [
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ]
    return _matmul3(_matmul3(rz, ry), rx)


def _matmul3(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> List[List[float]]:
    return [
        [
            sum(a[row][k] * b[k][col] for k in range(3))
            for col in range(3)
        ]
        for row in range(3)
    ]


def _rot_z(theta: float) -> List[List[float]]:
    c, s = math.cos(theta), math.sin(theta)
    return [
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _transform(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> List[List[float]]:
    r = _rpy_to_rot(roll, pitch, yaw)
    return [
        [r[0][0], r[0][1], r[0][2], x],
        [r[1][0], r[1][1], r[1][2], y],
        [r[2][0], r[2][1], r[2][2], z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_transform(rotation: Sequence[Sequence[float]]) -> List[List[float]]:
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], 0.0],
        [rotation[1][0], rotation[1][1], rotation[1][2], 0.0],
        [rotation[2][0], rotation[2][1], rotation[2][2], 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _quaternion_from_rotation(r: Sequence[Sequence[float]]) -> Tuple[float, float, float, float]:
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def _parse_scalar(text: str):
    lowered = text.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    try:
        if any(ch in text for ch in (".", "e", "E")):
            return float(text)
        return int(text)
    except ValueError:
        return text.strip('"').strip("'")


def _load_yaml(path: Path) -> Dict:
    root: Dict = {}
    stack: List[Tuple[int, Dict]] = [(-1, root)]

    with path.open("r", encoding="utf-8") as stream:
        for raw_line in stream:
            line = raw_line.split("#", 1)[0].rstrip()
            if not line.strip():
                continue

            indent = len(line) - len(line.lstrip(" "))
            stripped = line.strip()
            if ":" not in stripped:
                continue

            key, remainder = stripped.split(":", 1)
            remainder = remainder.strip()

            while indent <= stack[-1][0]:
                stack.pop()

            current = stack[-1][1]
            if remainder == "":
                current[key] = {}
                stack.append((indent, current[key]))
            else:
                current[key] = _parse_scalar(remainder)

    return root


def _load_joint_limits(path: Path) -> Dict[str, Tuple[float, float]]:
    data = _load_yaml(path)
    return {
        f"fr3_joint{i}": (
            data[f"joint{i}"]["limit"]["lower"],
            data[f"joint{i}"]["limit"]["upper"],
        )
        for i in range(1, 8)
    }


def _validate_joint_values(joints: Sequence[float], limits: Dict[str, Tuple[float, float]]) -> None:
    for name, value in zip(JOINT_NAMES, joints):
        lower, upper = limits[name]
        if value < lower or value > upper:
            raise ValueError(
                f"{name}={value:.6f} is outside limits [{lower:.6f}, {upper:.6f}]"
            )


def forward_kinematics(
    joints: Sequence[float],
    kinematics_path: Path,
    include_tool: bool = True,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    if len(joints) != 7:
        raise ValueError(f"Expected 7 joint values, got {len(joints)}")

    kinematics = _load_yaml(kinematics_path)
    transform = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    for idx, q in enumerate(joints, start=1):
        joint_key = f"joint{idx}"
        kin = kinematics[joint_key]["kinematic"]
        fixed_tf = _transform(
            kin["x"], kin["y"], kin["z"], kin["roll"], kin["pitch"], kin["yaw"]
        )
        transform = _matmul4(transform, fixed_tf)
        transform = _matmul4(transform, _rotation_transform(_rot_z(q)))

    if include_tool:
        tool_kin = kinematics["joint8"]["kinematic"]
        transform = _matmul4(
            transform,
            _transform(
                tool_kin["x"],
                tool_kin["y"],
                tool_kin["z"],
                tool_kin["roll"],
                tool_kin["pitch"],
                tool_kin["yaw"],
            ),
        )

    position = (transform[0][3], transform[1][3], transform[2][3])
    rotation = [row[:3] for row in transform[:3]]
    quaternion = _quaternion_from_rotation(rotation)
    return position, quaternion


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compute FR3 forward kinematics from 7 joint values."
    )
    parser.add_argument(
        "joints",
        nargs=7,
        type=float,
        metavar="q",
        help="Seven FR3 joint values in radians: q1 q2 q3 q4 q5 q6 q7",
    )
    parser.add_argument(
        "--link8",
        action="store_true",
        help="Return fr3_link8 pose instead of the default tool-origin pose.",
    )
    parser.add_argument(
        "--kinematics",
        type=Path,
        default=_default_kinematics_path(),
        help="Path to FR3 kinematics.yaml",
    )
    parser.add_argument(
        "--joint-limits",
        type=Path,
        default=_default_joint_limits_path(),
        help="Path to FR3 joint_limits.yaml",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print the result as a JSON-like dictionary string.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    limits = _load_joint_limits(args.joint_limits)
    _validate_joint_values(args.joints, limits)

    position, quaternion = forward_kinematics(
        joints=args.joints,
        kinematics_path=args.kinematics,
        include_tool=not args.link8,
    )

    if args.json:
        print(
            "{"
            f'"frame":"fr3_link0",'
            f'"tip":"{"fr3_link8" if args.link8 else "tool_origin"}",'
            f'"position":{{"x":{position[0]:.9f},"y":{position[1]:.9f},"z":{position[2]:.9f}}},'
            f'"quaternion":{{"x":{quaternion[0]:.9f},"y":{quaternion[1]:.9f},"z":{quaternion[2]:.9f},"w":{quaternion[3]:.9f}}}'
            "}"
        )
        return

    print(
        f"{position[0]:.9f} {position[1]:.9f} {position[2]:.9f} "
        f"{quaternion[0]:.9f} {quaternion[1]:.9f} {quaternion[2]:.9f} {quaternion[3]:.9f}"
    )


if __name__ == "__main__":
    main()
