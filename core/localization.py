"""Closed-loop AprilTag localization logic."""

import json
import time

import config as cfg
from core.helpers import add_axis_step, clamp, get_number
from core.math_utils import angle_error_deg, wrap_angle_deg
from core.robot_client import get_actual_pose
from core.motion import move_cart
from core.safety import check_safe_limits
from core.vision_client import collect_stable_tag, describe_vision_source, vision_source_policy_from_params


def correction_step(error, gain, max_step, min_step, tolerance):
    """
    Proportional correction with a minimum useful step.

    If the error is within tolerance, command 0. Otherwise clamp the step and
    force at least min_step so the robot does not send endless tiny corrections.
    """
    error = float(error)
    gain = float(gain)
    max_step = abs(float(max_step))
    min_step = abs(float(min_step))
    tolerance = abs(float(tolerance))

    if abs(error) <= tolerance:
        return 0.0

    step = clamp(gain * error, -max_step, max_step)
    if abs(step) < min_step:
        step = min_step if error > 0 else -min_step
    return step


def merged_stage_params(stage: dict, command: dict) -> dict:
    """Allow a JSON localize command to override stage tuning values."""
    out = dict(stage)

    nested = command.get(stage["name"])
    if isinstance(nested, dict):
        out.update(nested)

    for key in [
        "center_tol_mm", "distance_tol_mm", "rot_tol_deg",
        "samples", "max_iters", "gain_xy", "gain_z", "gain_pitch", "gain_yaw", "gain_roll",
        "max_xy_step_mm", "max_z_step_mm", "max_rot_step_deg",
        "min_xy_step_mm", "min_z_step_mm", "min_rot_step_deg",
    ]:
        if key in command:
            out[key] = command[key]

    return out


def get_targets(params: dict | None) -> dict:
    params = params or {}

    return {
        "target_x_mm": get_number(params, "target_x_mm", cfg.TARGET_X_MM),
        "target_y_mm": get_number(params, "target_y_mm", cfg.TARGET_Y_MM),
        "target_distance_mm": get_number(params, "target_distance_mm", cfg.TARGET_DISTANCE_MM),
        "target_pitch_deg": get_number(params, "target_pitch_deg", params.get("target_tilt_x_deg", cfg.TARGET_PITCH_DEG)),
        "target_yaw_deg": get_number(params, "target_yaw_deg", params.get("target_tilt_y_deg", cfg.TARGET_YAW_DEG)),
        "target_roll_deg": get_number(params, "target_roll_deg", cfg.TARGET_ROLL_DEG),
        "pitch_zero_offset_deg": get_number(params, "pitch_zero_offset_deg", cfg.PITCH_ZERO_OFFSET_DEG),
        "yaw_zero_offset_deg": get_number(params, "yaw_zero_offset_deg", cfg.YAW_ZERO_OFFSET_DEG),
        "roll_zero_offset_deg": get_number(params, "roll_zero_offset_deg", cfg.ROLL_ZERO_OFFSET_DEG),
    }


def get_corrected_angles(tag: dict, params: dict | None):
    targets = get_targets(params)
    pitch = float(tag["tag_pitch_deg"]) - targets["pitch_zero_offset_deg"]
    yaw = float(tag["tag_yaw_deg"]) - targets["yaw_zero_offset_deg"]
    roll = float(tag["tag_roll_deg"]) - targets["roll_zero_offset_deg"]
    return wrap_angle_deg(pitch), wrap_angle_deg(yaw), wrap_angle_deg(roll)


def build_corrected_pose(current_pose: list[float], tag: dict, params: dict, stage: dict):
    targets = get_targets(params)
    enable_rotation = bool(params.get("enable_rotation", cfg.ENABLE_ROTATION_3AXIS))

    center_tol_mm = float(stage["center_tol_mm"])
    distance_tol_mm = float(stage["distance_tol_mm"])
    rot_tol_deg = float(stage["rot_tol_deg"])

    tag_x_mm = float(tag["x_mm"])
    tag_y_mm = float(tag["y_mm"])
    tag_z_mm = float(tag["z_mm"])

    x_error_mm = tag_x_mm - targets["target_x_mm"]
    y_error_mm = tag_y_mm - targets["target_y_mm"]
    distance_error_mm = tag_z_mm - targets["target_distance_mm"]

    cam_x_step = correction_step(x_error_mm, stage["gain_xy"], stage["max_xy_step_mm"], stage["min_xy_step_mm"], center_tol_mm)
    cam_y_step = correction_step(y_error_mm, stage["gain_xy"], stage["max_xy_step_mm"], stage["min_xy_step_mm"], center_tol_mm)
    cam_z_step = correction_step(distance_error_mm, stage["gain_z"], stage["max_z_step_mm"], stage["min_z_step_mm"], distance_tol_mm)

    robot_x_step = cfg.CAMERA_X_SIGN * cam_x_step
    robot_y_step = cfg.CAMERA_Y_SIGN * cam_y_step
    robot_z_step = cfg.CAMERA_Z_SIGN * cam_z_step

    next_pose = list(current_pose)
    next_pose = add_axis_step(next_pose, cfg.CAMERA_X_TO_ROBOT_AXIS, robot_x_step)
    next_pose = add_axis_step(next_pose, cfg.CAMERA_Y_TO_ROBOT_AXIS, robot_y_step)
    next_pose = add_axis_step(next_pose, cfg.CAMERA_Z_TO_ROBOT_AXIS, robot_z_step)

    info = {
        "target_x_mm": targets["target_x_mm"],
        "target_y_mm": targets["target_y_mm"],
        "target_distance_mm": targets["target_distance_mm"],
        "tag_x_mm": tag_x_mm,
        "tag_y_mm": tag_y_mm,
        "tag_z_mm": tag_z_mm,
        "x_error_mm": x_error_mm,
        "y_error_mm": y_error_mm,
        "distance_error_mm": distance_error_mm,
        "robot_x_step": robot_x_step,
        "robot_y_step": robot_y_step,
        "robot_z_step": robot_z_step,
        "robot_pitch_step": 0.0,
        "robot_yaw_step": 0.0,
        "robot_roll_step": 0.0,
    }

    if enable_rotation:
        pitch_deg, yaw_deg, roll_deg = get_corrected_angles(tag, params)
        pitch_error = angle_error_deg(pitch_deg, targets["target_pitch_deg"])
        yaw_error = angle_error_deg(yaw_deg, targets["target_yaw_deg"])
        roll_error = angle_error_deg(roll_deg, targets["target_roll_deg"])

        pitch_step = correction_step(pitch_error, stage["gain_pitch"], stage["max_rot_step_deg"], stage["min_rot_step_deg"], rot_tol_deg)
        yaw_step = correction_step(yaw_error, stage["gain_yaw"], stage["max_rot_step_deg"], stage["min_rot_step_deg"], rot_tol_deg)
        roll_step = correction_step(roll_error, stage["gain_roll"], stage["max_rot_step_deg"], stage["min_rot_step_deg"], rot_tol_deg)

        robot_pitch_step = cfg.PITCH_SIGN * pitch_step
        robot_yaw_step = cfg.YAW_SIGN * yaw_step
        robot_roll_step = cfg.ROLL_SIGN * roll_step

        next_pose = add_axis_step(next_pose, cfg.PITCH_TO_ROBOT_ROT_AXIS, robot_pitch_step)
        next_pose = add_axis_step(next_pose, cfg.YAW_TO_ROBOT_ROT_AXIS, robot_yaw_step)
        next_pose = add_axis_step(next_pose, cfg.ROLL_TO_ROBOT_ROT_AXIS, robot_roll_step)

        info.update({
            "tag_pitch_deg": pitch_deg,
            "tag_yaw_deg": yaw_deg,
            "tag_roll_deg": roll_deg,
            "pitch_error_deg": pitch_error,
            "yaw_error_deg": yaw_error,
            "roll_error_deg": roll_error,
            "robot_pitch_step": robot_pitch_step,
            "robot_yaw_step": robot_yaw_step,
            "robot_roll_step": robot_roll_step,
        })

    check_safe_limits(next_pose)
    return next_pose, info


def is_localize_finished(tag: dict, params: dict, stage: dict) -> bool:
    targets = get_targets(params)
    enable_rotation = bool(params.get("enable_rotation", cfg.ENABLE_ROTATION_3AXIS))
    use_pixel_check = bool(params.get("use_pixel_finish_check", cfg.USE_PIXEL_FINISH_CHECK))

    center_tol_mm = float(stage["center_tol_mm"])
    distance_tol_mm = float(stage["distance_tol_mm"])
    rot_tol_deg = float(stage["rot_tol_deg"])

    x_error = float(tag["x_mm"]) - targets["target_x_mm"]
    y_error = float(tag["y_mm"]) - targets["target_y_mm"]
    z_error = float(tag["z_mm"]) - targets["target_distance_mm"]

    pose_ok = abs(x_error) <= center_tol_mm and abs(y_error) <= center_tol_mm
    distance_ok = abs(z_error) <= distance_tol_mm

    if use_pixel_check:
        target_u = get_number(params, "target_u", cfg.TARGET_U)
        target_v = get_number(params, "target_v", cfg.TARGET_V)
        pixel_ok = abs(float(tag["cx"]) - target_u) <= cfg.PIXEL_TOL and abs(float(tag["cy"]) - target_v) <= cfg.PIXEL_TOL
    else:
        pixel_ok = True

    if enable_rotation:
        pitch, yaw, roll = get_corrected_angles(tag, params)
        rotation_ok = (
            abs(angle_error_deg(pitch, targets["target_pitch_deg"])) <= rot_tol_deg and
            abs(angle_error_deg(yaw, targets["target_yaw_deg"])) <= rot_tol_deg and
            abs(angle_error_deg(roll, targets["target_roll_deg"])) <= rot_tol_deg
        )
    else:
        rotation_ok = True

    return pose_ok and distance_ok and pixel_ok and rotation_ok


def print_localize_status(tag: dict, params: dict, stage: dict, iteration: int, stage_iters: int):
    targets = get_targets(params)
    target_u = get_number(params, "target_u", cfg.TARGET_U)
    target_v = get_number(params, "target_v", cfg.TARGET_V)
    enable_rotation = bool(params.get("enable_rotation", cfg.ENABLE_ROTATION_3AXIS))

    cx = float(tag["cx"])
    cy = float(tag["cy"])
    tag_x = float(tag["x_mm"])
    tag_y = float(tag["y_mm"])
    tag_z = float(tag["z_mm"])

    print()
    print(f"Localization stage '{stage['name']}' iteration {iteration}/{stage_iters}")
    print("------------------------------------------------------------")
    print(f"Vision source:          {describe_vision_source(tag)}")
    print(f"Samples:                {tag.get('sample_count', 1)}/{tag.get('requested_sample_count', 1)}")
    print(f"Tag pixel center:       u={cx:.1f}, v={cy:.1f}")
    print(f"Pixel error:            du={cx - target_u:.1f}, dv={cy - target_v:.1f}")
    print(f"Camera-space tag pose:  x={tag_x:.2f} mm, y={tag_y:.2f} mm, z={tag_z:.2f} mm")
    print(
        f"Target camera pose:     x={targets['target_x_mm']:.2f} mm, "
        f"y={targets['target_y_mm']:.2f} mm, z={targets['target_distance_mm']:.2f} mm"
    )
    print(f"Camera pose error:      x={tag_x - targets['target_x_mm']:.2f} mm, y={tag_y - targets['target_y_mm']:.2f} mm")
    print(f"Target distance error:  {tag_z - targets['target_distance_mm']:.2f} mm")

    if enable_rotation:
        pitch, yaw, roll = get_corrected_angles(tag, params)
        print(f"Corrected tag rot:      pitch={pitch:.2f} deg, yaw={yaw:.2f} deg, roll={roll:.2f} deg")
        print(
            f"Rotation error:         pitch={angle_error_deg(pitch, targets['target_pitch_deg']):.2f}, "
            f"yaw={angle_error_deg(yaw, targets['target_yaw_deg']):.2f}, "
            f"roll={angle_error_deg(roll, targets['target_roll_deg']):.2f} deg"
        )


def execute_localize(robot, command: dict):
    name = command.get("name", "localize")
    required_tag_id = command.get("tag_id")
    enable_rotation = bool(command.get("enable_rotation", cfg.ENABLE_ROTATION_3AXIS))

    if cfg.DRY_RUN and bool(command.get("skip_vision_in_dry_run", True)):
        print(f"[DRY RUN] Would localize: {name}")
        print(f"[DRY RUN] Required tag id: {required_tag_id}")
        print(f"[DRY RUN] Rotation correction: {enable_rotation}")
        return

    targets = get_targets(command)
    localize_vel = get_number(command, "vel", command.get("localize_vel", cfg.LOCALIZE_VEL))
    localize_acc = get_number(command, "acc", command.get("localize_acc", cfg.LOCALIZE_ACC))
    localize_ovl = get_number(command, "ovl", command.get("localize_ovl", cfg.LOCALIZE_OVL))
    wait_after_move_s = get_number(command, "wait_after_move_s", cfg.WAIT_AFTER_MOVE_S)
    sample_delay_s = get_number(command, "sample_delay_s", cfg.VISION_SAMPLE_DELAY_S)

    print(f"\nStarting precise stereo localization: {name}")
    print(
        f"Target camera pose: x={targets['target_x_mm']:.2f} mm, "
        f"y={targets['target_y_mm']:.2f} mm, z={targets['target_distance_mm']:.1f} mm"
    )
    print(
        f"Target rotation: pitch={targets['target_pitch_deg']:.2f}, "
        f"yaw={targets['target_yaw_deg']:.2f}, roll={targets['target_roll_deg']:.2f} deg"
    )
    print(f"Required tag id: {required_tag_id}")
    print(f"Rotation correction: {enable_rotation}")
    print(f"Vision source policy: {vision_source_policy_from_params(command)}")

    last_tag = None

    for stage_index, base_stage in enumerate(cfg.LOCALIZE_STAGES):
        stage = merged_stage_params(base_stage, command)
        stage_iters = int(stage["max_iters"])
        stage_samples = int(stage["samples"])

        print()
        print(f"===== LOCALIZATION STAGE: {stage['name'].upper()} =====")
        print(f"tolerances: center={stage['center_tol_mm']} mm, z={stage['distance_tol_mm']} mm, rot={stage['rot_tol_deg']} deg")

        for iteration in range(1, stage_iters + 1):
            tag = collect_stable_tag(
                required_tag_id=required_tag_id,
                enable_rotation=enable_rotation,
                params=command,
                samples=stage_samples,
                delay_s=sample_delay_s,
            )
            last_tag = tag

            print_localize_status(tag, command, stage, iteration, stage_iters)

            if is_localize_finished(tag, command, stage):
                print(f"Localization stage '{stage['name']}' done.")
                break

            current_pose = get_actual_pose(robot)
            next_pose, info = build_corrected_pose(current_pose, tag, command, stage)

            print()
            print("Correction:")
            print(f"  X error {info['x_error_mm']:.3f} mm -> robot axis {cfg.CAMERA_X_TO_ROBOT_AXIS}: {info['robot_x_step']:.3f} mm")
            print(f"  Y error {info['y_error_mm']:.3f} mm -> robot axis {cfg.CAMERA_Y_TO_ROBOT_AXIS}: {info['robot_y_step']:.3f} mm")
            print(f"  Z error {info['distance_error_mm']:.3f} mm -> robot axis {cfg.CAMERA_Z_TO_ROBOT_AXIS}: {info['robot_z_step']:.3f} mm")
            if enable_rotation:
                print(f"  pitch -> robot axis {cfg.PITCH_TO_ROBOT_ROT_AXIS}: {info['robot_pitch_step']:.3f} deg")
                print(f"  yaw   -> robot axis {cfg.YAW_TO_ROBOT_ROT_AXIS}: {info['robot_yaw_step']:.3f} deg")
                print(f"  roll  -> robot axis {cfg.ROLL_TO_ROBOT_ROT_AXIS}: {info['robot_roll_step']:.3f} deg")

            print(f"Current pose: {[round(v, 3) for v in current_pose]}")
            print(f"Next pose:    {[round(v, 3) for v in next_pose]}")

            if next_pose == current_pose:
                print("No correction needed. Localization done.")
                return

            move_cart(robot, next_pose, vel=localize_vel, acc=localize_acc, ovl=localize_ovl)
            time.sleep(wait_after_move_s)
        else:
            print(f"Localization stage '{stage['name']}' reached max iterations without meeting tolerance.")
            continue

        if stage_index == len(cfg.LOCALIZE_STAGES) - 1:
            print("Precise localization done.")
            return

    if last_tag is not None:
        print("Last localization measurement:")
        targets = get_targets(command)
        print(json.dumps({
            "x_mm": last_tag.get("x_mm"),
            "target_x_mm": targets["target_x_mm"],
            "y_mm": last_tag.get("y_mm"),
            "target_y_mm": targets["target_y_mm"],
            "z_mm": last_tag.get("z_mm"),
            "target_distance_mm": targets["target_distance_mm"],
            "tag_pitch_deg": last_tag.get("tag_pitch_deg"),
            "tag_yaw_deg": last_tag.get("tag_yaw_deg"),
            "tag_roll_deg": last_tag.get("tag_roll_deg"),
            "sample_count": last_tag.get("sample_count"),
        }, indent=2))

    raise RuntimeError("Precise localization failed to meet final tolerance.")
