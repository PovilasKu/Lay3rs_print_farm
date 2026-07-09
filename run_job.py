#!/usr/bin/env python3
"""Run a planned Fairino autoprint-farm job."""

import argparse
import json
import time

import config as cfg
from core.command_executor import execute_command
from core.motion import move_cart
from core.robot_client import connect_robot, get_actual_pose, robot_preflight
from planner.job_planner import (
    build_plan,
    print_available_jobs,
    print_available_printers,
    print_available_slots,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Fairino autoprint-farm job runner")
    parser.add_argument("--job", help="Job name from data/jobs/job_catalog.json")
    parser.add_argument("--printer", help="Printer ID from data/printers/printers.json")
    parser.add_argument("--drop-slot", help="Rack slot ID where the old plate should be dropped")
    parser.add_argument("--pick-slot", help="Rack slot ID where the clean plate should be picked")
    parser.add_argument("--vars-json", help="Optional JSON string of extra variables for ${vars.*} placeholders")
    parser.add_argument("--list-jobs", action="store_true", help="List available jobs and exit")
    parser.add_argument("--list-printers", action="store_true", help="List available printers and exit")
    parser.add_argument("--list-slots", action="store_true", help="List available rack slots and exit")
    parser.add_argument("--plan-only", action="store_true", help="Build and print/save the plan, but do not connect to the robot")
    parser.add_argument("--dry-run", action="store_true", help="Override config.DRY_RUN=True for this run")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.dry_run:
        cfg.DRY_RUN = True

    if args.list_jobs:
        print_available_jobs()
        return
    if args.list_printers:
        print_available_printers()
        return
    if args.list_slots:
        print_available_slots()
        return

    if not args.job:
        print("No job selected.")
        print("Use one of:")
        print("  python run_job.py --list-jobs")
        print("  python run_job.py --list-printers")
        print("  python run_job.py --list-slots")
        print("  python run_job.py --job trial_full_swap_legacy --printer printer_03 --plan-only")
        return

    extra_vars = {}
    if args.vars_json:
        extra_vars = json.loads(args.vars_json)
        if not isinstance(extra_vars, dict):
            raise RuntimeError("--vars-json must decode to a JSON object/dict.")

    print()
    print("Fairino autoprint-farm job runner")
    print("==================================")
    print(f"Selected job:     {args.job}")
    print(f"Selected printer: {args.printer}")
    print(f"Drop slot:        {args.drop_slot}")
    print(f"Pick slot:        {args.pick_slot}")
    print(f"Vision URL:       {cfg.VISION_URL}")
    print(f"DRY_RUN:          {cfg.DRY_RUN}")
    print()

    commands = build_plan(
        args.job,
        printer_id=args.printer,
        drop_slot_id=args.drop_slot,
        pick_slot_id=args.pick_slot,
        extra_vars=extra_vars,
        save_plan=True,
    )

    executable_commands = [cmd for cmd in commands if cmd.get("type") != "_marker"]
    print(f"Planned executable command count: {len(executable_commands)}")

    if args.plan_only:
        print("Plan-only mode: not connecting to robot.")
        return

    robot = connect_robot()

    if cfg.ROBOT_PREFLIGHT:
        robot_preflight(robot)

    if cfg.MOVE_TO_START_POSE:
        print("Moving to known start pose...")
        move_cart(robot, cfg.TEST_START_POSE)
        time.sleep(1.5)
    else:
        current_start_pose = get_actual_pose(robot)
        print("Using current robot position as start pose:")
        print([round(v, 3) for v in current_start_pose])
        time.sleep(1.0)

    total = len(executable_commands)
    command_number = 0

    for command in commands:
        if command.get("type") == "_marker":
            print()
            print("############################################################")
            print(command.get("name", "Starting sequence"))
            print("############################################################")
            continue

        command_number += 1
        execute_command(robot, command, command_number, total)

    print()
    print(f"Job '{args.job}' complete.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped by user.")
    except Exception as e:
        print()
        print(f"ERROR: {e}")
        raise
