# Fairino Autoprint Farm Framework

This is a multi-printer framework for a Fairino robot arm changing buildplates in a 3x3 Bambu X1C rack. It is prepared for the target process with door/outside/inside AprilTags, rack slot IDs, and reusable JSON sequences. For first testing, use `printer_03` and the legacy sequence derived from the previously tested JSON.

## First commands

```bash
python run_job.py --list-jobs
python run_job.py --list-printers
python run_job.py --list-slots
python run_job.py --job trial_full_swap_legacy --plan-only
python run_job.py --job trial_full_swap_split --printer printer_03 --drop-slot slot_01 --pick-slot slot_01 --plan-only
```

## First live test

Start with small jobs before running a full sequence:

```bash
python run_job.py --job localize_printer_outer_only --printer printer_03 --dry-run
python run_job.py --job localize_printer_outer_only --printer printer_03
```

The target full process is:

```bash
python run_job.py --job full_buildplate_change --printer printer_03 --drop-slot slot_01 --pick-slot slot_02 --plan-only
```

Do not run the target full process live until all sequence files have been calibrated and the Vision Pi outputs the tag family if `require_family` is set to true.
