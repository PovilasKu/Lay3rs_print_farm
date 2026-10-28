import asyncio, sys, json
from nicegui import app, ui
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config import VISION_PI_IP, VISION_PORT
from fastapi.responses import StreamingResponse
import httpx

VISION_URL = f"http://{VISION_PI_IP}:{VISION_PORT}"
RUN_JOB_SCRIPT = Path("/home/main/Lay3rs_print_farm/run_job.py")
PRINTERS = {"printer_03": "X1C-4", "printer_04": "X1C-2", "printer_05": "X1C-1"}
N_SLOTS = 16
SLOT_POLL_S = 2.0
JOB_TIMEOUT_S = 600
SLOT_FILE = Path(__file__).with_name("slots.json")

MAIN_JOBS = {
    "Change plate": ("full_buildplate_change", True, True, "sym_r_stack"),
    "Home": ("go_home", False, False, "sym_r_home"),
    "Transfer": ("transfer_buildplate", True, True, "sym_r_swap_vert"),
}

DEBUG_JOBS = {
    "Open door": ("open_door_with_verification", False, False),
    "Remove plate (door open)": ("remove_buildplate_only", False, False),
    "Open door": ("open_door_with_verification", False, False),
    "Open and take plate": ("open_door_and_take_out_buildplate", False, False),
    "Place plate (printer)": ("localize_and_place_buildplate", False, False),
    "Drop plate": ("rack_drop_only", False, True),
    "Pickup plate (rack)": ("rack_pickup_only", True, False),
    "Close door": ("close_door", False, False),
    "Localize door": ("localize_printer_door_only", False, False),
}

state = {"busy": False, "proc": None, "editing": False, "printer": next(iter(PRINTERS))}
# True = occupied, False = empty
slots = {"pickup": [False] * N_SLOTS, "dropoff": [False] * N_SLOTS}
selected = {"pickup": None, "dropoff": None}


@app.get("/vision/stream")
async def vision_stream():
    async def gen():
        try:
            async with httpx.AsyncClient(timeout=None) as client:
                async with client.stream("GET", f"{VISION_URL}/stream") as r:
                    async for chunk in r.aiter_raw():
                        yield chunk
        except httpx.HTTPError:
            return

    return StreamingResponse(
        gen(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


def load_slots():
    try:
        data = json.loads(SLOT_FILE.read_text())
        for kind in ("pickup", "dropoff"):
            if len(data[kind]) == N_SLOTS:
                slots[kind] = [bool(x) for x in data[kind]]
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"Could not read {SLOT_FILE}: {e}")


def save_slots():
    try:
        SLOT_FILE.write_text(json.dumps(slots))
    except Exception as e:
        ui.notify(f"Could not save slots: {e}", type="negative")


load_slots()


def fetch_slot_status() -> dict:
    return slots


GCODE_DIR = Path("/home/main/Lay3rs_print_farm/gcode")


def list_gcode_files() -> list[Path]:
    return sorted(
        GCODE_DIR.glob("*.gcode"), key=lambda p: ("start" not in p.name.lower(), p.name)
    )


async def copy_gcode(path: Path):
    try:
        text = path.read_text()  # read on click, so edits to the file are picked up
    except Exception as e:  # noqa: BLE001
        ui.notify(f"Could not read {path.name}: {e}", type="negative")
        return
    # navigator.clipboard only works on https/localhost, and you open the page over
    # plain http on a LAN IP, so fall back to a hidden textarea + execCommand.
    ui.run_javascript(f"""
        const t = {json.dumps(text)};
        if (navigator.clipboard && window.isSecureContext) {{
            navigator.clipboard.writeText(t);
        }} else {{
            const a = document.createElement('textarea');
            a.value = t; a.style.position = 'fixed'; a.style.opacity = '0';
            document.body.appendChild(a); a.select();
            document.execCommand('copy'); a.remove();
        }}
    """)
    ui.notify(f"Copied {path.name}", type="positive")


ui.colors(primary="#20BCEF", negative="#ef4444", positive="#3fb68b", warning="#f2a93b")
ui.dark_mode(True)
ui.add_css("""
:root {
    --bg: #0a0c10; --surface: #0f1218; --raised: #151a22;
    --line: #1c222c; --line-hi: #2a3240;
    --text: #e7eaee; --muted: #6b7686;
    --accent: #20BCEF; --live: #20BCEF;
}
body { background: var(--bg); color: var(--text);
       font-family: Inter, ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif;
       -webkit-font-smoothing: antialiased; }
.nicegui-content { padding: 0; }
.mono { font-family: ui-monospace, "SF Mono", Menlo, Consolas, monospace; }
.panel { background: var(--surface); border: 1px solid var(--line); border-radius: 8px; }
.eyebrow { font-size: 11px; letter-spacing: .09em; text-transform: uppercase;
           color: var(--muted); font-weight: 500; }
.q-btn__content { flex-wrap: nowrap; }

/* printers */
.q-btn.printer { height: 56px; padding: 0 14px; border: 1px solid var(--line);
                 border-radius: 6px; background: transparent !important; color: var(--muted); }
.q-btn.printer:hover { background: var(--raised) !important; color: var(--text); }
.q-btn.printer.sel { background: var(--raised) !important; border-color: var(--line-hi);
                     color: var(--text); box-shadow: inset 2px 0 0 var(--accent) !important; }

.q-btn.copybtn { color: var(--muted) !important; }
.q-btn.copybtn:hover { color: var(--accent) !important; background: var(--raised) !important; }

/* slots */
.q-btn.slot { width: 100%; height: 30px; padding: 0 10px; border: 1px solid var(--line);
              border-radius: 5px; background: transparent !important; color: var(--muted);
              font-family: ui-monospace, "SF Mono", Menlo, Consolas, monospace; font-size: 12px; }
.q-btn.slot .q-btn__content { justify-content: flex-start; }
.q-btn.slot:hover { border-color: var(--line-hi); color: var(--text); }
.slot .dot, .legend-dot { width: 6px; height: 6px; border-radius: 50%;
                          border: 1px solid var(--muted); margin-right: 8px; flex: none; }
.q-btn.slot.occ { color: var(--text); background: rgba(91,141,239,.08) !important;
                  border-color: rgba(91,141,239,.35); }
.slot.occ .dot, .legend-dot.occ { background: var(--accent); border-color: var(--accent); }
.q-btn.slot.sel { border-color: #fff !important; color: #fff; }
.legend-sel { width: 10px; height: 10px; border: 1px solid #fff; border-radius: 3px; }
.q-btn.editbtn { border: 1px solid var(--line-hi); border-radius: 5px; padding: 0 12px;
                color: var(--text); }
.q-btn.editbtn.on { border-color: var(--live); color: var(--live); }
.editing .q-btn.slot { border-style: dashed; } 
.q-btn.slot.blocked { opacity: .3; cursor: not-allowed; }
.q-btn.slot.blocked:hover { border-color: var(--line); color: var(--muted); }
/* jobs */
.q-btn.job { height: 40px; padding: 0 18px; border-radius: 6px; font-weight: 500;
             letter-spacing: 0; color: var(--text); border: 1px solid var(--line-hi);
             background: var(--raised) !important; }
.q-btn.job:hover { border-color: var(--accent); }
.q-btn.stop { color: #ffffff !important; background: rgba(239,68,68,.8) !important; width: 350px; height: 40px;}
.q-btn.jobtile { width: 108px; height: 108px; border: 1px solid var(--line-hi);
                 border-radius: 10px; background: var(--raised) !important;
                 color: var(--text); }
.q-btn.jobtile:hover { border-color: var(--accent); }
.q-btn.jobtile .q-icon { color: var(--accent); }
.q-btn.morebtn { color: var(--muted); }
.q-btn.morebtn:hover { color: var(--text); background: var(--raised) !important; }
.q-menu { background: var(--raised) !important; border: 1px solid var(--line-hi);
          border-radius: 8px; }

/* status + badges */
.sdot { width: 7px; height: 7px; border-radius: 50%; }
.sdot.idle { background: #3fb68b; }
.sdot.run { background: var(--accent); }
.badge { font-size: 11px; letter-spacing: .06em; padding: 3px 8px; border-radius: 4px;
         border: 1px solid var(--line-hi); color: var(--muted); }
.badge.live { color: rgba(242,169,59,1); border-color: rgba(242,169,59,.5);
              background: rgba(242,169,59,.08); }

/* log */
.log-box { background: var(--bg) !important; border: 1px solid var(--line);
           border-radius: 6px; font-family: ui-monospace, "SF Mono", Menlo, Consolas, monospace;
           font-size: 12px; color: #9aa4b2; }
.q-toggle__label { font-size: 13px; color: var(--text); }
""")


def mark(el, cls: str, on: bool):
    el.classes(add=cls) if on else el.classes(remove=cls)


# Printers
def refresh_printers():
    for name, btn in printer_buttons.items():
        mark(btn, "sel", state["printer"] == name)


def on_printer_click(name: str):
    state["printer"] = name
    refresh_printers()


# Slots
def refresh_slots():
    blocked = {} if state["editing"] else blocked_dropoffs()
    if selected["dropoff"] in blocked:
        selected["dropoff"] = None

    for kind, buttons in slot_buttons.items():
        for i, btn in enumerate(buttons):
            mark(btn, "occ", slots[kind][i])
            mark(btn, "sel", selected[kind] == i)
            mark(btn, "blocked", kind == "dropoff" and i in blocked)

    h = get_print_height_cm(selected["dropoff"])
    height_msg.set_visibility(h is not None)
    if h is not None:
        height_msg.set_text(f"Make sure your print is no taller than {h:g} cm")

    for kind, lbl in sel_labels.items():
        i = selected[kind]
        if i is None:
            lbl.set_text(f"{kind.capitalize():<8} none")
        else:
            lbl.set_text(
                f"{kind.capitalize():<8} {i + 1:02d} · "
                f"{'occupied' if slots[kind][i] else 'empty'}"
            )


def on_slot_click(kind: str, i: int):
    if kind == "dropoff" and not state["editing"] and i in blocked_dropoffs():
        ui.notify(f"Dropoff {i + 1:02d} is blocked by pickup {i:02d}", type="warning")
        return
    if state["editing"]:  # edit mode: toggle occupied / empty
        slots[kind][i] = not slots[kind][i]
        save_slots()
    else:  # normal mode: select for jobs
        selected[kind] = None if selected[kind] == i else i
    refresh_slots()


def toggle_edit():
    state["editing"] = not state["editing"]
    editing = state["editing"]
    if editing:
        selected["pickup"] = selected["dropoff"] = None
    edit_btn.set_text("Done" if editing else "Edit")
    mark(edit_btn, "on", editing)
    mark(slots_row, "editing", editing)
    edit_hint.set_visibility(editing)
    refresh_slots()


def clear_selection():
    selected["pickup"] = selected["dropoff"] = None
    refresh_slots()


async def poll_slots():
    if state["editing"]:
        return
    try:
        data = await asyncio.to_thread(fetch_slot_status)
        for kind in ("pickup", "dropoff"):
            if len(data[kind]) == N_SLOTS:
                slots[kind] = [bool(x) for x in data[kind]]
        refresh_slots()
    except Exception as e:
        log.push(f"Slot status error: {e}")


def validate_slots(needs_slot: bool, slot_type: str) -> str | None:
    p = selected[slot_type]
    if p is None:
        return f"Select a {slot_type} slot"
    if slot_type == "pickup" and not slots["pickup"][p]:
        return f"Pickup slot {p + 1} is empty"
    if slot_type == "dropoff" and slots["dropoff"][p]:
        return f"Dropoff slot {p + 1} is already occupied"
    return None


def refresh_mode(_=None):
    live = not dry.value
    mode_badge.set_text("LIVE · ROBOT WILL MOVE" if live else "TEST MODE")
    mark(mode_badge, "live", live)


def set_busy(value: bool):
    state["busy"] = value
    for b in job_buttons:
        b.set_enabled(not value)
    for btns in slot_buttons.values():
        for b in btns:
            b.set_enabled(not value)
    for b in printer_buttons.values():
        b.set_enabled(not value)
    clear_btn.set_enabled(not value)
    dry.set_enabled(not value)
    stop_btn.set_visibility(value)
    status.set_text("RUNNING" if value else "IDLE")
    status_dot.classes(replace="sdot run animate-pulse" if value else "sdot idle")


async def run(job: str, needs_drop_slot: bool, needs_pick_slot: bool, printer_name: str, dry_run: bool):
    transfer = None
    if state["busy"]:
        ui.notify("A job is already running", type="warning")
        return
    if not RUN_JOB_SCRIPT.exists():
        ui.notify(f"Script not found: {RUN_JOB_SCRIPT}", type="negative")
        log.push(f"ERROR: {RUN_JOB_SCRIPT} does not exist")
        return

    cmd = [sys.executable, str(RUN_JOB_SCRIPT), "--job", job, "--printer", printer_name]
    desc = f"{job} on {printer_name}"
    if needs_drop_slot or needs_pick_slot:
        err = validate_slots(needs_drop_slot, "dropoff") or validate_slots(needs_pick_slot, "pickup")
        if err:
            ui.notify(err, type="warning")
            return
        p, d = selected["pickup"] + 1, selected["dropoff"] + 1
        transfer = (selected["pickup"], selected["dropoff"])
        cmd += ["--pick-slot", f"slot_{p:02d}p", "--drop-slot", f"slot_{d:02d}d"]
        desc += f", pickup {p} → dropoff {d}"
    if dry_run:
        cmd.append("--dry-run")

    set_busy(True)
    log.push(f"> {desc} (dry run: {dry_run})")
    code = None
    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        state["proc"] = proc

        async def stream():
            async for line in proc.stdout:
                log.push(line.decode(errors="replace").rstrip())
            await proc.wait()

        await asyncio.wait_for(stream(), timeout=JOB_TIMEOUT_S)
        code = proc.returncode
    except asyncio.TimeoutError:
        log.push(f"Timed out after {JOB_TIMEOUT_S}s, stopping job")
        await stop_job()
        code = -1
    except Exception as e:
        log.push(f"ERROR: {e}")
        code = -1
    finally:
        state["proc"] = None
        set_busy(False)

    if code == 0:
        log.push("Finished successfully")
        ui.notify("Job finished", type="positive")
        if transfer and not dry_run:
            pi, di = transfer
            slots["pickup"][pi] = False
            slots["dropoff"][di] = True
            save_slots()
            clear_selection()
    else:
        log.push(f"Finished with errors (exit code {code})")
        ui.notify(f"Job failed (exit code {code})", type="negative")
    await poll_slots()


def get_print_height_cm(dropoff: int | None) -> int | None:
    """Returns the max print height in cm, or None to hide the message."""
    if dropoff == None:
        return None
    idx = dropoff - 1
    while idx >= 0:
        if slots["pickup"][idx] == True:
            return (dropoff - idx - 1) * 7 if ((dropoff - idx - 1) * 7 <= 25) else None
        if slots["dropoff"][idx] == True or idx == 0:
            return 4 + (dropoff - idx) * 7 if (4 + (dropoff - idx) * 7 <= 25) else None
        idx -= 1
    return 4


def blocked_dropoffs() -> dict[int, int]:
    """dropoff index -> index of the pickup slot blocking it.
    A full pickup slot N blocks dropoff slot N+1 (not the other way round)."""
    return {
        i + 1: i for i, occ in enumerate(slots["pickup"]) if occ and i + 1 < N_SLOTS
    }


async def stop_job():
    proc = state["proc"]
    if proc and proc.returncode is None:
        proc.terminate()
        try:
            await asyncio.wait_for(proc.wait(), timeout=5)
        except asyncio.TimeoutError:
            proc.kill()
        log.push("Job stopped")


slot_buttons = {"pickup": [], "dropoff": []}
printer_buttons = {}
sel_labels = {}
job_buttons = []

with ui.column().classes("w-full min-h-screen items-center justify-center gap-8 p-6"):
    with ui.row().classes("w-full justify-center items-start gap-5"):
        # Printer panel
        with ui.column().classes("w-full max-w-[13rem] gap-5"):
            with ui.column().classes("panel w-full max-w-[13rem] p-5 gap-3"):
                ui.label("Printers").classes("eyebrow")
                for name, pos in PRINTERS.items():
                    with (
                        ui.button(on_click=lambda n=name: on_printer_click(n))
                        .props("flat no-caps")
                        .classes("printer w-full") as pb
                    ):
                        with ui.row().classes(
                            "w-full items-center justify-between no-wrap"
                        ):
                            ui.label(pos).classes("text-sm font-medium")
                    printer_buttons[name] = pb
            with ui.column().classes("panel w-full p-5 gap-3"):
                ui.label("G-code").classes("eyebrow")
                files = list_gcode_files()
                if not files:
                    ui.label("No .gcode files found").classes("text-xs").style(
                        "color:var(--muted)"
                    )
                for f in files:
                    with ui.row().classes(
                        "w-full items-center justify-between no-wrap gap-2"
                    ):
                        ui.label(f.stem).classes(
                            "text-sm truncate flex-1 min-w-0"
                        ).tooltip(f.name)
                        ui.button(
                            icon="content_copy", on_click=lambda p=f: copy_gcode(p)
                        ).props("flat dense round size=sm").classes("copybtn")

        # Center panel
        with ui.column().classes("w-full max-w-xl gap-5"):
            with ui.column().classes("panel w-full p-6 gap-5 items-center"):
                with ui.row().classes("w-full items-center justify-between"):
                    ui.label("Controls").classes("eyebrow")
                    with ui.row().classes("items-center gap-2"):
                        status_dot = ui.element("div").classes("sdot idle")
                        status = ui.label("IDLE").classes("eyebrow mono")
                    mode_badge = ui.label("TEST MODE").classes("badge mono")

                with ui.row().classes("w-60 items-center justify-between"):
                    dry = ui.switch("Test mode", value=True, on_change=refresh_mode)
                    with (
                        ui.button(icon="more_horiz")
                        .props("flat round dense")
                        .classes("morebtn") as more_btn
                    ):
                        with ui.menu():
                            for label, (job, needs_drop_slot, needs_pick_slot) in DEBUG_JOBS.items():
                                ui.menu_item(
                                    label,
                                    on_click=lambda j=job, n_d=needs_drop_slot, n_p=needs_pick_slot: run(
                                        j, n_d, n_p, state["printer"], dry.value
                                    ),
                                )

                with ui.row().classes("gap-4 justify-center"):
                    for label, (job, needs_drop_slot, needs_pick_slot, icon) in MAIN_JOBS.items():
                        with (
                            ui.button(
                                on_click=lambda j=job, n_d=needs_drop_slot, n_p=needs_pick_slot: run(
                                    j, n_d, n_p, state["printer"], dry.value
                                ),
                            )
                            .props("flat no-caps")
                            .classes("jobtile") as btn
                        ):
                            with ui.column().classes("items-center gap-2"):
                                ui.icon(icon).classes("text-4xl")
                                ui.label(label).classes("text-xs font-medium")
                        job_buttons.append(btn)

                job_buttons.append(more_btn)

                stop_btn = (
                    ui.button("Stop", on_click=stop_job, color="#ffffff")
                    .props("flat no-caps")
                    .classes("stop")
                )
                stop_btn.set_visibility(False)

            with ui.column().classes("panel w-full p-4 gap-3"):
                ui.label("Vision").classes("eyebrow")
                ui.element("img").props("src=/vision/stream").classes("w-full").style(
                    "border-radius:6px; background:var(--bg); min-height:200px"
                )

            with ui.column().classes("panel w-full p-4 gap-3"):
                with ui.row().classes("w-full items-center justify-between"):
                    ui.label("Output").classes("eyebrow")
                    ui.button("Clear", on_click=lambda: log.clear()).props(
                        "flat dense no-caps size=sm color=grey-6"
                    )
                log = ui.log(max_lines=1000).classes("log-box w-full h-32")

        # Slot panel
        with ui.column().classes("panel w-full max-w-xs p-6 gap-5    items-center"):
            with (
                ui.row()
                .classes("gap-4 text-xs items-center")
                .style("color:var(--muted)")
            ):
                ui.label("Slots").classes("eyebrow self-start")
                with ui.row().classes("items-center gap-0"):
                    ui.element("div").classes("legend-dot occ")
                    ui.label("Occupied")
                with ui.row().classes("items-center gap-0"):
                    ui.element("div").classes("legend-dot")
                    ui.label("Empty")
                with ui.row().classes("items-center gap-2"):
                    ui.element("div").classes("legend-sel")
                    ui.label("Selected")

            height_msg = (
                ui.label("")
                .classes("w-full text-xs text-center p-2 rounded")
                .style(
                    "color:rgba(242,169,59,.75); background:rgba(242,169,59,.08); "
                    "border:1px solid rgba(242,169,59,.35)"
                )
            )

            with ui.row().classes(
                "w-full justify-center items-start gap-5"
            ) as slots_row:
                for kind in ("pickup", "dropoff"):
                    with ui.column().classes("items-stretch gap-1 w-24"):
                        ui.label(kind.capitalize()).classes("eyebrow mb-1")
                        for i in range(N_SLOTS):
                            with (
                                ui.button(
                                    on_click=lambda k=kind, n=i: on_slot_click(k, n),
                                )
                                .props("flat dense no-caps")
                                .classes("slot") as b
                            ):
                                ui.element("div").classes("dot")
                                ui.label(f"{i + 1:02d}")
                            slot_buttons[kind].append(b)
            edit_hint = (
                ui.label("Click a slot to mark it occupied / empty")
                .classes("text-xs")
                .style("color:var(--live)")
            )
            edit_hint.set_visibility(False)

            with ui.row().classes("w-full items-center justify-between"):
                clear_btn = ui.button(
                    "Clear selection", on_click=clear_selection
                ).props("flat dense no-caps size=sm color=grey-6")
                edit_btn = (
                    ui.button("Edit", on_click=toggle_edit)
                    .props("flat dense no-caps size=sm")
                    .classes("editbtn")
                )


refresh_printers()
refresh_slots()
refresh_mode()
ui.timer(SLOT_POLL_S, poll_slots)
app.on_shutdown(stop_job)

ui.run(
    host="0.0.0.0",
    port=8080,
    title="Lay3rs Print Farm",
    dark=True,
    show=False,
    favicon="https://www.lay3rs.nl/wp-content/uploads/2020/10/beeldmerk-lay3rs-donker.png",
)
