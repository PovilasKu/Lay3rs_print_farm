import asyncio, sys
from nicegui import app, ui
from pathlib import Path

RUN_JOB_SCRIPT = Path("/home/main/Lay3rs_print_farm/run_job.py")
PRINTERS = ["printer_03", "printer_04", "printer_05"]
JOBS = {
    "Home": "go_home",
    "Localize (door)": "localize_printer_door_only",
}
JOB_TIMEOUT_S = 600

state = {"busy": False, "proc": None}

ui.colors(primary="#3b82f6", secondary="#1e293b", accent="#60a5fa",
          positive="#22c55e", negative="#ef4444", warning="#f59e0b")
ui.dark_mode(True)
ui.add_css("""
    body { background: #0f172a; }
    .nicegui-content { padding: 0; }
    .panel { background: #1e293b; border: 1px solid #334155; border-radius: 14px; }
    .q-field--outlined .q-field__control { background: #0f172a; }
    .log-box { background: #0b1220 !important; border: 1px solid #334155;
               border-radius: 10px; font-family: ui-monospace, Menlo, monospace;
               font-size: 13px; color: #cbd5e1; }
""")

def set_busy(value: bool):
    state["busy"] = value
    for b in job_buttons:
        b.set_enabled(not value)
    printer.set_enabled(not value)
    dry.set_enabled(not value)
    stop_btn.set_visibility(value)
    status.set_text("Running…" if value else "Idle")
    status_dot.classes(
        replace="bg-amber-400 animate-pulse" if value else "bg-green-500")

async def run(job: str, printer_name: str, dry_run: bool):
    if state["busy"]:
        ui.notify("A job is already running", type="warning")
        return
    if not RUN_JOB_SCRIPT.exists():
        ui.notify(f"Script not found: {RUN_JOB_SCRIPT}", type="negative")
        log.push(f"ERROR: {RUN_JOB_SCRIPT} does not exist")
        return

    cmd = [sys.executable, str(RUN_JOB_SCRIPT), "--job", job, "--printer", printer_name]
    if dry_run:
        cmd.append("--dry-run")

    set_busy(True)
    log.push(f"> {job} on {printer_name} (dry run: {dry_run})")
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
    except Exception as e:  # noqa: BLE001
        log.push(f"ERROR: {e}")
        code = -1
    finally:
        state["proc"] = None
        set_busy(False)

    if code == 0:
        log.push("Finished successfully")
        ui.notify("Job finished", type="positive")
    else:
        log.push(f"Finished with errors (exit code {code})")
        ui.notify(f"Job failed (exit code {code})", type="negative")


async def stop_job():
    proc = state["proc"]
    if proc and proc.returncode is None:
        proc.terminate()
        try:
            await asyncio.wait_for(proc.wait(), timeout=5)
        except asyncio.TimeoutError:
            proc.kill()
        log.push("Job stopped")

with ui.column().classes("w-full min-h-screen items-center justify-center gap-6 p-6"):
    ui.label("Print Farm Control").classes("text-4xl font-bold text-white text-center")
    with ui.column().classes("panel w-full max-w-xl p-6 gap-5 items-center"):
        with ui.row().classes("items-center gap-2"):
            status_dot = ui.element("div").classes("w-3 h-3 rounded-full bg-green-500")
            status = ui.label("Idle").classes("text-sm text-slate-300")

        printer = ui.select(PRINTERS, value=PRINTERS[0], label="Printer") \
            .props("outlined dark").classes("w-full")

        dry = ui.switch("Test mode (robot won't move)", value=True).props("color=amber")

        job_buttons = []
        with ui.row().classes("gap-3 justify-center"):
            for label, job in JOBS.items():
                btn = ui.button(
                    label,
                    on_click=lambda j=job: run(j, printer.value, dry.value),
                ).props("size=lg unelevated no-caps")
                job_buttons.append(btn)

        stop_btn = ui.button("Stop job", on_click=stop_job, color="negative") \
            .props("outline no-caps")
        stop_btn.set_visibility(False)

    with ui.column().classes("panel w-full max-w-xl p-4 gap-2"):
        with ui.row().classes("w-full items-center justify-between"):
            ui.label("Output").classes("text-sm text-slate-400")
            ui.button("Clear", on_click=lambda: log.clear()) \
                .props("flat dense no-caps size=sm color=grey-5")
        log = ui.log(max_lines=1000).classes("log-box w-full h-72")

app.on_shutdown(stop_job)

ui.run(host="0.0.0.0", port=8080, title="Lay3rs Print Farm", dark=True, favicon=";)")