import asyncio, sys
from nicegui import ui

PRINTERS = ["printer_03", "printer_04", "printer_05"]
JOBS = {"Go home": "go_home", "Localize (door)": "localize_printer_door_only"}
busy = False

async def run(job, printer, dry):
    global busy
    if busy:
        ui.notify("A job is already running", type="warning")
        return
    busy = True
    log.push(f"> {job} on {printer} (dry run: {dry})")
    cmd = [sys.executable, "/home/main/Lay3rs_print_farm/run_job.py", "--job", job, "--printer", printer]
    if dry:
        cmd.append("--dry-run")
    proc = await asyncio.create_subprocess_exec(
        *cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
    async for line in proc.stdout:
        log.push(line.decode().rstrip())
    await proc.wait()
    log.push(f"Finished (exit code {proc.returncode})")
    busy = False

ui.label("Print Farm Control").classes("text-3xl font-bold")
printer = ui.select(PRINTERS, value=PRINTERS[0], label="Printer").classes("w-64")
dry = ui.switch("Practice mode (robot won't move)", value=True)
with ui.row():
    for label, job in JOBS.items():
        ui.button(label, on_click=lambda j=job: run(j, printer.value, dry.value)).props("size=lg")
log = ui.log().classes("w-full h-64")
ui.run(host="0.0.0.0", port=8080, title="Print Farm")