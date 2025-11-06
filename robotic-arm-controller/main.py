from enum import IntEnum
from tui import *
from menus import *
import socket
import struct

class Commands(IntEnum):
    PC_CMD_CREATE_CURVE = 0xF0
    PC_CMD_COMPUTE_CURVE = 0xFA
    PC_CMD_QUEUE_MOVEMENT = 0xFB
    PC_CMD_EXECUTE = 0xFC
    PC_CMD_MOVE = 0xE0

class EaseType(Enum):
    EASE_LINEAR = 0
    EASE_SINE = 1
    EASE_CUBIC = 2

CURVE_CMD_STRUCT_FMT = '<BBBBBBffff'
COMPUTE_CMD_STRUCT_FMT = '<BBBBBfB'
EXECUTE_CMD_FMT = '<BBBBB'

slots = [[Text(), Text()] for _ in range(3)]

#s = socket.socket()
#s.connect(('192.168.4.1', 3333))
print("Connected to RORA")
getch()

#-------------TUI Components-------------
blank = Label("")
back_btn = ButtonOption("Back", lambda ctx: ctx.manager.goto("cmd"))
target_slot = SliderOption("Target Slot", 1, 3, False, dispaly_func=(lambda x: str(x)))
source_slot = SliderOption("Source Slot", 1, 3, False, dispaly_func=(lambda x: str(x)))
controller = DropdownOption("Controller", ["Core", "Satellite"], 0)

motor = DynamicOption("Motor", [
    DropdownOption("__core__Motor", ["Shoulder", "Elbow"], 0),
    DropdownOption("__sat__Motor", ["Base"], 0),
], 0)

duration = TextInputOption("Duration", 1, r"^\s*\d+(\.\d+)?\s*$")
accel_time = TextInputOption("Accel. Time", 0.25, r"^\s*\d+(\.\d+)?\s*$")
decel_time = TextInputOption("Decel. Time", 0.25, r"^\s*\d+(\.\d+)?\s*$")
resolution = TextInputOption("Resolution", 0.01, r"^\s*\d+(\.\d+)?\s*$")

degrees = TextInputOption("Degrees", "", r"^\s*\d+(\.\d+)?\s*$")

ease = DropdownOption("Easing Type", ["Linear", "Sine", "Cubic"], 1)
direction = DropdownOption("Direction", ["CCW", "CW"], 1)

#-------------Main Functions-------------

def show_slot_table(manager: InterfaceManager):
    out = Text()
    for i, slot in enumerate(slots):
        out.append(Text(f"Slot {i+1}".ljust(8)))
        out.append(" | ")
        s0 = slot[0]
        s0.align("left", 27)
        out.append(s0)
        out.append(" | ")
        s1 = slot[1]
        s1.align("left", 31)
        out.append(s1)
        out.append(" | ")
        out.append("\n")

    return out


def send_cmd(ctx: ActionContext) -> None:
    if ctx.manager.current_ui.id == "curve":
        packet = struct.pack(CURVE_CMD_STRUCT_FMT,
            int(Commands.PC_CMD_CREATE_CURVE),
            target_slot.current - 1,
            0,
            controller.i,
            1,
            ease.i,
            float(accel_time.contents),
            float(decel_time.contents),
            float(duration.contents),
            float(resolution.contents)
        )
        #s.send(packet)

        slots[target_slot.current - 1][0] = Text(f"{ease.current.ljust(6)} ⋮ {duration.contents.rjust(5)}s ⋮ A: {accel_time.contents.rjust(5)}s", "blue bold")

    if ctx.manager.current_ui.id == "compute":
        packet = struct.pack(COMPUTE_CMD_STRUCT_FMT,
            int(Commands.PC_CMD_COMPUTE_CURVE),
            target_slot.current - 1,
            source_slot.current - 1,
            controller.i,
            1,
            float(degrees.contents),
            direction.i
        )
        #s.send(packet)

        slots[target_slot.current - 1][1] = Text(f"Src: {str(source_slot.current).ljust(1)} ⋮ {motor.option.current.rjust(10)} ⋮ {degrees.contents.rjust(3)} ⋮ {direction.current.rjust(3)} ", "blue bold")

    if ctx.manager.current_ui.id == "execute":
        packet = struct.pack(EXECUTE_CMD_FMT,
            int(Commands.PC_CMD_EXECUTE),
            0,
            source_slot.current - 1,
            controller.i,
            1
        )
        #s.send(packet)

    ctx.manager.goto("cmd")

def update_motor(ctx):
    motor.set_option(0 if controller.current == "Core" else 1)

send_btn = ButtonOption("Send", send_cmd)

#-------------TUI Menus-------------

curve_menu = OptionSelection([
    controller, target_slot, blank, ease, duration, accel_time, decel_time, resolution, blank, send_btn, back_btn
], 2, preprocessor=show_slot_table).add_event(controller, update_motor)

compute_menu = OptionSelection([
    controller, motor, target_slot, source_slot, blank, degrees, direction, blank, send_btn, back_btn
], 2, preprocessor=show_slot_table).add_event(controller, update_motor)

exec_menu = OptionSelection([
    controller, source_slot, blank, send_btn, back_btn
], 2, preprocessor=show_slot_table).add_event(controller, update_motor)

cmd_menu = SimpleSelection({
    "Create Curve": lambda ctx: ctx.manager.goto("curve"),
    "Compute Curve": lambda ctx: ctx.manager.goto("compute"),
    "Execute": lambda ctx: ctx.manager.goto("execute"),
    "Exit": lambda ctx: ctx.manager.quit_app()
})

app = InterfaceManager() \
    .add_nav() \
    .add_ui(cmd_menu, "cmd") \
    .add_ui(curve_menu, "curve") \
    .add_ui(compute_menu, "compute") \
    .add_ui(exec_menu, "execute")

app.goto("cmd")
app.start_renderer()
