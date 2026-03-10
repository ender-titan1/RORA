from enum import IntEnum
from tui import *
from menus import *
import socket
import struct

class Direction(IntEnum):
    CW = 1
    CCW = 0

class Commands(IntEnum):
    PC_CMD_CREATE_CURVE = 0xF0
    PC_CMD_COMPUTE_CURVE = 0xFA
    PC_CMD_QUEUE_MOVEMENT = 0xFB
    PC_CMD_EXECUTE = 0xFC
    PC_CMD_MOVE = 0xE0

class EaseType(IntEnum):
    LINEAR = 0
    SINE = 1
    CUBIC = 2

class DirectionOverride(IntEnum):
    NONE = 0xFF
    CW = 0
    CCW = 1

class DisableOverride(IntEnum):
    NONE = 0xFF
    DISABLE = 1
    KEEP_ENABLED = 0

CURVE_CMD_STRUCT_FMT = '<BBBBBBffff'
COMPUTE_CMD_STRUCT_FMT = '<BBBBBfB'
EXECUTE_CMD_FMT = '<BBBBBBB'

NUMBER_REGEX = r"^\s*\d+(\.\d+)?\s*$"

slots = [[Text(), Text()] for _ in range(3)]
slots_sat = [[Text(), Text()]]

s = socket.socket()
s.connect(('192.168.4.1', 3333))
print("Connected to RORA")
getch()

#-------------TUI Components-------------
blank = Label("")
back_btn = ButtonOption("Back", lambda ctx: ctx.manager.goto("cmd"))
target_slot = SliderOption("Target Slot", 1, 3, False, display_val=True)
source_slot = SliderOption("Source Slot", 1, 3, False, display_val=True)
controller = DropdownOption("Controller", ["Core", "Satellite"], 0)

motor = DynamicOption[DropdownOption]("Motor", [
    DropdownOption("motor_core", ["Shoulder", "Elbow"], 0),
    DropdownOption("motor_sat", ["Base"], 0),
], 0)

duration = TextInputOption("Duration", 1, NUMBER_REGEX)
accel_time = TextInputOption("Accel. Time", 0.25, NUMBER_REGEX)
decel_time = TextInputOption("Decel. Time", 0.25, NUMBER_REGEX)
resolution = TextInputOption("Resolution", 0.01, NUMBER_REGEX)

degrees = TextInputOption("Degrees", "45", NUMBER_REGEX)

ease = EnumDropdownOption("Easing Type", EaseType, EaseType.SINE)
direction = EnumDropdownOption("Direction", Direction, Direction.CW, capitalize=False)
disable_override = EnumDropdownOption("[Override] Disable", DisableOverride, DisableOverride.NONE);
direction_override = EnumDropdownOption("[Override] Direction", DirectionOverride, DirectionOverride.NONE, capitalize=False);

#-------------Main Functions-------------

def show_slot_table(manager: InterfaceManager):

    sl = slots if controller.current == "Core" else slots_sat

    out = Text()
    for i, slot in enumerate(sl):
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

    sl = slots if controller.current == "Core" else slots_sat

    if ctx.manager.current_ui.id == "curve":
        packet = struct.pack(CURVE_CMD_STRUCT_FMT,
            int(Commands.PC_CMD_CREATE_CURVE),
            target_slot.value - 1,
            0,
            controller.value[0],
            motor.option.value[0] + 1,
            ease.value.value,
            float(accel_time.value),
            float(decel_time.value),
            float(duration.value),
            float(resolution.value)
        )
        s.send(packet)

        sl[target_slot.current - 1][0] = Text(f"{ease.current.ljust(6)} ⋮ {duration.value.rjust(5)}s ⋮ A: {accel_time.value.rjust(5)}s", "blue bold")

    if ctx.manager.current_ui.id == "compute":
        packet = struct.pack(COMPUTE_CMD_STRUCT_FMT,
            int(Commands.PC_CMD_COMPUTE_CURVE),
            target_slot.value - 1,
            source_slot.value - 1,
            controller.value[0],
            motor.option.value[0] + 1,
            float(degrees.value),
            direction.value.value
        )
        s.send(packet)

        sl[target_slot.current - 1][1] = Text(f"Src: {str(source_slot.current).ljust(1)} ⋮ {motor.option.current.rjust(10)} ⋮ {degrees.contents.rjust(3)} ⋮ {direction.current.rjust(3)} ", "blue bold")

    if ctx.manager.current_ui.id == "execute":
        packet = struct.pack(EXECUTE_CMD_FMT,
            int(Commands.PC_CMD_EXECUTE),
            0,
            source_slot.value - 1,
            controller.value[0],
            motor.option.value[0] + 1,
            direction_override.value.value,
            disable_override.value.value,
        )
        s.send(packet)


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
    controller, source_slot, blank, direction_override, disable_override, blank, send_btn, back_btn,
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
