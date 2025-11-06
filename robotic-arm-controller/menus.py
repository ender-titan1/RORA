from tui import *
from typing import List, Callable
from abc import ABC, abstractmethod
from termcolor import colored
from rich.text import Text
import re

class Option(ABC):
    def __init__(self, name: str, action: Callable[[ActionContext], None]):
        self.action = action
        self.name = name

    @abstractmethod
    def __repr__(self):
        pass

    def on_input(self, ui, key):
        pass

    def on_bind(self, manager: InterfaceManager, parent: InterfaceComponent):
        pass

    def reset(self, manager):
        pass

    def state_name(self):
        return f"__option__{self.name}"

class DropdownOption(Option):
    def __init__(self, name, options, default):
        super().__init__(name, lambda ctx: ctx.manager.goto(f"__option__{self.name}"))
        self.default = default
        self.options = options
        self.i = default
        self.current = options[default]

    def on_bind(self, manager, parent):
        option_ui = SimpleSelection({
            x: (lambda ctx, i=i: DropdownOption.selection_method(manager, self, ctx.string, i)) for i, x in enumerate(self.options)
        })

        manager.add_ui(option_ui, f"__option__{self.name}")

        manager.state[f"__option__{self.name}"] = self.current

    def __repr__(self):
        return f"[{self.current}] ▼"
    
    @staticmethod
    def selection_method(manager, option, string, i):
        option.current = string
        option.i = i
        manager.state[f"__option__{option.name}"] = option.current
        manager.old_ui.trigger_event(option)
        manager.goto(manager.old_ui.id)

class SliderOption(Option):
    def __init__(self, name,
                 default: int, max: int, fill: bool = True,
                 left_label: str = "", right_label: str = "",
                 dispaly_func: Callable[[int], str] = None):
        super().__init__(name, lambda _: None)
        self.default = default
        self.current = default
        self.max = max
        self.fill = fill
        self.left = f"{left_label} " if left_label is not None else None
        self.right = f" {right_label}" if right_label is not None else None
        self.display_func = dispaly_func

    def on_input(self, ui, key):
        if key == "left":
            self.current -= 1
        elif key == "right":
            self.current += 1

        ui.trigger_event(self)

        if self.current > self.max:
            self.current = self.max
        
        if self.current < 1:
            self.current = 1

    def __repr__(self):
        empty_sq = "□"
        full_sq =  "■"
        remaining = (self.max - self.current)

        if (self.fill):
            string = (full_sq * self.current) + (empty_sq * remaining)
        else:
            empty = self.current - 1
            string = (empty_sq * empty) + full_sq + (empty_sq * remaining)

        display = ""

        if self.display_func != None:
            display = f" ({self.display_func(self.current)})"

        return f"{self.left}{string}{self.right}{display}"

class ButtonOption(Option):
    def __init__(self, name, action):
        super().__init__(name, action)

    def __repr__(self):
        return self.name

class TextInputOption(Option):
    def __init__(self, name, default="", regex=None):
        super().__init__(name, lambda ctx: TextInputOption.handler(ctx.ui, self, ctx.string))
        self.contents = str(default)
        self.default = str(default)
        self.regex = regex

    def on_bind(self, manager, parent):
        manager.state[f"__field__{self.name}"] = self.default

    def reset(self, manager):
        self.contents = self.default
        manager.state[f"__field__{self.name}"] = self.default

    @staticmethod
    def handler(ui: InterfaceComponent, opt: Option, s: str):
        manager = ui.get_manager()
        manager.live.stop()
        while True:
            clear()
            user_in = input(f"Input {s}: ")

            if opt.regex is None:
                break

            m = re.findall(opt.regex, user_in)
            
            if len(m) >= 1:
                break

            print("Input invald! (Press ctrl+x to cancel)")
            c = getch()

            if c == "CTRL+X":
                manager.live.start()
                manager.goto(ui.id)
                return

        opt.contents = user_in
        manager.state[f"__field__{s}"] = user_in
        ui.trigger_event(opt)
        manager.live.start()
        manager.goto(ui.id)

    def __repr__(self):
        return self.contents
    
    def state_name(self):
        return f"__field__{self.name}"

class Label(Option):
    def __init__(self, name):
        super().__init__(name, None)

    def __repr__(self):
        return self.name
    
class DynamicOption(Option):
    def __init__(self, name, options, default):
        super().__init__(name, None)

        self.option: Option = options[default]
        self.options = options
        self.action = self.option.action
        self.idx = default

    def on_input(self, ui, key):
        self.option.on_input(ui, key)

    def on_bind(self, manager, parent):
        for opt in self.options:
            opt.on_bind(manager, parent)

    def set_option(self, idx):
        self.option = self.options[idx]
        self.action = self.option.action
        self.idx = idx

    def __repr__(self):
        return self.option.__repr__()


class OptionSelection(AbstractSelection):
    def __init__(self, selection: List[Option], padding=0, preprocessor=None):
        super().__init__(len(selection))
        self.selection = selection

        self.preprocessor = preprocessor
        self.events = {}
        self.padding = padding
        self.max_name_len = max([len(o.name) for o in selection])
        self.edit_mode = False
        self.parent = None

    def add_event(self, option, func):
        self.events[option.name] = func
        return self

    def trigger_event(self, option):
        for (n, f) in self.events.items():
            if n == option.name:
                f(ActionContext(self, self.get_manager(), option.name))

    def on_goto(self, from_ui):
        if from_ui == None:
            return
        
        self.edit_mode = False
        
        if not "__option__" in from_ui.id:
            self.idx = 0

    def on_bind(self, manager):
        for o in self.selection:
            o.on_bind(manager, self)

    def on_input(self, key):
        if not self.edit_mode:
            super().on_input(key)

        # Skip over labels by moving twice
        if isinstance(self.selection[self.idx], Label):
            super().on_input(key)
        
        if key == "enter":
            self.edit_mode = not self.edit_mode

        for i, o in enumerate(self.selection):
            if i == self.idx and self.edit_mode:
                if o.action is None:
                    self.edit_mode = False
                    break

                if key == "enter":
                    o.action(ActionContext(self, self.get_manager(), o.name))
                else:
                    o.on_input(self, key)

    def __repr__(self):
        return ""

    def reset(self):
        for option in self.selection:
            option.reset(self.get_manager())

    def render(self):
        output = Text()
            
        if self.preprocessor is not None:
            pre = self.preprocessor(self.get_manager())
            if isinstance(pre, Text):
                output += pre
            else:
                output.append(str(pre))
            output.append("\n")

        for i, o in enumerate(self.selection):
            if isinstance(o, ButtonOption):
                btn_str = str(o)
                if i == self.idx:
                    btn_str += " <"

                output.append(f"{btn_str}\n")
                continue

            if isinstance(o, Label):
                output.append(str(o))
                output.append("\n")
                continue

            padding = self.max_name_len - len(o.name) + self.padding

            option_str = str(o)

            if i == self.idx and self.edit_mode:
                option_str_text = Text(option_str, style="bold black on white")
            else:
                option_str_text = Text(option_str)

            line = Text(f"{o.name}:{' ' * (padding + 1)}")
            line += option_str_text
            
            if i == self.idx:
                line += " <"

            output += line
            output.append("\n")

        return output
    