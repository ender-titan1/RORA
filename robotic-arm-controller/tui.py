import os, math
from abc import ABC, abstractmethod
from typing import Dict, Callable
from getch import getch, clear
from dataclasses import dataclass
from enum import Enum
from rich.live import Live
from rich.console import Console
from rich.text import Text

class InputMode(Enum):
    MAPPED = 0
    RAW = 1

class InterfaceComponent(ABC):
    @abstractmethod
    def render(self) -> str:
        pass

    @abstractmethod
    def on_goto(self, from_ui):
        pass

    @abstractmethod
    def on_input(self, key: str):
        pass

    @abstractmethod
    def get_manager(self):
        pass

    @abstractmethod
    def bind_manager(self, id, manager):
        pass

    def on_bind(self, manager):
        pass

class InterfaceManager:
    def __init__(self):
        self.current_ui: InterfaceComponent = None
        self.old_ui: InterfaceComponent = None
        self.input_mode = InputMode.MAPPED
        self.uis = {}
        self.input_map = {}
        self.quit = False
        self.state = {}
        self.console = Console(force_terminal=True)
        self.live = None

    def add_input(self, key: str, *ids: str):
        for id in ids:
            self.input_map[id.lower()] = key
        return self

    def add_ui(self, ui: InterfaceComponent, id: str):
        self.uis[id] = ui
        ui.bind_manager(id, self)
        ui.on_bind(self)
        return self
    
    def add_nav(self):
        self.add_input("up", "UP_ARROW", "w") \
            .add_input("down", "DOWN_ARROW", "s") \
            .add_input("enter", "\r", "\n") \
            .add_input("left", "LEFT_ARROW", "a") \
            .add_input("right", "RIGHT_ARROW", "d") \
            .add_input("exit", "CTRL+X")
        return self

    def render(self):
        return self.current_ui.render()

    def start_renderer(self):
        clear()
        with Live(self.render(), console=self.console, refresh_per_second=20) as live:
            self.live = live
            self.main()

    def main(self):
        while not self.quit:
            self.await_input()
            self.update()

    def goto(self, id: str):
        if id not in self.uis.keys():
            return
        
        self.input_mode = InputMode.MAPPED
        self.old_ui = self.current_ui
        self.current_ui = self.uis[id]
        self.current_ui.on_goto(self.old_ui)
        self.update()

    def update(self):
        if self.live:
            self.live.update(self.render())

    def quit_app(self):
        self.quit = True

    def await_input(self):
        char = getch()

        if self.input_mode == InputMode.RAW:
            key = char
        else:
            try:
                key = self.input_map[char.lower()]
            except(KeyError):
                return
            
        self.current_ui.on_input(key)

@dataclass
class ActionContext:
    ui: InterfaceComponent
    manager: InterfaceManager
    string: str

class AbstractSelection(InterfaceComponent):
    def __init__(self, sel_len: int):
        self.manager = None
        self.idx = 0
        self.sel_len = sel_len
        self.id = None

    def bind_manager(self, id, manager):
        self.manager = manager
        self.id = id

    def get_manager(self):
        return self.manager

    def on_goto(self, from_ui):
        self.idx = 0

    def on_input(self, key: str):
        if key == "down":
            self.idx += 1
        if key == "up":
            self.idx -= 1

        if self.idx >= self.sel_len:
            self.idx = self.sel_len - 1
        if self.idx < 0:
            self.idx = 0


class SimpleSelection(AbstractSelection):
    def __init__(self, selection: Dict[str, Callable[[ActionContext], None]]):
        super().__init__(len(selection))
        self.selection = selection

    def on_goto(self, from_ui):
        self.idx = 0

    def on_input(self, key: str):
        super().on_input(key)

        if key == "enter":
            for i, (k, v) in enumerate(zip(self.selection.keys(), self.selection.values())):
                if i == self.idx:
                    v(ActionContext(self, self.get_manager(), k))

    def __repr__(self):
        out = ""
        for i, s in enumerate(self.selection.keys()):
            out += s
            if self.idx == i:
                out += " <\n"
            else:
                out += "\n"

        return out

    def render(self):
        return str(self)