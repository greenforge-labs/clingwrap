from launch import Action

from typing import Protocol


class ActionList(Protocol):
    def add_action(self, action: Action):
        ...


class ActionListImpl(ActionList):
    _actions: list[Action]

    def __init__(self):
        self._actions = []

    def add_action(self, action: Action):
        self._actions.append(action)

    @property
    def actions(self):
        return self._actions
