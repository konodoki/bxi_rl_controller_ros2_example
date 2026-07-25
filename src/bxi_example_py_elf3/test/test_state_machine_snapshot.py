import unittest

from bxi_example_py_elf3._runtime.state_machine import (
    ResolvedTransition,
    RobotStateMachine,
    TransitionRule,
)
from bxi_example_py_elf3.mod_api.state import StateBehavior


class _InstantPlan:
    def snapshot(self) -> dict[str, object]:
        return {"type": "instant"}


class StateMachineSnapshotTest(unittest.TestCase):
    def setUp(self) -> None:
        idle = StateBehavior("idle", 0)
        active = StateBehavior("active", 1)
        transition = ResolvedTransition("instant", _InstantPlan())

        machine = RobotStateMachine.__new__(RobotStateMachine)
        machine._states = {"idle": idle, "active": active}
        machine._profiles = {}
        machine._default_transition = transition
        machine._config = {
            "remote_events": {
                "activate_event": {"slot": "btn_1", "value": 1},
                "toggle_pause_event": {"slot": "btn_9", "value": 1},
            }
        }
        machine._rules = {
            "idle": [
                TransitionRule(
                    to_state="active",
                    event="activate_event",
                    transition=transition,
                ),
                TransitionRule(
                    event="toggle_pause_event",
                    action="toggle_pause",
                    transition=transition,
                ),
            ],
            "active": [],
        }
        self.machine = machine

    def test_action_only_event_is_filtered_from_published_graph(self) -> None:
        graph = self.machine._graph_snapshot()

        self.assertEqual(
            set(graph["remote_events"]),
            {"activate_event"},
        )
        self.assertEqual(len(graph["transitions"]), 1)
        self.assertEqual(graph["transitions"][0]["event"], "activate_event")
        self.assertEqual(graph["transitions"][0]["to"], "active")

    def test_action_only_event_remains_active_at_runtime(self) -> None:
        calls: list[str] = []
        self.machine.current = self.machine._states["idle"]
        self.machine._ctx = object()
        self.machine._actions = {"toggle_pause": lambda: calls.append("pause")}
        self.machine._active = None
        self.machine._pending = None

        self.machine._handle_events(["toggle_pause_event"])

        self.assertEqual(calls, ["pause"])
        self.assertIsNone(self.machine._active)
        self.assertIsNone(self.machine._pending)


if __name__ == "__main__":
    unittest.main()
