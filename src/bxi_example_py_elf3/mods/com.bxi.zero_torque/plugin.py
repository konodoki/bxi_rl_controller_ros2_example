from bxi_example_py_elf3.utils.mod_system import ModDefinition, ModLoadContext
from .state import ZeroTorqueState


def create_mod(context: ModLoadContext) -> ModDefinition:
    return ModDefinition(
        state_factories={
            "zero_torque": lambda state: ZeroTorqueState(state.name, state.state_id)
        }
    )
