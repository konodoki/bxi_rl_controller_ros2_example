from bxi_example_py_elf3.utils.mod_system import ModDefinition, ModLoadContext
from .state import InitialPosState


def create_mod(context: ModLoadContext) -> ModDefinition:
    return ModDefinition(
        state_factories={
            "initial_pos": lambda state: InitialPosState(state.name, state.state_id)
        }
    )
