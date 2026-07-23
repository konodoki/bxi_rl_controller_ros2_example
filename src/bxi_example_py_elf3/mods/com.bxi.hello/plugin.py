from bxi_example_py_elf3.inference.amp import HumanoidGaitPolicyLiteIsaaclab
from bxi_example_py_elf3.utils.mod_system import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
)
from .state import HelloState


POLICY = ResourceKey[HumanoidGaitPolicyLiteIsaaclab]("com.bxi.withoutarm_policy/policy")


def create_mod(context: ModLoadContext) -> ModDefinition:
    policy = context.resource(POLICY)
    return ModDefinition(
        state_factories={
            "hello": lambda state: HelloState(state.name, state.state_id, policy)
        }
    )
