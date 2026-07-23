from bxi_example_py_elf3.inference.amp import HumanoidGaitPolicyLiteIsaaclab
from bxi_example_py_elf3.utils.mod_system import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from .state import AmpRunState


POLICY = ResourceKey[HumanoidGaitPolicyLiteIsaaclab]("com.bxi.amp_run/policy")


def _load(context: ResourceLoadContext) -> HumanoidGaitPolicyLiteIsaaclab:
    return HumanoidGaitPolicyLiteIsaaclab(str(context.asset("assets/amp_run.onnx")))


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load)
    policy = context.resource(POLICY)
    return ModDefinition(
        state_factories={
            "amp_run": lambda state: AmpRunState(state.name, state.state_id, policy)
        }
    )
