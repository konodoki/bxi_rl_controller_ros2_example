from bxi_example_py_elf3.inference.beyondmimic import DanceMotionPolicyMjlab
from bxi_example_py_elf3.utils.mod_system import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from .state import RecoverState


POLICY = ResourceKey[DanceMotionPolicyMjlab]("com.bxi.recover/policy")


def _load(context: ResourceLoadContext) -> DanceMotionPolicyMjlab:
    return DanceMotionPolicyMjlab(
        str(context.asset("assets/recover.npz")),
        str(context.asset("assets/recover.onnx")),
        start_frame=600,
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load)
    policy = context.resource(POLICY)
    return ModDefinition(
        state_factories={
            "recover": lambda state: RecoverState(state.name, state.state_id, policy)
        }
    )
