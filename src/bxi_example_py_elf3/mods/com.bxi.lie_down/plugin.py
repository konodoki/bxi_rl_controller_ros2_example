from bxi_example_py_elf3.inference.beyondmimic import DanceMotionPolicyGravityIsaaclabV2
from bxi_example_py_elf3.utils.mod_system import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from .state import LieDownState


POLICY = ResourceKey[DanceMotionPolicyGravityIsaaclabV2]("com.bxi.lie_down/policy")


def _load(context: ResourceLoadContext) -> DanceMotionPolicyGravityIsaaclabV2:
    return DanceMotionPolicyGravityIsaaclabV2(
        str(context.asset("assets/lie_down.npz")),
        str(context.asset("assets/lie_down.onnx")),
        start_frame=150,
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load)
    policy = context.resource(POLICY)
    return ModDefinition(
        state_factories={
            "lie_down": lambda state: LieDownState(
                state.name, state.state_id, policy
            )
        }
    )
