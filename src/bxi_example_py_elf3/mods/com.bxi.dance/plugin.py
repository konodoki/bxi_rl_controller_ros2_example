from bxi_example_py_elf3.inference.beyondmimic import DanceMotionPolicyGravityIsaaclabV3
from bxi_example_py_elf3.utils.mod_system import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from .state import DanceState


POLICY = ResourceKey[DanceMotionPolicyGravityIsaaclabV3]("com.bxi.dance/policy")


def _load(context: ResourceLoadContext) -> DanceMotionPolicyGravityIsaaclabV3:
    return DanceMotionPolicyGravityIsaaclabV3(
        str(context.asset("assets/shuishou.npz")),
        str(context.asset("assets/shuishou.onnx")),
        start_frame=60,
        fixed_pos=True,
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load)
    policy = context.resource(POLICY)
    return ModDefinition(
        state_factories={
            "dance": lambda state: DanceState(
                state.name,
                state.state_id,
                policy,
                start_frame=state.int_param("start_frame", 100),
            )
        }
    )
