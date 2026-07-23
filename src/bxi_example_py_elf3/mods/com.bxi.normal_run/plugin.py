from bxi_example_py_elf3.inference.normal import NormalMotionPolicyMjlab
from bxi_example_py_elf3.utils.mod_system import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from .state import NormalRunState


POLICY = ResourceKey[NormalMotionPolicyMjlab]("com.bxi.normal_run/policy")


def _load(context: ResourceLoadContext) -> NormalMotionPolicyMjlab:
    return NormalMotionPolicyMjlab(str(context.asset("assets/model_normal.onnx")))


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load)
    policy = context.resource(POLICY)
    return ModDefinition(
        state_factories={
            "normal_run": lambda state: NormalRunState(
                state.name, state.state_id, policy
            )
        }
    )
