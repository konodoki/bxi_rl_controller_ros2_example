from bxi_example_py_elf3.inference.amp import HumanoidGaitPolicyLiteIsaaclab
from bxi_example_py_elf3.utils.mod_system import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)


POLICY = ResourceKey[HumanoidGaitPolicyLiteIsaaclab]("com.bxi.normal_policy/policy")


def _load(context: ResourceLoadContext) -> HumanoidGaitPolicyLiteIsaaclab:
    return HumanoidGaitPolicyLiteIsaaclab(str(context.asset("assets/amp_terrain.onnx")))


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load)
    return ModDefinition()
