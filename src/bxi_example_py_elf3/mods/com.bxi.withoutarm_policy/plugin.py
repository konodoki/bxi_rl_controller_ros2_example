from bxi_example_py_elf3.inference.amp import HumanoidGaitPolicyLiteIsaaclab
from bxi_example_py_elf3.utils.mod_system import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)


POLICY = ResourceKey[HumanoidGaitPolicyLiteIsaaclab]("com.bxi.withoutarm_policy/policy")


def _load(context: ResourceLoadContext) -> HumanoidGaitPolicyLiteIsaaclab:
    return HumanoidGaitPolicyLiteIsaaclab(str(context.asset("assets/withoutarm.onnx")))


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load)
    return ModDefinition()
