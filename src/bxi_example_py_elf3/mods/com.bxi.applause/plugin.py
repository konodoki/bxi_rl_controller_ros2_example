from bxi_example_py_elf3.inference.amp import HumanoidGaitPolicyLiteIsaaclab
from bxi_example_py_elf3.utils.mod_system import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from .state import ApplauseState, PlaybackClip, load_clip


POLICY = ResourceKey[HumanoidGaitPolicyLiteIsaaclab]("com.bxi.withoutarm_policy/policy")
CLIP = ResourceKey[PlaybackClip]("com.bxi.applause/clip")


def _load_clip(context: ResourceLoadContext) -> PlaybackClip:
    return load_clip(
        context.asset("assets/applause.pkl"), start_frame=600, tail_trim_frames=600
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(CLIP, _load_clip)
    policy = context.resource(POLICY)
    clip = context.resource(CLIP)
    return ModDefinition(
        state_factories={
            "applause": lambda state: ApplauseState(
                state.name, state.state_id, policy, clip
            )
        }
    )
