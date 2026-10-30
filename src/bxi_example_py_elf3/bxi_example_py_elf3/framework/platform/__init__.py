"""Public robot platform integration API."""

from typing import TYPE_CHECKING

from .cpu_affinity import CpuAffinityPlan, CpuAffinityRole, CpuAffinitySpec
if TYPE_CHECKING:
    from .api import ControlPlatformAdapter, RobotObservation
    from .joint_io import (
        FixedOrderJointCommandEncoder,
        FixedOrderJointStateSource,
        NamedJointCommandEncoder,
        NamedJointStateSource,
    )
    from .synchronization import (
        ApproximateTimeSynchronizer, ExactTimeSynchronizer, TimeSynchronizer,
        QueueSynchronizer, SynchronizedSample, SynchronizationResult,
    )
    from .joint_sync import SynchronizedJointStateSource
    from .runtime import ControlRuntimeConfig, RobotControlRuntime


def __getattr__(name: str):
    if name in {
        "ApproximateTimeSynchronizer", "ExactTimeSynchronizer", "TimeSynchronizer",
        "QueueSynchronizer", "SynchronizedSample", "SynchronizationResult",
    }:
        from . import synchronization
        return getattr(synchronization, name)
    if name == "SynchronizedJointStateSource":
        from .joint_sync import SynchronizedJointStateSource
        return SynchronizedJointStateSource
    if name in {"ControlPlatformAdapter", "RobotObservation"}:
        from .api import ControlPlatformAdapter, RobotObservation

        return {
            "ControlPlatformAdapter": ControlPlatformAdapter,
            "RobotObservation": RobotObservation,
        }[name]
    if name in {
        "FixedOrderJointCommandEncoder",
        "FixedOrderJointStateSource",
        "NamedJointCommandEncoder",
        "NamedJointStateSource",
    }:
        from .joint_io import (
            FixedOrderJointCommandEncoder,
            FixedOrderJointStateSource,
            NamedJointCommandEncoder,
            NamedJointStateSource,
        )

        return {
            "FixedOrderJointCommandEncoder": FixedOrderJointCommandEncoder,
            "FixedOrderJointStateSource": FixedOrderJointStateSource,
            "NamedJointCommandEncoder": NamedJointCommandEncoder,
            "NamedJointStateSource": NamedJointStateSource,
        }[name]
    if name in {"ControlRuntimeConfig", "RobotControlRuntime"}:
        from .runtime import ControlRuntimeConfig, RobotControlRuntime

        return {
            "ControlRuntimeConfig": ControlRuntimeConfig,
            "RobotControlRuntime": RobotControlRuntime,
        }[name]
    raise AttributeError(name)

__all__ = [
    "ControlPlatformAdapter",
    "ControlRuntimeConfig",
    "CpuAffinityPlan",
    "CpuAffinityRole",
    "CpuAffinitySpec",
    "FixedOrderJointCommandEncoder",
    "FixedOrderJointStateSource",
    "NamedJointCommandEncoder",
    "NamedJointStateSource",
    "RobotControlRuntime",
    "RobotObservation",
    "SynchronizedJointStateSource",
    "ApproximateTimeSynchronizer",
    "ExactTimeSynchronizer",
    "TimeSynchronizer",
    "QueueSynchronizer",
    "SynchronizedSample",
    "SynchronizationResult",
]
