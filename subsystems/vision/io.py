"""
Structs and IOs for the VisionSubsystem.

Structs are used to simplify logging and reduce logging overhead.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from math import radians
from typing import List, Callable

from ntcore import NetworkTableInstance
from pykit.autolog import autolog
from wpilib import RobotController
from wpimath.geometry import Pose3d, Transform3d, Rotation2d, Rotation3d, Pose2d
from wpiutil.wpistruct import make_wpistruct


class ObservationType(Enum):
    """
    Types of observations from different cameras.

    MegaTag 1 and 2 comes from Limelight cameras, PhotonVision comes from simulation.
    """
    MEGATAG_1 = auto()
    MEGATAG_2 = auto()
    PHOTONVISION = auto()


@make_wpistruct(name="CameraObservation")
@autolog
@dataclass
class CameraObservation:
    """Observation struct from cameras."""
    timestamp: float = 0
    pose: Pose3d = field(default_factory=Pose3d)
    ambiguity: float = 0
    tag_count: int = 0
    avg_tag_dist: float = 0
    observation: int = ObservationType.PHOTONVISION.value


@dataclass
class VisionObservation:
    """
    Represents a vision measurement for the robot pose estimator.
    """
    pose: Pose2d
    timestamp: float
    std: tuple[float, float, float]


# pylint: disable=too-few-public-methods
class VisionIO(ABC):
    """Base class for VisionIO."""

    @autolog
    @dataclass
    class VisionIOInputs:
        """Inputs for the VisionSubsystem."""
        name: str = "Unnamed Camera"
        connected: bool = False
        observations: List[CameraObservation] = field(default_factory=list)
        tag_ids: List[int] = field(default_factory=list)


    @abstractmethod
    def update_inputs(self, inputs: VisionIOInputs) -> None:
        """Update subsystem inputs."""

    @abstractmethod
    def set_throttle(self, throttle: int) -> None:
        """Set the throttle."""

    @abstractmethod
    def get_name(self) -> str:
        """Returns the name of the camera."""


# pylint: disable=too-many-instance-attributes
class VisionIOLimelight(VisionIO):
    """VisionIO for Limelight cameras."""

    def __init__(self,
                 name: str,
                 transform: Transform3d,
                 rotation_supplier: Callable[[], Rotation2d]) -> None:
        super().__init__()
        self.name = name
        self.location = transform
        self.table = NetworkTableInstance.getDefault().getTable(name)
        self.rotation_supplier = rotation_supplier
        self.valid_target = self.table.getIntegerTopic("tv").subscribe(0)
        self.latency = self.table.getDoubleTopic("tl").subscribe(0.0)

        self.megatag1 = self.table.getDoubleArrayTopic("botpose_wpiblue").subscribe([])
        self.megatag2 = self.table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe([])

        self.cam_pose_setter = self.table.getDoubleArrayTopic(
            "camerapose_robotspace_set"
        ).publish()
        self.orientation_publisher = self.table.getDoubleArrayTopic(
            "robot_orientation_set"
        ).publish()
        self.throttle_set = self.table.getDoubleTopic("throttle_set").publish()


    def update_inputs(self, inputs: VisionIO.VisionIOInputs) -> None:
        """Update subsystem inputs."""
        inputs.name = self.name

        # We're considered connected if an update has been seen in the last 250ms
        latency_ms = (RobotController.getFPGATime() - self.latency.getLastChange()) / 1000
        inputs.connected = latency_ms < 250

        # Update orientation
        self.orientation_publisher.set(
            [self.rotation_supplier().degrees(), 0, 0, 0, 0, 0]
        )

        # Horrendous for network traffic but LL says to so...
        NetworkTableInstance.getDefault().flush()

        tag_ids = []
        pose_observations = []

        for sample in self.megatag1.readQueue():
            if len(sample.value) == 0:
                continue
            for i in range(11, len(sample.value), 7):
                tag_ids.append(int(sample.value[i]))
                pose_observations.append(
                    CameraObservation(
                        # Timestamp (based on server timestamp of publish and latency)
                        sample.time * 1e-6 - sample.value[6] * 1e-3,

                        # 3D pose estimate
                        VisionIOLimelight.parse_pose(sample.value),

                        # Ambiguity
                        # Using only the first tag because ambiguity isn't applicable for multi tag
                        sample.value[17] if len(sample.value) >= 18 else 0,

                        # Tag count
                        int(sample.value[7]),

                        # Average tag distance
                        sample.value[9],

                        # Observation type
                        ObservationType.MEGATAG_1.value
                    )
                )

        for sample in self.megatag2.readQueue():
            if len(sample.value) == 0:
                continue
            for i in range(11, len(sample.value), 7):
                tag_ids.append(int(sample.value[i]))
                pose_observations.append(
                    CameraObservation(
                        # Timestamp (based on server timestamp of publish and latency)
                        sample.time * 1e-6 - sample.value[6] * 1e-3,

                        # 3D pose estimate
                        VisionIOLimelight.parse_pose(sample.value),

                        # Ambiguity (0 because MegaTag 2 is already disambiguated)
                        0.0,

                        # Tag count
                        int(sample.value[7]),

                        # Average tag distance
                        sample.value[9],

                        # Observation type
                        ObservationType.MEGATAG_2.value
                    )
                )

        inputs.observations = pose_observations
        inputs.tag_ids = tag_ids

    def set_throttle(self, throttle: int) -> None:
        """Set the throttle."""
        self.throttle_set.set(throttle)

    def get_name(self) -> str:
        return self.name

    @staticmethod
    def parse_pose(raw_ll_array: list[float]) -> Pose3d:
        """Translated from Limelight's LimelightHelpers.java"""
        return Pose3d(
            raw_ll_array[0],
            raw_ll_array[1],
            raw_ll_array[2],
            Rotation3d(
                radians(raw_ll_array[3]),
                radians(raw_ll_array[4]),
                radians(raw_ll_array[5])
            )
        )
