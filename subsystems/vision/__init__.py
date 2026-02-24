"""
VisionSubsystem!

Reads all cameras, updates camera inputs and throttles, etc.
"""
from math import inf
from typing import Callable

from commands2 import Subsystem
from phoenix6 import units
from pykit.logger import Logger
from wpilib import Alert
from wpimath.geometry import Pose2d

from constants import Constants
from subsystems.vision.io import (CameraObservation, VisionIO, VisionObservation,
                                  ObservationType, VisionIOLimelight)


# pylint: disable=too-few-public-methods, too-many-instance-attributes
class VisionSubsystem(Subsystem):
    """el visione"""


    def __init__(
        self,
        vision_consumer: Callable[[Pose2d, units.second, tuple[float, float, float] | None], None],
        *ios: VisionIO) -> None:
        super().__init__()

        self._consumer = vision_consumer
        self._io = ios

        # Initialize inputs
        self._inputs = tuple(VisionIO.VisionIOInputs() for _ in ios)

        # Initialize disconnected alerts
        self._alerts = tuple(
            Alert(f"Camera {cam.get_name()} is disconnected.", Alert.AlertType.kWarning)
            for cam in ios
        )

        # Pre-allocate lists and other things for fewer lookups
        self._tag_poses = [[] for _ in self._inputs]
        self._robot_poses = [[] for _ in self._inputs]
        self._robot_poses_accepted = [[] for _ in self._inputs]
        self._robot_poses_rejected = [[] for _ in self._inputs]

        self._all_tag_poses = []
        self._all_robot_poses = []
        self._all_robot_poses_accepted = []
        self._all_robot_poses_rejected = []

        self._field_layout = Constants.FIELD_LAYOUT
        self._field_length = self._field_layout.getFieldLength()
        self._field_width = self._field_layout.getFieldWidth()

        self._max_ambiguity = Constants.VisionConstants.max_ambiguity
        self._max_z_error = Constants.VisionConstants.max_z_error

        self._linear_std_baseline = Constants.VisionConstants.linear_std_dev_baseline
        self._angular_std_baseline = Constants.VisionConstants.angular_std_dev_baseline

    # pylint: disable=too-many-locals
    def periodic(self) -> None:
        """Log and send all observations."""
        io = self._io
        inputs = self._inputs
        alerts = self._alerts
        max_amb = self._max_ambiguity
        max_z = self._max_z_error
        field = self._field_layout
        field_len = self._field_length
        field_wid = self._field_width
        lin_base = self._linear_std_baseline
        ang_base = self._angular_std_baseline
        tag_poses = self._tag_poses
        robot_poses = self._robot_poses
        robot_poses_accepted = self._robot_poses_accepted
        robot_poses_rejected = self._robot_poses_rejected
        all_tag_poses = self._all_tag_poses
        all_robot_poses = self._all_robot_poses
        all_robot_poses_accepted = self._all_robot_poses_accepted
        all_robot_poses_rejected = self._all_robot_poses_rejected
        consumer = self._consumer

        for i, (cam, inp) in enumerate(zip(io, inputs)):
            cam.update_inputs(inp)
            Logger.processInputs(f"Vision/{inp.name}", inputs[i])
            alerts[i].set(not inp.connected)

        all_tag_poses.clear()
        all_robot_poses.clear()
        all_robot_poses_accepted.clear()
        all_robot_poses_rejected.clear()
        for i, camera in enumerate(inputs):
            tag_poses = tag_poses[i]
            robot_poses = robot_poses[i]
            robot_poses_accepted = robot_poses_accepted[i]
            robot_poses_rejected = robot_poses_rejected[i]
            tag_poses.clear()
            robot_poses.clear()
            robot_poses_accepted.clear()
            robot_poses_rejected.clear()
            for tag_id in camera.tag_ids:
                pose = field.getTagPose(tag_id)
                if pose is not None:
                    tag_poses.append(pose)

            for observation in camera.observations:
                reject_pose = (
                    observation.tag_count == 0
                    or (
                    observation.tag_count == 1
                    and observation.ambiguity > max_amb
                    )
                    or abs(observation.pose.Z()) > max_z
                    or not (0.0 <= observation.pose.X() <= field_len)
                    or not (0.0 <= observation.pose.Y() <= field_wid)
                )

                robot_poses.append(observation.pose)
                if reject_pose:
                    robot_poses_rejected.append(observation.pose)
                else:
                    robot_poses_accepted.append(observation.pose)

                if reject_pose:
                    continue

                std_dev_factor = (
                        (observation.avg_tag_dist * observation.avg_tag_dist)
                        / observation.tag_count
                )
                linear_std = lin_base * std_dev_factor
                if observation.observation == ObservationType.MEGATAG_2.value:
                    angular_std = inf
                else:
                    angular_std = (ang_base *
                                   std_dev_factor)

                consumer(
                    observation.pose.toPose2d(),
                    observation.timestamp,
                    (linear_std, linear_std, angular_std)
                )

            Logger.recordOutput(f"Vision/{camera.name}/TagPoses", tag_poses)
            Logger.recordOutput(f"Vision/{camera.name}/RobotPoses", robot_poses)
            Logger.recordOutput(f"Vision/{camera.name}/RobotPosesAccepted", robot_poses_accepted)
            Logger.recordOutput(f"Vision/{camera.name}/RobotPosesRejected", robot_poses_rejected)
            all_tag_poses.extend(tag_poses)
            all_robot_poses.extend(robot_poses)
            all_robot_poses_accepted.extend(robot_poses_accepted)
            all_robot_poses_rejected.extend(robot_poses_rejected)

        Logger.recordOutput("Vision/AllTagPoses", all_tag_poses)
        Logger.recordOutput("Vision/AllRobotPoses", all_robot_poses)
        Logger.recordOutput("Vision/AllRobotPosesAccepted", all_robot_poses_accepted)
        Logger.recordOutput("Vision/AllRobotPosesRejected", all_robot_poses_rejected)

    def set_throttle(self, throttle: int):
        """
        Sets the throttle for all Limelights.
        This is to prevent overheating.
        """
        for cam in self._io:
            cam.set_throttle(throttle)
