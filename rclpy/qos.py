# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum, IntEnum
from typing import (Any, List, Optional, Tuple, TypeVar, Union)
import warnings

from rclpy.duration import Duration
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from typing_extensions import TypeAlias


class QoSPolicyKind(IntEnum):
    """
    Enum for types of QoS policies that a Publisher or Subscription can set.

    This enum matches the one defined in rmw/incompatible_qos_events_statuses.h
    """

    # TODO(mm3188): obtain these enum values from the rmw layer, instead of hardcoding
    INVALID = 1 << 0
    DURABILITY = 1 << 1
    DEADLINE = 1 << 2
    LIVELINESS = 1 << 3
    RELIABILITY = 1 << 4
    HISTORY = 1 << 5
    LIFESPAN = 1 << 6,
    DEPTH = 1 << 7,
    LIVELINESS_LEASE_DURATION = 1 << 8,
    AVOID_ROS_NAMESPACE_CONVENTIONS = 1 << 9,


def qos_policy_name_from_kind(policy_kind: Union[QoSPolicyKind, int]) -> str:
    """Get QoS policy name from QoSPolicyKind enum."""
    return QoSPolicyKind(policy_kind).name


class InvalidQoSProfileException(Exception):
    """Raised when constructing a QoSProfile with invalid arguments."""

    def __init__(self, message: str) -> None:
        Exception(self, f'Invalid QoSProfile: {message}')


class QoSProfile:
    """Define Quality of Service policies."""

    # default QoS profile not exposed to the user to encourage them to think about QoS settings
    __qos_profile_default_dict = \
        _rclpy.rmw_qos_profile_t.predefined('qos_profile_default').to_dict()

    __slots__ = [
        '_history',
        '_depth',
        '_reliability',
        '_durability',
        '_lifespan',
        '_deadline',
        '_liveliness',
        '_liveliness_lease_duration',
        '_avoid_ros_namespace_conventions',
    ]

    def __init__(self,
                 history: Union['QoSHistoryPolicy', int, None] = None,
                 depth: Optional[int] = None,
                 reliability: Union['QoSReliabilityPolicy', int, None] = None,
                 durability: Union['QoSDurabilityPolicy', int, None] = None,
                 lifespan: Optional[Duration] = None,
                 deadline: Optional[Duration] = None,
                 liveliness: Union['QoSLivelinessPolicy', int, None] = None,
                 liveliness_lease_duration: Optional[Duration] = None,
                 avoid_ros_namespace_conventions: Optional[bool] = None) -> None:

        if history is None:
            if depth is None:
                raise InvalidQoSProfileException('History and/or depth settings are required.')
            history = QoSHistoryPolicy.KEEP_LAST

        self.history = history

        if (
            QoSHistoryPolicy.KEEP_LAST == self.history and depth is None
        ):
            raise InvalidQoSProfileException('History set to KEEP_LAST without a depth setting.')

        self.depth = QoSProfile.__qos_profile_default_dict['depth'] if depth is None else depth

        self.reliability = QoSProfile.__qos_profile_default_dict['reliability'] \
            if reliability is None else reliability

        self.durability = QoSProfile.__qos_profile_default_dict['durability'] \
            if durability is None else durability

        self.lifespan = QoSProfile.__qos_profile_default_dict['lifespan'] \
            if lifespan is None else lifespan

        self.deadline = QoSProfile.__qos_profile_default_dict['deadline'] \
            if deadline is None else deadline

        self.liveliness = QoSProfile.__qos_profile_default_dict['liveliness'] \
            if liveliness is None else liveliness

        self.liveliness_lease_duration =  \
            QoSProfile.__qos_profile_default_dict['liveliness_lease_duration'] \
            if liveliness_lease_duration is None else liveliness_lease_duration

        self.avoid_ros_namespace_conventions = \
            QoSProfile.__qos_profile_default_dict['avoid_ros_namespace_conventions'] \
            if avoid_ros_namespace_conventions is None else avoid_ros_namespace_conventions

    # Has to be marked Any due to mypy#3004. Return type is actually QoSHistoryPolicy
    @property
    def history(self) -> Any:
        """
        Get field 'history'.

        :returns: history attribute
        """
        return self._history

    @history.setter
    def history(self, value: Union['QoSHistoryPolicy', int]) -> None:
        assert isinstance(value, QoSHistoryPolicy) or isinstance(value, int)
        self._history = QoSHistoryPolicy(value)

    # Has to be marked Any due to mypy#3004. Return type is actually QoSReliabilityPolicy
    @property
    def reliability(self) -> Any:
        """
        Get field 'reliability'.

        :returns: reliability attribute
        """
        return self._reliability

    @reliability.setter
    def reliability(self, value: Union['QoSReliabilityPolicy', int]) -> None:
        assert isinstance(value, QoSReliabilityPolicy) or isinstance(value, int)
        self._reliability = QoSReliabilityPolicy(value)

    # Has to be marked Any due to mypy#3004. Return type is actually QoSDurabilityPolicy
    @property
    def durability(self) -> Any:
        """
        Get field 'durability'.

        :returns: durability attribute
        """
        return self._durability

    @durability.setter
    def durability(self, value: Union['QoSDurabilityPolicy', int]) -> None:
        assert isinstance(value, QoSDurabilityPolicy) or isinstance(value, int)
        self._durability = QoSDurabilityPolicy(value)

    @property
    def depth(self) -> int:
        """
        Get field 'depth'.

        :returns: depth attribute
        """
        return self._depth

    @depth.setter
    def depth(self, value: int) -> None:
        assert isinstance(value, int)

        if self.history == QoSHistoryPolicy.KEEP_LAST and value == 0:
            warnings.warn(
                "A zero depth with KEEP_LAST doesn't make sense; no data could be stored. "
                'This will be interpreted as SYSTEM_DEFAULT')

        self._depth = value

    @property
    def lifespan(self) -> Duration:
        """
        Get field 'lifespan'.

        :returns: lifespan attribute
        """
        return self._lifespan

    @lifespan.setter
    def lifespan(self, value: Duration) -> None:
        assert isinstance(value, Duration)
        self._lifespan = value

    @property
    def deadline(self) -> Duration:
        """
        Get field 'deadline'.

        :returns: deadline attribute.
        """
        return self._deadline

    @deadline.setter
    def deadline(self, value: Duration) -> None:
        assert isinstance(value, Duration)
        self._deadline = value

    # Has to be marked Any due to mypy#3004. Return type is actually QoSLivelinessPolicy
    @property
    def liveliness(self) -> Any:
        """
        Get field 'liveliness'.

        :returns: liveliness attribute
        """
        return self._liveliness

    @liveliness.setter
    def liveliness(self, value: Union['QoSLivelinessPolicy', int]) -> None:
        assert isinstance(value, (QoSLivelinessPolicy, int))
        self._liveliness = QoSLivelinessPolicy(value)

    @property
    def liveliness_lease_duration(self) -> Duration:
        """
        Get field 'liveliness_lease_duration'.

        :returns: liveliness_lease_duration attribute.
        """
        return self._liveliness_lease_duration

    @liveliness_lease_duration.setter
    def liveliness_lease_duration(self, value: Duration) -> None:
        assert isinstance(value, Duration)
        self._liveliness_lease_duration = value

    @property
    def avoid_ros_namespace_conventions(self) -> bool:
        """
        Get field 'avoid_ros_namespace_conventions'.

        :returns: avoid_ros_namespace_conventions attribute
        """
        return self._avoid_ros_namespace_conventions

    @avoid_ros_namespace_conventions.setter
    def avoid_ros_namespace_conventions(self, value: bool) -> None:
        assert isinstance(value, bool)
        self._avoid_ros_namespace_conventions = value

    def get_c_qos_profile(self) -> _rclpy.rmw_qos_profile_t:
        return _rclpy.rmw_qos_profile_t(
            self.history,
            self.depth,
            self.reliability,
            self.durability,
            self.lifespan.get_c_duration(),
            self.deadline.get_c_duration(),
            self.liveliness,
            self.liveliness_lease_duration.get_c_duration(),
            self.avoid_ros_namespace_conventions,
        )

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, QoSProfile):
            return False
        return all(
            self.__getattribute__(slot) == other.__getattribute__(slot)
            for slot in self.__slots__)

    def __str__(self) -> str:
        return f'{type(self).__name__}(%s)' % (
            ', '.join(f'{slot[1:]}=%s' % getattr(self, slot) for slot in self.__slots__)
        )


class QoSPolicyEnum(IntEnum):
    """
    Base for QoS Policy enumerations.

    Provides helper function to filter keys for utilities.
    """

    @classmethod
    def short_keys(cls) -> List[str]:
        """Return a list of shortened typing-friendly enum values."""
        return [k.lower() for k in cls.__members__.keys() if not k.startswith('RMW')]

    @classmethod
    def get_from_short_key(cls, name: str) -> int:
        """Retrieve a policy type from a short name, case-insensitive."""
        return cls[name.upper()].value

    @property
    def short_key(self) -> str:
        for k, v in self.__class__.__members__.items():
            if k.startswith('RMW'):
                continue
            if self.value == v:
                return k.lower()
        raise AttributeError(
            'failed to find value %s in %s' %
            (self.value, self.__class__.__name__))


_T = TypeVar('_T', bound=QoSPolicyEnum)


class HistoryPolicy(QoSPolicyEnum):
    """
    Enum for QoS History settings.

    This enum matches the one defined in rmw/types.h
    """

    SYSTEM_DEFAULT = 0
    KEEP_LAST = 1
    KEEP_ALL = 2
    UNKNOWN = 3


# Alias with the old name, for retrocompatibility
QoSHistoryPolicy: TypeAlias = HistoryPolicy


class ReliabilityPolicy(QoSPolicyEnum):
    """
    Enum for QoS Reliability settings.

    This enum matches the one defined in rmw/types.h
    """

    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2
    UNKNOWN = 3
    BEST_AVAILABLE = 4


# Alias with the old name, for retrocompatibility
QoSReliabilityPolicy: TypeAlias = ReliabilityPolicy


class DurabilityPolicy(QoSPolicyEnum):
    """
    Enum for QoS Durability settings.

    This enum matches the one defined in rmw/types.h
    """

    SYSTEM_DEFAULT = 0
    TRANSIENT_LOCAL = 1
    VOLATILE = 2
    UNKNOWN = 3
    BEST_AVAILABLE = 4


# Alias with the old name, for retrocompatibility
QoSDurabilityPolicy: TypeAlias = DurabilityPolicy


class LivelinessPolicy(QoSPolicyEnum):
    """
    Enum for QoS Liveliness settings.

    This enum matches the one defined in rmw/types.h
    """

    SYSTEM_DEFAULT = 0
    AUTOMATIC = 1
    MANUAL_BY_TOPIC = 3
    UNKNOWN = 4
    BEST_AVAILABLE = 5


# Alias with the old name, for retrocompatibility
QoSLivelinessPolicy: TypeAlias = LivelinessPolicy

# Deadline policy to match the majority of endpoints while being as strict as possible
# See `RMW_QOS_DEADLINE_BEST_AVAILABLE` in rmw/types.h for more info.
DeadlineBestAvailable = Duration(nanoseconds=_rclpy.RMW_QOS_DEADLINE_BEST_AVAILABLE)

# Liveliness lease duraiton policy to match the majority of endpoints while being as strict as
# possible
# See `RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE` in rmw/types.h for more info.
LivelinessLeaseDurationeBestAvailable = Duration(
    nanoseconds=_rclpy.RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE
)


# The details of the following profiles can be found at
# 1. ROS QoS principles:
#    https://design.ros2.org/articles/qos.html
# 2. ros2/rmw : rmw/include/rmw/qos_profiles.h

#: Used for initialization. Should not be used as the actual QoS profile.
qos_profile_unknown = QoSProfile(**_rclpy.rmw_qos_profile_t.predefined(
    'qos_profile_unknown').to_dict())
qos_profile_default = QoSProfile(**_rclpy.rmw_qos_profile_t.predefined(
    'qos_profile_default').to_dict())
#: Uses the default QoS settings defined in the DDS vendor tool
qos_profile_system_default = QoSProfile(**_rclpy.rmw_qos_profile_t.predefined(
    'qos_profile_system_default').to_dict())
#: For sensor data, using best effort reliability and small
#: queue depth
qos_profile_sensor_data = QoSProfile(**_rclpy.rmw_qos_profile_t.predefined(
    'qos_profile_sensor_data').to_dict())
#: For services, using reliable reliability and volatile durability
qos_profile_services_default = QoSProfile(**_rclpy.rmw_qos_profile_t.predefined(
    'qos_profile_services_default').to_dict())
#: For parameter communication. Similar to service QoS profile but with larger
#: queue depth so that requests do not get lost.
qos_profile_parameters = QoSProfile(**_rclpy.rmw_qos_profile_t.predefined(
    'qos_profile_parameters').to_dict())
#: For parameter change events. Currently same as the QoS profile for
#: parameters.
qos_profile_parameter_events = QoSProfile(**_rclpy.rmw_qos_profile_t.predefined(
    'qos_profile_parameter_events').to_dict())
#: Match majority of endpoints currently available while maintaining the highest level of service.
#: Policies are chosen at the time of creating a subscription or publisher.
#: The middleware is not expected to update policies after creating a subscription or
#: publisher, even if one or more policies are incompatible with newly discovered endpoints.
#: Therefore, this profile should be used with care since non-deterministic behavior
#: can occur due to races with discovery.
qos_profile_best_available = QoSProfile(**_rclpy.rmw_qos_profile_t.predefined(
    'qos_profile_best_available').to_dict())
# Separate rcl_action profile defined at
# ros2/rcl : rcl/rcl_action/include/rcl_action/default_qos.h
#
#: For actions, using reliable reliability, transient-local durability.
qos_profile_action_status_default = QoSProfile(**_rclpy.rclpy_action_get_rmw_qos_profile(
    'rcl_action_qos_profile_status_default'))
#: The default qos profile setting for topic /rosout publisher.
qos_profile_rosout_default = QoSProfile(**_rclpy.rmw_qos_profile_t.predefined(
    'qos_profile_rosout_default').to_dict())


class QoSPresetProfiles(Enum):
    UNKNOWN = qos_profile_unknown
    DEFAULT = qos_profile_default
    SYSTEM_DEFAULT = qos_profile_system_default
    SENSOR_DATA = qos_profile_sensor_data
    SERVICES_DEFAULT = qos_profile_services_default
    PARAMETERS = qos_profile_parameters
    PARAMETER_EVENTS = qos_profile_parameter_events
    ACTION_STATUS_DEFAULT = qos_profile_action_status_default
    BEST_AVAILABLE = qos_profile_best_available
    ROSOUT_DEFAULT = qos_profile_rosout_default

    """Noted that the following are duplicated from QoSPolicyEnum.

    Our supported version of Python3 (3.5) doesn't have a fix that allows mixins on Enum.
    """
    @classmethod
    def short_keys(cls) -> List[str]:
        """Return a list of shortened typing-friendly enum values."""
        return [k.lower() for k in cls.__members__.keys() if not k.startswith('RMW')]

    @classmethod
    def get_from_short_key(cls, name: str) -> QoSProfile:
        """Retrieve a policy type from a short name, case-insensitive."""
        return cls[name.upper()].value


QoSCompatibility: TypeAlias = _rclpy.QoSCompatibility


def qos_check_compatible(publisher_qos: QoSProfile,
                         subscription_qos: QoSProfile) -> Tuple[QoSCompatibility, str]:
    """
    Check if two QoS profiles are compatible.

    Two QoS profiles are compatible if a publisher and subscription
    using the QoS policies can communicate with each other.

    If any policies have value "system default" or "unknown" then it is possible that
    compatibility cannot be determined.
    In this case, the value QoSCompatibility.WARNING is set as part of
    the returned structure.
    """
    result = _rclpy.rclpy_qos_check_compatible(
        publisher_qos.get_c_qos_profile(),
        subscription_qos.get_c_qos_profile()
    )
    compatibility = QoSCompatibility(
        result.compatibility
    )
    reason = result.reason
    return compatibility, reason
