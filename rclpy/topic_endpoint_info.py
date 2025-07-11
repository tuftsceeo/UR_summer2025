# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

from enum import IntEnum
from typing import Any, List, Union

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSHistoryPolicy, QoSPresetProfiles, QoSProfile
from rclpy.type_hash import TypeHash, TypeHashDictionary


class TopicEndpointTypeEnum(IntEnum):
    """
    Enum for possible types of topic endpoints.

    This enum matches the one defined in rmw/types.h
    """

    INVALID = 0
    PUBLISHER = 1
    SUBSCRIPTION = 2


class TopicEndpointInfo:
    """Information on a topic endpoint."""

    __slots__ = [
        '_node_name',
        '_node_namespace',
        '_topic_type',
        '_topic_type_hash',
        '_endpoint_type',
        '_endpoint_gid',
        '_qos_profile'
    ]

    def __init__(
        self,
        node_name: str = '',
        node_namespace: str = '',
        topic_type: str = '',
        topic_type_hash: Union[TypeHash, TypeHashDictionary] = TypeHash(),
        endpoint_type: Union[TopicEndpointTypeEnum, int] = TopicEndpointTypeEnum.INVALID,
        endpoint_gid: List[int] = [],
        qos_profile: Union[QoSProfile, '_rclpy._rmw_qos_profile_dict'] =
            QoSPresetProfiles.UNKNOWN.value
    ):
        self.node_name = node_name
        self.node_namespace = node_namespace
        self.topic_type = topic_type
        self.topic_type_hash = topic_type_hash
        self.endpoint_type = endpoint_type
        self.endpoint_gid = endpoint_gid
        self.qos_profile = qos_profile

    @property
    def node_name(self) -> str:
        """
        Get field 'node_name'.

        :returns: node_name attribute
        """
        return self._node_name

    @node_name.setter
    def node_name(self, value: str) -> None:
        assert isinstance(value, str)
        self._node_name = value

    @property
    def node_namespace(self) -> str:
        """
        Get field 'node_namespace'.

        :returns: node_namespace attribute
        """
        return self._node_namespace

    @node_namespace.setter
    def node_namespace(self, value: str) -> None:
        assert isinstance(value, str)
        self._node_namespace = value

    @property
    def topic_type(self) -> str:
        """
        Get field 'topic_type'.

        :returns: topic_type attribute
        """
        return self._topic_type

    @topic_type.setter
    def topic_type(self, value: str) -> None:
        assert isinstance(value, str)
        self._topic_type = value

    # Has to be marked Any due to mypy#3004. Return type is actually TypeHash
    @property
    def topic_type_hash(self) -> Any:
        """
        Get field 'topic_type_hash'.

        :returns: topic_type_hash attribute
        """
        return self._topic_type_hash

    @topic_type_hash.setter
    def topic_type_hash(self, value: Union[TypeHash, TypeHashDictionary]) -> None:
        if isinstance(value, TypeHash):
            self._topic_type_hash = value
        elif isinstance(value, dict):
            self._topic_type_hash = TypeHash(**value)
        else:
            assert False

    # Has to be marked Any due to mypy#3004. Return type is actually TopicEndpointTypeEnum
    @property
    def endpoint_type(self) -> Any:
        """
        Get field 'endpoint_type'.

        :returns: endpoint_type attribute
        """
        return self._endpoint_type

    @endpoint_type.setter
    def endpoint_type(self, value: Union[TopicEndpointTypeEnum, int]) -> None:
        if isinstance(value, TopicEndpointTypeEnum):
            self._endpoint_type = value
        elif isinstance(value, int):
            self._endpoint_type = TopicEndpointTypeEnum(value)
        else:
            assert False

    @property
    def endpoint_gid(self) -> List[int]:
        """
        Get field 'endpoint_gid'.

        :returns: endpoint_gid attribute
        """
        return self._endpoint_gid

    @endpoint_gid.setter
    def endpoint_gid(self, value: List[int]) -> None:
        assert all(isinstance(x, int) for x in value)
        self._endpoint_gid = value

    # Has to be marked Any due to mypy#3004. Return type is actually QoSProfile
    @property
    def qos_profile(self) -> Any:
        """
        Get field 'qos_profile'.

        :returns: qos_profile attribute
        """
        return self._qos_profile

    @qos_profile.setter
    def qos_profile(self, value: Union[QoSProfile, '_rclpy._rmw_qos_profile_dict']) -> None:
        if isinstance(value, QoSProfile):
            self._qos_profile = value
        elif isinstance(value, dict):
            self._qos_profile = QoSProfile(**value)
        else:
            assert False

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, TopicEndpointInfo):
            return False
        return all(
            self.__getattribute__(slot) == other.__getattribute__(slot)
            for slot in self.__slots__)

    def __str__(self) -> str:
        gid = '.'.join(format(x, '02x') for x in self.endpoint_gid)
        if self.qos_profile.history.value != QoSHistoryPolicy.KEEP_LAST:
            history_depth_str = self.qos_profile.history.name
        else:
            history_depth_str = f'{self.qos_profile.history.name} ({self.qos_profile.depth})'
        return '\n'.join([
            f'Node name: {self.node_name}',
            f'Node namespace: {self.node_namespace}',
            f'Topic type: {self.topic_type}',
            f'Topic type hash: {self.topic_type_hash}',
            f'Endpoint type: {self.endpoint_type.name}',
            f'GID: {gid}',
            'QoS profile:',
            f'  Reliability: {self.qos_profile.reliability.name}',
            f'  History (Depth): {history_depth_str}',
            f'  Durability: {self.qos_profile.durability.name}',
            f'  Lifespan: {self.qos_profile.lifespan}',
            f'  Deadline: {self.qos_profile.deadline}',
            f'  Liveliness: {self.qos_profile.liveliness.name}',
            f'  Liveliness lease duration: {self.qos_profile.liveliness_lease_duration}',
        ])
