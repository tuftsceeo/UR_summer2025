# Copyright 2018 Open Source Robotics Foundation, Inc.
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
from __future__ import annotations

import array
from enum import IntEnum
import sys
from typing import Any
from typing import Dict
from typing import Final
from typing import Generic
from typing import List
from typing import Literal
from typing import Optional
from typing import overload
from typing import Tuple
from typing import TYPE_CHECKING
from typing import TypeVar
from typing import Union

from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
import yaml

PARAMETER_SEPARATOR_STRING: Final = '.'

if TYPE_CHECKING:
    # Mypy does not handle string literals of array.array[int/str/float] very well
    # So if user has newer version of python can use proper array types.
    if sys.version_info > (3, 9):
        AllowableParameterValue = Union[None, bool, int, float, str,
                                        list[bytes], Tuple[bytes, ...],
                                        list[bool], Tuple[bool, ...],
                                        list[int], Tuple[int, ...], array.array[int],
                                        list[float], Tuple[float, ...], array.array[float],
                                        list[str], Tuple[str, ...], array.array[str]]
    else:
        AllowableParameterValue = Union[None, bool, int, float, str,
                                        List[bytes], Tuple[bytes, ...],
                                        List[bool], Tuple[bool, ...],
                                        List[int], Tuple[int, ...], 'array.array[int]',
                                        List[float], Tuple[float, ...], 'array.array[float]',
                                        List[str], Tuple[str, ...], 'array.array[str]']

else:
    # Done to prevent runtime errors of undefined values.
    # after python3.13 is minimum support this could be removed.
    AllowableParameterValue = Any

AllowableParameterValueT = TypeVar('AllowableParameterValueT',
                                   bound=AllowableParameterValue)


class Parameter(Generic[AllowableParameterValueT]):

    class Type(IntEnum):
        NOT_SET = ParameterType.PARAMETER_NOT_SET
        BOOL = ParameterType.PARAMETER_BOOL
        INTEGER = ParameterType.PARAMETER_INTEGER
        DOUBLE = ParameterType.PARAMETER_DOUBLE
        STRING = ParameterType.PARAMETER_STRING
        BYTE_ARRAY = ParameterType.PARAMETER_BYTE_ARRAY
        BOOL_ARRAY = ParameterType.PARAMETER_BOOL_ARRAY
        INTEGER_ARRAY = ParameterType.PARAMETER_INTEGER_ARRAY
        DOUBLE_ARRAY = ParameterType.PARAMETER_DOUBLE_ARRAY
        STRING_ARRAY = ParameterType.PARAMETER_STRING_ARRAY

        @classmethod
        def from_parameter_value(cls,
                                 parameter_value: AllowableParameterValue
                                 ) -> Parameter.Type:
            """
            Get a Parameter.Type from a given variable.

            :return: A Parameter.Type corresponding to the instance type of the given value.
            :raises: TypeError if the conversion to a type was not possible.
            """
            if parameter_value is None:
                return Parameter.Type.NOT_SET
            elif isinstance(parameter_value, bool):
                return Parameter.Type.BOOL
            elif isinstance(parameter_value, int):
                return Parameter.Type.INTEGER
            elif isinstance(parameter_value, float):
                return Parameter.Type.DOUBLE
            elif isinstance(parameter_value, str):
                return Parameter.Type.STRING
            elif isinstance(parameter_value, (list, tuple, array.array)):
                if all(isinstance(v, bytes) for v in parameter_value):
                    return Parameter.Type.BYTE_ARRAY
                elif all(isinstance(v, bool) for v in parameter_value):
                    return Parameter.Type.BOOL_ARRAY
                elif all(isinstance(v, int) for v in parameter_value):
                    return Parameter.Type.INTEGER_ARRAY
                elif all(isinstance(v, float) for v in parameter_value):
                    return Parameter.Type.DOUBLE_ARRAY
                elif all(isinstance(v, str) for v in parameter_value):
                    return Parameter.Type.STRING_ARRAY
                else:
                    raise TypeError(
                        'The given value is not a list of one of the allowed types'
                        f" '{parameter_value}'.")
            else:
                raise TypeError(
                    f"The given value is not one of the allowed types '{parameter_value}'.")

        def check(self, parameter_value: AllowableParameterValue) -> bool:
            if Parameter.Type.NOT_SET == self:
                return parameter_value is None
            if Parameter.Type.BOOL == self:
                return isinstance(parameter_value, bool)
            if Parameter.Type.INTEGER == self:
                return isinstance(parameter_value, int)
            if Parameter.Type.DOUBLE == self:
                return isinstance(parameter_value, float)
            if Parameter.Type.STRING == self:
                return isinstance(parameter_value, str)
            if Parameter.Type.BYTE_ARRAY == self:
                return isinstance(parameter_value, (list, tuple)) and \
                    all(isinstance(v, bytes) and len(v) == 1 for v in parameter_value)
            if Parameter.Type.BOOL_ARRAY == self:
                return isinstance(parameter_value, (list, tuple)) and \
                    all(isinstance(v, bool) for v in parameter_value)
            if Parameter.Type.INTEGER_ARRAY == self:
                return isinstance(parameter_value, (list, tuple, array.array)) and \
                    all(isinstance(v, int) for v in parameter_value)
            if Parameter.Type.DOUBLE_ARRAY == self:
                return isinstance(parameter_value, (list, tuple, array.array)) and \
                    all(isinstance(v, float) for v in parameter_value)
            if Parameter.Type.STRING_ARRAY == self:
                return isinstance(parameter_value, (list, tuple)) and \
                    all(isinstance(v, str) for v in parameter_value)
            return False

    @classmethod
    def from_parameter_msg(cls, param_msg: ParameterMsg) -> Parameter[Any]:
        value = None
        type_ = Parameter.Type(value=param_msg.value.type)
        if Parameter.Type.BOOL == type_:
            value = param_msg.value.bool_value
        elif Parameter.Type.INTEGER == type_:
            value = param_msg.value.integer_value
        elif Parameter.Type.DOUBLE == type_:
            value = param_msg.value.double_value
        elif Parameter.Type.STRING == type_:
            value = param_msg.value.string_value
        elif Parameter.Type.BYTE_ARRAY == type_:
            value = param_msg.value.byte_array_value
        elif Parameter.Type.BOOL_ARRAY == type_:
            value = param_msg.value.bool_array_value
        elif Parameter.Type.INTEGER_ARRAY == type_:
            value = param_msg.value.integer_array_value
        elif Parameter.Type.DOUBLE_ARRAY == type_:
            value = param_msg.value.double_array_value
        elif Parameter.Type.STRING_ARRAY == type_:
            value = param_msg.value.string_array_value
        return cls(param_msg.name, type_, value)

    @overload
    def __init__(self: Parameter[None], name: str) -> None: ...

    @overload
    def __init__(self: Parameter[None], name: str, type_: Literal[Parameter.Type.NOT_SET]
                 ) -> None: ...

    @overload
    def __init__(self: Parameter[bool], name: str, type_: Literal[Parameter.Type.BOOL]
                 ) -> None: ...

    @overload
    def __init__(self: Parameter[int], name: str, type_: Literal[Parameter.Type.INTEGER]
                 ) -> None: ...

    @overload
    def __init__(self: Parameter[float], name: str, type_: Literal[Parameter.Type.DOUBLE]
                 ) -> None: ...

    @overload
    def __init__(self: Parameter[str], name: str, type_: Literal[Parameter.Type.STRING]
                 ) -> None: ...

    @overload
    def __init__(self: Parameter[Union[list[bytes], Tuple[bytes, ...]]],
                 name: str,
                 type_: Literal[Parameter.Type.BYTE_ARRAY]) -> None: ...

    @overload
    def __init__(self: Parameter[Union[list[bool], Tuple[bool, ...]]],
                 name: str,
                 type_: Literal[Parameter.Type.BOOL_ARRAY]) -> None: ...

    @overload
    def __init__(self: Parameter[Union[list[int], Tuple[int, ...], array.array[int]]],
                 name: str,
                 type_: Literal[Parameter.Type.INTEGER_ARRAY]) -> None: ...

    @overload
    def __init__(self: Parameter[Union[list[float], Tuple[float, ...], array.array[float]]],
                 name: str,
                 type_: Literal[Parameter.Type.DOUBLE_ARRAY]) -> None: ...

    @overload
    def __init__(self: Parameter[Union[list[str], Tuple[str, ...], array.array[str]]],
                 name: str,
                 type_: Literal[Parameter.Type.STRING_ARRAY]) -> None: ...

    @overload
    def __init__(self, name: str, *, value: AllowableParameterValueT) -> None: ...

    @overload
    def __init__(self, name: str, type_: Optional[Parameter.Type] = None,
                 value: None = None) -> None: ...

    @overload
    def __init__(self, name: str, type_: Parameter.Type,
                 value: AllowableParameterValueT) -> None: ...

    def __init__(self, name: str, type_: Optional[Parameter.Type] = None, value=None) -> None:
        if type_ is None:
            # This will raise a TypeError if it is not possible to get a type from the value.
            type_ = Parameter.Type.from_parameter_value(value)

        if not isinstance(type_, Parameter.Type):
            raise TypeError("type must be an instance of '{}'".format(repr(Parameter.Type)))

        if not type_.check(value):
            raise ValueError("Type '{}' and value '{}' do not agree".format(type_, value))

        self._type_ = type_
        self._name = name
        self._value = value

    @property
    def name(self) -> str:
        return self._name

    @property
    def type_(self) -> 'Parameter.Type':
        return self._type_

    @property
    def value(self) -> AllowableParameterValueT:
        return self._value

    def get_parameter_value(self) -> ParameterValue:
        parameter_value = ParameterValue(type=self.type_.value)
        if Parameter.Type.BOOL == self.type_:
            parameter_value.bool_value = self.value
        elif Parameter.Type.INTEGER == self.type_:
            parameter_value.integer_value = self.value
        elif Parameter.Type.DOUBLE == self.type_:
            parameter_value.double_value = self.value
        elif Parameter.Type.STRING == self.type_:
            parameter_value.string_value = self.value
        elif Parameter.Type.BYTE_ARRAY == self.type_:
            parameter_value.byte_array_value = self.value
        elif Parameter.Type.BOOL_ARRAY == self.type_:
            parameter_value.bool_array_value = self.value
        elif Parameter.Type.INTEGER_ARRAY == self.type_:
            parameter_value.integer_array_value = self.value
        elif Parameter.Type.DOUBLE_ARRAY == self.type_:
            parameter_value.double_array_value = self.value
        elif Parameter.Type.STRING_ARRAY == self.type_:
            parameter_value.string_array_value = self.value
        return parameter_value

    def to_parameter_msg(self) -> ParameterMsg:
        return ParameterMsg(name=self.name, value=self.get_parameter_value())


def get_parameter_value(string_value: str) -> ParameterValue:
    """
    Guess the desired type of the parameter based on the string value.

    :param string_value: The string value to be converted to a ParameterValue.
    :return: The ParameterValue.
    """
    value = ParameterValue()
    try:
        yaml_value = yaml.safe_load(string_value)
    except yaml.parser.ParserError:
        yaml_value = string_value

    if isinstance(yaml_value, bool):
        value.type = ParameterType.PARAMETER_BOOL
        value.bool_value = yaml_value
    elif isinstance(yaml_value, int):
        value.type = ParameterType.PARAMETER_INTEGER
        value.integer_value = yaml_value
    elif isinstance(yaml_value, float):
        value.type = ParameterType.PARAMETER_DOUBLE
        value.double_value = yaml_value
    elif isinstance(yaml_value, list):
        if all((isinstance(v, bool) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_BOOL_ARRAY
            value.bool_array_value = yaml_value
        elif all((isinstance(v, int) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_INTEGER_ARRAY
            value.integer_array_value = yaml_value
        elif all((isinstance(v, float) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            value.double_array_value = yaml_value
        elif all((isinstance(v, str) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_STRING_ARRAY
            value.string_array_value = yaml_value
        else:
            value.type = ParameterType.PARAMETER_STRING
            value.string_value = string_value
    else:
        value.type = ParameterType.PARAMETER_STRING
        value.string_value = yaml_value if yaml_value is not None else string_value
    return value


def parameter_value_to_python(parameter_value: ParameterValue) -> AllowableParameterValue:
    """
    Get the value for the Python builtin type from a rcl_interfaces/msg/ParameterValue object.

    Returns the value member of the message based on the ``type`` member.
    Returns ``None`` if the parameter is "NOT_SET".

    :param parameter_value: The message to get the value from.
    :raises RuntimeError: if the member ``type`` has an unexpected value.
    """
    if parameter_value.type == ParameterType.PARAMETER_BOOL:
        value = parameter_value.bool_value
    elif parameter_value.type == ParameterType.PARAMETER_INTEGER:
        value = parameter_value.integer_value
    elif parameter_value.type == ParameterType.PARAMETER_DOUBLE:
        value = parameter_value.double_value
    elif parameter_value.type == ParameterType.PARAMETER_STRING:
        value = parameter_value.string_value
    elif parameter_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
        value = list(parameter_value.byte_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
        value = list(parameter_value.bool_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        value = list(parameter_value.integer_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        value = list(parameter_value.double_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_STRING_ARRAY:
        value = list(parameter_value.string_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_NOT_SET:
        value = None
    else:
        raise RuntimeError(f'unexpected parameter type {parameter_value.type}')

    return value


def parameter_dict_from_yaml_file(
    parameter_file: str,
    use_wildcard: bool = False,
    target_nodes: Optional[List[str]] = None,
    namespace: str = ''
) -> Dict[str, ParameterMsg]:
    """
    Build a dict of parameters from a YAML file.

    Will load all parameters if ``target_nodes`` is None or empty.

    :raises RuntimeError: if a target node is not in the file
    :raises RuntimeError: if the is not a valid ROS parameter file

    :param parameter_file: Path to the YAML file to load parameters from.
    :param use_wildcard: Use wildcard matching for the target nodes.
    :param target_nodes: List of nodes in the YAML file to load parameters from.
    :param namespace: Namespace to prepend to all parameters.
    :return: A dict of Parameter messages keyed by the parameter names
    """
    with open(parameter_file, 'r') as f:
        param_file = yaml.safe_load(f)
        param_dict = {}

        # check and add if wildcard is available in 1st keys
        # wildcard key must go to the front of param_keys so that
        # node-namespaced parameters will override the wildcard parameters
        if use_wildcard and '/**' in param_file.keys():
            value = param_file['/**']
            if not isinstance(value, dict) and 'ros__parameters' not in value:
                raise RuntimeError(
                    'YAML file is not a valid ROS parameter file for wildcard(/**)')
            param_dict.update(value['ros__parameters'])

        # parse parameter yaml file based on target node namespace and name
        if target_nodes:
            for n in target_nodes:
                abs_name = _get_absolute_node_name(n)
                if abs_name is None:
                    continue
                if abs_name in param_file.keys():
                    # found absolute node name w or w/o namespace
                    value = param_file[abs_name]
                    if not isinstance(value, dict) and 'ros__parameters' not in value:
                        raise RuntimeError(
                            f'YAML file is not a valid ROS parameter file for node {abs_name}')
                    param_dict.update(value['ros__parameters'])
                ns, node_basename = abs_name.rsplit('/', 1)
                if not ns and node_basename in param_file.keys():
                    # found non-absolute node name without namespace
                    value = param_file[node_basename]
                    if not isinstance(value, dict) and 'ros__parameters' not in value:
                        raise RuntimeError('YAML file is not a valid ROS parameter '
                                           f'file for node {node_basename}')
                    param_dict.update(value['ros__parameters'])
                elif ns in param_file.keys():
                    # found namespace
                    if node_basename in param_file[ns].keys():
                        value = param_file[ns][node_basename]
                        if not isinstance(value, dict) and 'ros__parameters' not in value:
                            raise RuntimeError('YAML file is not a valid ROS parameter '
                                               f'file for namespace {ns} node {node_basename}')
                        param_dict.update(value['ros__parameters'])

        if not param_dict:
            raise RuntimeError('Param file does not contain any valid parameters')

        return _unpack_parameter_dict(namespace, param_dict)


def _get_absolute_node_name(node_name: str) -> Optional[str]:
    if not node_name:
        return None
    if node_name[0] != '/':
        node_name = '/' + node_name
    return node_name


def _unpack_parameter_dict(namespace: str,
                           parameter_dict: Dict[str, ParameterMsg]) -> Dict[str, ParameterMsg]:
    """
    Flatten a parameter dictionary recursively.

    :param namespace: The namespace to prepend to the parameter names.
    :param parameter_dict: A dictionary of parameters keyed by the parameter names
    :return: A dict of Parameter objects keyed by the parameter names
    """
    parameters: Dict[str, ParameterMsg] = {}
    for param_name, param_value in parameter_dict.items():
        full_param_name = namespace + param_name
        # Unroll nested parameters
        if isinstance(param_value, dict):
            parameters.update(_unpack_parameter_dict(
                    namespace=full_param_name + PARAMETER_SEPARATOR_STRING,
                    parameter_dict=param_value))
        else:
            parameter = ParameterMsg()
            parameter.name = full_param_name
            parameter.value = get_parameter_value(str(param_value))
            parameters[full_param_name] = parameter
    return parameters
