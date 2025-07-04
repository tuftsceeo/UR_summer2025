# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import inspect
import os
from types import FrameType
from typing import cast
from typing import ClassVar
from typing import Dict
from typing import List
from typing import Literal
from typing import NamedTuple
from typing import Optional
from typing import OrderedDict
from typing import Tuple
from typing import Type
from typing import TypedDict
from typing import Union

from rclpy.clock import Clock
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from typing_extensions import Unpack


SupportedFiltersKeys = Literal['throttle', 'skip_first', 'once']

# Known filenames from which logging methods can be called (will be ignored in `_find_caller`).
_internal_callers: List[str] = []
# This will cause rclpy filenames to be registered in `_internal_callers` on first logging call.
_populate_internal_callers = True


def _find_caller(frame: Optional[FrameType]) -> FrameType:
    """Get the first calling frame that is outside of rclpy."""
    global _populate_internal_callers
    global _internal_callers
    if _populate_internal_callers:
        # Populate the list of internal filenames from which logging methods can be called.
        # This has to be done from within a function to avoid cyclic module imports.
        import rclpy.logging
        # Extend the list to preserve any filenames that may have been added by third parties.
        # Note: the call to `realpath` will also resolve mixed slashes that can result on Windows.
        _internal_callers.extend([
            os.path.realpath(__file__),
            os.path.realpath(rclpy.logging.__file__),
        ])
        _populate_internal_callers = False

    if frame is None:
        raise ValueError('Cannot get frame info if frame is None')

    file_path = os.path.realpath(inspect.getframeinfo(frame).filename)
    while any(f in file_path for f in _internal_callers):
        frame = frame.f_back

        if frame is None:
            raise ValueError('Cannot get frame info if frame is None')

        file_path = os.path.realpath(inspect.getframeinfo(frame).filename)
    return frame


class CallerId(
        NamedTuple('CallerId',
                   [('function_name', str),
                    ('file_path', str),
                    ('line_number', int),
                    ('last_index', int)])):

    def __new__(cls, frame: Optional[FrameType] = None) -> 'CallerId':
        if not frame:
            frame = _find_caller(inspect.currentframe())
        return super(CallerId, cls).__new__(
            cls,
            function_name=frame.f_code.co_name,
            file_path=os.path.abspath(inspect.getframeinfo(frame).filename),
            line_number=frame.f_lineno,
            last_index=frame.f_lasti,  # To distinguish between two callers on the same line
        )


class RcutilsLoggerContext(TypedDict):
    name: str
    severity: LoggingSeverity
    filters: List[SupportedFiltersKeys]


class OnceContext(RcutilsLoggerContext):
    has_been_logged_once: bool


class ThrottleContext(RcutilsLoggerContext):
    throttle_duration_sec: float
    throttle_time_source_type: Clock
    throttle_last_logged: int


class SkipFirstContext(RcutilsLoggerContext):
    first_has_been_skipped: bool


class LoggingFilterArgs(TypedDict, total=False):
    once: bool
    throttle_duration_sec: float
    throttle_time_source_type: Clock
    skip_first: bool


class LoggingFilterParams(TypedDict, total=False):
    once: Optional[bool]
    throttle_duration_sec: Optional[float]
    throttle_time_source_type: Clock
    skip_first: Optional[bool]


class LoggingArgs(LoggingFilterArgs, total=False):
    name: str


class LoggingFilter:
    """Base class for logging filters."""

    """
    Parameters of a filter and their default value, if appropriate.

    A default value of None makes a parameter required.
    """
    params: ClassVar[LoggingFilterParams] = {}

    """
    Initialize the context of a logging call, e.g. declare variables needed for
    determining the log condition and add them to the context.
    """
    @classmethod
    def initialize_context(cls, context: RcutilsLoggerContext,
                           **kwargs: 'Unpack[LoggingFilterArgs]') -> None:
        # Store all parameters in the context so we can check that users never try to change them.
        for param_name, param_value in cls.params.items():
            kwargs_value = kwargs.get(param_name, param_value)
            if kwargs_value is None:
                raise TypeError(
                    'Required parameter "{0}" was not specified for logging filter "{1}"'
                    .format(param_name, cls.__name__))
            context[param_name] = kwargs_value  # type: ignore[literal-required]

    """
    Decide if it's appropriate to log given a context, and update the context accordingly.
    """
    @staticmethod
    def should_log(context: RcutilsLoggerContext) -> bool:
        return True


class Once(LoggingFilter):
    """Ignore all log calls except the first one."""

    params: ClassVar[LoggingFilterParams] = {
        'once': None,
    }

    @classmethod
    def initialize_context(cls, context: RcutilsLoggerContext,
                           **kwargs: 'Unpack[LoggingFilterArgs]') -> None:
        context = cast(OnceContext, context)
        super(Once, cls).initialize_context(context, **kwargs)
        context['has_been_logged_once'] = False

    @staticmethod
    def should_log(context: RcutilsLoggerContext) -> bool:
        context = cast(OnceContext, context)
        logging_condition = False
        if not context['has_been_logged_once']:
            logging_condition = True
            context['has_been_logged_once'] = True
        return logging_condition


class Throttle(LoggingFilter):
    """Ignore log calls if the last call is not longer ago than the specified duration."""

    params: ClassVar[LoggingFilterParams] = {
        'throttle_duration_sec': None,
        'throttle_time_source_type': Clock(),
    }

    @classmethod
    def initialize_context(cls, context: RcutilsLoggerContext,
                           **kwargs: 'Unpack[LoggingFilterArgs]') -> None:
        context = cast(ThrottleContext, context)
        super(Throttle, cls).initialize_context(context, **kwargs)
        context['throttle_last_logged'] = 0
        if not isinstance(context['throttle_time_source_type'], Clock):
            raise ValueError(
                'Received throttle_time_source_type of "{0}" '
                'is not a clock instance'
                .format(context['throttle_time_source_type']))

    @staticmethod
    def should_log(context: RcutilsLoggerContext) -> bool:
        context = cast(ThrottleContext, context)
        logging_condition = True
        now = context['throttle_time_source_type'].now().nanoseconds
        next_log_time = context['throttle_last_logged'] + (context['throttle_duration_sec'] * 1e+9)
        logging_condition = now >= next_log_time
        if logging_condition:
            context['throttle_last_logged'] = now
        return logging_condition


class SkipFirst(LoggingFilter):
    """Ignore the first log call but process all subsequent calls."""

    params: ClassVar[LoggingFilterParams] = {
        'skip_first': None,
    }

    @classmethod
    def initialize_context(cls, context: RcutilsLoggerContext,
                           **kwargs: 'Unpack[LoggingFilterArgs]') -> None:
        context = cast(SkipFirstContext, context)
        super(SkipFirst, cls).initialize_context(context, **kwargs)
        context['first_has_been_skipped'] = False

    @staticmethod
    def should_log(context: RcutilsLoggerContext) -> bool:
        context = cast(SkipFirstContext, context)
        logging_condition = True
        if not context['first_has_been_skipped']:
            logging_condition = False
            context['first_has_been_skipped'] = True
        return logging_condition


# The ordering of this dictionary defines the order in which filters will be processed.
supported_filters: OrderedDict[SupportedFiltersKeys, Type[LoggingFilter]] = OrderedDict()
supported_filters['throttle'] = Throttle
supported_filters['skip_first'] = SkipFirst
supported_filters['once'] = Once


def get_filters_from_kwargs(**kwargs: 'Unpack[LoggingFilterArgs]') -> List[SupportedFiltersKeys]:
    """
    Determine which filters have had parameters specified in the given keyword arguments.

    Returns the list of filters using the order specified by `supported_filters`.
    """
    detected_filters: List[SupportedFiltersKeys] = []
    all_supported_params: List[str] = []
    for supported_filter, filter_class in supported_filters.items():
        filter_params = filter_class.params.keys()
        all_supported_params.extend(filter_params)
        if any(kwargs.get(param_name) for param_name in filter_params):
            detected_filters.append(supported_filter)
    # Check that all required parameters (with no default value) have been specified
    for detected_filter in detected_filters:
        for param_name, default_value in supported_filters[detected_filter].params.items():
            if param_name in kwargs:
                continue

            # Param not specified; use the default.
            if default_value is None:
                raise TypeError(
                    'required parameter "{0}" not specified '
                    'but is required for the the logging filter "{1}"'.format(
                        param_name, detected_filter))
            kwargs[param_name] = default_value  # type: ignore
    for kwarg in kwargs:
        if kwarg not in all_supported_params:
            raise TypeError(
                'parameter "{0}" is not one of the recognized logging options "{1}"'
                .format(kwarg, all_supported_params)
            )
    return detected_filters


class RcutilsLogger:

    def __init__(self, name: str = '') -> None:
        self.name = name
        self.logger_sublogger_namepair: Optional[Tuple[str, str]] = None
        self.contexts: Dict[CallerId, RcutilsLoggerContext] = {}

    def __del__(self) -> None:
        if self.logger_sublogger_namepair:
            _rclpy.rclpy_logging_rosout_remove_sublogger(
                self.logger_sublogger_namepair[0], self.logger_sublogger_namepair[1])
            self.logger_sublogger_namepair = None

    def get_child(self, name: str) -> 'RcutilsLogger':
        if not name:
            raise ValueError('Child logger name must not be empty.')

        if self.name:
            # Prepend the name of this logger
            fullname = self.name + _rclpy.rclpy_logging_get_separator_string() + name
        else:
            fullname = name

        logger = RcutilsLogger(name=fullname)
        if self.name and _rclpy.rclpy_logging_rosout_add_sublogger(self.name, name):
            logger.logger_sublogger_namepair = (self.name, name)
        return logger

    def set_level(self, level: Union[int, LoggingSeverity]) -> None:
        level = LoggingSeverity(level)
        _rclpy.rclpy_logging_set_logger_level(self.name, level)

    def get_effective_level(self) -> LoggingSeverity:
        level = LoggingSeverity(
            _rclpy.rclpy_logging_get_logger_effective_level(self.name))
        return level

    def is_enabled_for(self, severity: Union[int, LoggingSeverity]) -> bool:
        severity = LoggingSeverity(severity)
        result: bool = _rclpy.rclpy_logging_logger_is_enabled_for(self.name, severity)
        return result

    def log(self, message: str, severity: Union[int, LoggingSeverity],
            name: Optional[str] = None, **kwargs: 'Unpack[LoggingFilterArgs]') -> bool:
        r"""
        Log a message with the specified severity.

        The message will not be logged if:
          * the logger is not enabled for the message's severity (the message severity is less than
            the level of the logger), or
          * a logging filter causes the message to be skipped.

        .. note::
           Logging filters will only be evaluated if the logger is enabled for the message's
           severity.

        :param message: message to log.
        :param severity: severity of the message.
        :keyword name: name of the logger to use.
        :param \**kwargs: optional parameters for logging filters (see below).

        :Keyword Arguments:
            * *throttle_duration_sec* (``float``) --
              Duration of the throttle interval for the :py:class:Throttle: filter.
            * *throttle_time_source_type* (``str``) --
              Optional time source type for the :py:class:Throttle: filter (default of
              ``RCUTILS_STEADY_TIME``)
            * *skip_first* (``bool``) --
              If True, enable the :py:class:SkipFirst: filter.
            * *once* (``bool``) --
              If True, enable the :py:class:Once: filter.
        :returns: False if a filter caused the message to not be logged; True otherwise.
        :raises: TypeError on invalid filter parameter combinations.
        :raises: ValueError on invalid parameters values.
        """
        # Gather context info and check filters only if the severity is appropriate.
        if not self.is_enabled_for(severity):
            return False

        severity = LoggingSeverity(severity)

        if name is None:
            name = self.name

        # Infer the requested log filters from the keyword arguments
        detected_filters = get_filters_from_kwargs(**kwargs)

        # Get/prepare the context corresponding to the caller.
        caller_id = CallerId()
        if caller_id not in self.contexts:
            context: RcutilsLoggerContext = {'name': name, 'severity': severity,
                                             'filters': detected_filters}
            for detected_filter in detected_filters:
                if detected_filter in supported_filters:
                    supported_filters[detected_filter].initialize_context(context, **kwargs)
            self.contexts[caller_id] = context
        else:
            context = self.contexts[caller_id]
            # Don't support any changes to the logger.
            if severity != context['severity']:
                raise ValueError('Logger severity cannot be changed between calls.')
            if name != context['name']:
                raise ValueError('Logger name cannot be changed between calls.')
            if detected_filters != context['filters']:
                raise ValueError('Requested logging filters cannot be changed between calls.')
            for detected_filter in detected_filters:
                filter_params = supported_filters[detected_filter].params
                if any(
                    context[p] != kwargs.get(p, filter_params[p])  # type: ignore
                        for p in filter_params):
                    raise ValueError(
                        'Logging filter parameters cannot be changed between calls.')

        # Check if any filter determines the message shouldn't be processed.
        # Note(dhood): even if a message doesn't get logged, a filter might still update its state
        # as if it had been. This matches the behavior of the C logging macros provided by rcutils.
        for logging_filter in context['filters']:
            if not supported_filters[logging_filter].should_log(context):
                return False

        # Call the relevant function from the C extension.
        _rclpy.rclpy_logging_rcutils_log(
            severity, name, message,
            caller_id.function_name, caller_id.file_path, caller_id.line_number)
        return True

    def debug(self, message: str, **kwargs: 'Unpack[LoggingArgs]') -> bool:
        """Log a message with `DEBUG` severity via :py:classmethod:RcutilsLogger.log:."""
        return self.log(message, LoggingSeverity.DEBUG, **kwargs)

    def info(self, message: str, **kwargs: 'Unpack[LoggingArgs]') -> bool:
        """Log a message with `INFO` severity via :py:classmethod:RcutilsLogger.log:."""
        return self.log(message, LoggingSeverity.INFO, **kwargs)

    def warning(self, message: str, **kwargs: 'Unpack[LoggingArgs]') -> bool:
        """Log a message with `WARN` severity via :py:classmethod:RcutilsLogger.log:."""
        return self.log(message, LoggingSeverity.WARN, **kwargs)

    def error(self, message: str, **kwargs: 'Unpack[LoggingArgs]') -> bool:
        """Log a message with `ERROR` severity via :py:classmethod:RcutilsLogger.log:."""
        return self.log(message, LoggingSeverity.ERROR, **kwargs)

    def fatal(self, message: str, **kwargs: 'Unpack[LoggingArgs]') -> bool:
        """Log a message with `FATAL` severity via :py:classmethod:RcutilsLogger.log:."""
        return self.log(message, LoggingSeverity.FATAL, **kwargs)
