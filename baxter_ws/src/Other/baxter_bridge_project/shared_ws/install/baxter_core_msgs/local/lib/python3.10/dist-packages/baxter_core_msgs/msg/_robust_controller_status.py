# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:msg/RobustControllerStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobustControllerStatus(type):
    """Metaclass of message 'RobustControllerStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NOT_COMPLETE': 0,
        'COMPLETE_W_FAILURE': 1,
        'COMPLETE_W_SUCCESS': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('baxter_core_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'baxter_core_msgs.msg.RobustControllerStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robust_controller_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robust_controller_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robust_controller_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robust_controller_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robust_controller_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NOT_COMPLETE': cls.__constants['NOT_COMPLETE'],
            'COMPLETE_W_FAILURE': cls.__constants['COMPLETE_W_FAILURE'],
            'COMPLETE_W_SUCCESS': cls.__constants['COMPLETE_W_SUCCESS'],
        }

    @property
    def NOT_COMPLETE(self):
        """Message constant 'NOT_COMPLETE'."""
        return Metaclass_RobustControllerStatus.__constants['NOT_COMPLETE']

    @property
    def COMPLETE_W_FAILURE(self):
        """Message constant 'COMPLETE_W_FAILURE'."""
        return Metaclass_RobustControllerStatus.__constants['COMPLETE_W_FAILURE']

    @property
    def COMPLETE_W_SUCCESS(self):
        """Message constant 'COMPLETE_W_SUCCESS'."""
        return Metaclass_RobustControllerStatus.__constants['COMPLETE_W_SUCCESS']


class RobustControllerStatus(metaclass=Metaclass_RobustControllerStatus):
    """
    Message class 'RobustControllerStatus'.

    Constants:
      NOT_COMPLETE
      COMPLETE_W_FAILURE
      COMPLETE_W_SUCCESS
    """

    __slots__ = [
        '_is_enabled',
        '_complete',
        '_control_uid',
        '_timed_out',
        '_error_codes',
        '_labels',
    ]

    _fields_and_field_types = {
        'is_enabled': 'boolean',
        'complete': 'int32',
        'control_uid': 'string',
        'timed_out': 'boolean',
        'error_codes': 'sequence<string>',
        'labels': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.is_enabled = kwargs.get('is_enabled', bool())
        self.complete = kwargs.get('complete', int())
        self.control_uid = kwargs.get('control_uid', str())
        self.timed_out = kwargs.get('timed_out', bool())
        self.error_codes = kwargs.get('error_codes', [])
        self.labels = kwargs.get('labels', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.is_enabled != other.is_enabled:
            return False
        if self.complete != other.complete:
            return False
        if self.control_uid != other.control_uid:
            return False
        if self.timed_out != other.timed_out:
            return False
        if self.error_codes != other.error_codes:
            return False
        if self.labels != other.labels:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def is_enabled(self):
        """Message field 'is_enabled'."""
        return self._is_enabled

    @is_enabled.setter
    def is_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_enabled' field must be of type 'bool'"
        self._is_enabled = value

    @builtins.property
    def complete(self):
        """Message field 'complete'."""
        return self._complete

    @complete.setter
    def complete(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'complete' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'complete' field must be an integer in [-2147483648, 2147483647]"
        self._complete = value

    @builtins.property
    def control_uid(self):
        """Message field 'control_uid'."""
        return self._control_uid

    @control_uid.setter
    def control_uid(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'control_uid' field must be of type 'str'"
        self._control_uid = value

    @builtins.property
    def timed_out(self):
        """Message field 'timed_out'."""
        return self._timed_out

    @timed_out.setter
    def timed_out(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'timed_out' field must be of type 'bool'"
        self._timed_out = value

    @builtins.property
    def error_codes(self):
        """Message field 'error_codes'."""
        return self._error_codes

    @error_codes.setter
    def error_codes(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'error_codes' field must be a set or sequence and each value of type 'str'"
        self._error_codes = value

    @builtins.property
    def labels(self):
        """Message field 'labels'."""
        return self._labels

    @labels.setter
    def labels(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'labels' field must be a set or sequence and each value of type 'str'"
        self._labels = value
