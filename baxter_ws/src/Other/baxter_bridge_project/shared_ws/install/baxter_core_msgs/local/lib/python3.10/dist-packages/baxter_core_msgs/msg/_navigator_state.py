# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:msg/NavigatorState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NavigatorState(type):
    """Metaclass of message 'NavigatorState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
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
                'baxter_core_msgs.msg.NavigatorState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__navigator_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__navigator_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__navigator_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__navigator_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__navigator_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigatorState(metaclass=Metaclass_NavigatorState):
    """Message class 'NavigatorState'."""

    __slots__ = [
        '_button_names',
        '_buttons',
        '_wheel',
        '_light_names',
        '_lights',
    ]

    _fields_and_field_types = {
        'button_names': 'sequence<string>',
        'buttons': 'sequence<boolean>',
        'wheel': 'uint8',
        'light_names': 'sequence<string>',
        'lights': 'sequence<boolean>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.button_names = kwargs.get('button_names', [])
        self.buttons = kwargs.get('buttons', [])
        self.wheel = kwargs.get('wheel', int())
        self.light_names = kwargs.get('light_names', [])
        self.lights = kwargs.get('lights', [])

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
        if self.button_names != other.button_names:
            return False
        if self.buttons != other.buttons:
            return False
        if self.wheel != other.wheel:
            return False
        if self.light_names != other.light_names:
            return False
        if self.lights != other.lights:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def button_names(self):
        """Message field 'button_names'."""
        return self._button_names

    @button_names.setter
    def button_names(self, value):
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
                "The 'button_names' field must be a set or sequence and each value of type 'str'"
        self._button_names = value

    @builtins.property
    def buttons(self):
        """Message field 'buttons'."""
        return self._buttons

    @buttons.setter
    def buttons(self, value):
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
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'buttons' field must be a set or sequence and each value of type 'bool'"
        self._buttons = value

    @builtins.property
    def wheel(self):
        """Message field 'wheel'."""
        return self._wheel

    @wheel.setter
    def wheel(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'wheel' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'wheel' field must be an unsigned integer in [0, 255]"
        self._wheel = value

    @builtins.property
    def light_names(self):
        """Message field 'light_names'."""
        return self._light_names

    @light_names.setter
    def light_names(self, value):
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
                "The 'light_names' field must be a set or sequence and each value of type 'str'"
        self._light_names = value

    @builtins.property
    def lights(self):
        """Message field 'lights'."""
        return self._lights

    @lights.setter
    def lights(self, value):
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
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'lights' field must be a set or sequence and each value of type 'bool'"
        self._lights = value
