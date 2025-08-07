# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:msg/HeadState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HeadState(type):
    """Metaclass of message 'HeadState'."""

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
                'baxter_core_msgs.msg.HeadState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__head_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__head_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__head_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__head_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__head_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class HeadState(metaclass=Metaclass_HeadState):
    """Message class 'HeadState'."""

    __slots__ = [
        '_pan',
        '_is_turning',
        '_is_nodding',
        '_is_pan_enabled',
    ]

    _fields_and_field_types = {
        'pan': 'float',
        'is_turning': 'boolean',
        'is_nodding': 'boolean',
        'is_pan_enabled': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.pan = kwargs.get('pan', float())
        self.is_turning = kwargs.get('is_turning', bool())
        self.is_nodding = kwargs.get('is_nodding', bool())
        self.is_pan_enabled = kwargs.get('is_pan_enabled', bool())

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
        if self.pan != other.pan:
            return False
        if self.is_turning != other.is_turning:
            return False
        if self.is_nodding != other.is_nodding:
            return False
        if self.is_pan_enabled != other.is_pan_enabled:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pan(self):
        """Message field 'pan'."""
        return self._pan

    @pan.setter
    def pan(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pan' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pan' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pan = value

    @builtins.property
    def is_turning(self):
        """Message field 'is_turning'."""
        return self._is_turning

    @is_turning.setter
    def is_turning(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_turning' field must be of type 'bool'"
        self._is_turning = value

    @builtins.property
    def is_nodding(self):
        """Message field 'is_nodding'."""
        return self._is_nodding

    @is_nodding.setter
    def is_nodding(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_nodding' field must be of type 'bool'"
        self._is_nodding = value

    @builtins.property
    def is_pan_enabled(self):
        """Message field 'is_pan_enabled'."""
        return self._is_pan_enabled

    @is_pan_enabled.setter
    def is_pan_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_pan_enabled' field must be of type 'bool'"
        self._is_pan_enabled = value
