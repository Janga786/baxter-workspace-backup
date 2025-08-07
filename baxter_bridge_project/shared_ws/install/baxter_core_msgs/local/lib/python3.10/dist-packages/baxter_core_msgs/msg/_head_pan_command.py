# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:msg/HeadPanCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HeadPanCommand(type):
    """Metaclass of message 'HeadPanCommand'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MAX_SPEED_RATIO': 1.0,
        'MIN_SPEED_RATIO': 0.0,
        'REQUEST_PAN_DISABLE': 0,
        'REQUEST_PAN_ENABLE': 1,
        'REQUEST_PAN_VOID': 2,
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
                'baxter_core_msgs.msg.HeadPanCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__head_pan_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__head_pan_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__head_pan_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__head_pan_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__head_pan_command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MAX_SPEED_RATIO': cls.__constants['MAX_SPEED_RATIO'],
            'MIN_SPEED_RATIO': cls.__constants['MIN_SPEED_RATIO'],
            'REQUEST_PAN_DISABLE': cls.__constants['REQUEST_PAN_DISABLE'],
            'REQUEST_PAN_ENABLE': cls.__constants['REQUEST_PAN_ENABLE'],
            'REQUEST_PAN_VOID': cls.__constants['REQUEST_PAN_VOID'],
        }

    @property
    def MAX_SPEED_RATIO(self):
        """Message constant 'MAX_SPEED_RATIO'."""
        return Metaclass_HeadPanCommand.__constants['MAX_SPEED_RATIO']

    @property
    def MIN_SPEED_RATIO(self):
        """Message constant 'MIN_SPEED_RATIO'."""
        return Metaclass_HeadPanCommand.__constants['MIN_SPEED_RATIO']

    @property
    def REQUEST_PAN_DISABLE(self):
        """Message constant 'REQUEST_PAN_DISABLE'."""
        return Metaclass_HeadPanCommand.__constants['REQUEST_PAN_DISABLE']

    @property
    def REQUEST_PAN_ENABLE(self):
        """Message constant 'REQUEST_PAN_ENABLE'."""
        return Metaclass_HeadPanCommand.__constants['REQUEST_PAN_ENABLE']

    @property
    def REQUEST_PAN_VOID(self):
        """Message constant 'REQUEST_PAN_VOID'."""
        return Metaclass_HeadPanCommand.__constants['REQUEST_PAN_VOID']


class HeadPanCommand(metaclass=Metaclass_HeadPanCommand):
    """
    Message class 'HeadPanCommand'.

    Constants:
      MAX_SPEED_RATIO
      MIN_SPEED_RATIO
      REQUEST_PAN_DISABLE
      REQUEST_PAN_ENABLE
      REQUEST_PAN_VOID
    """

    __slots__ = [
        '_target',
        '_speed_ratio',
        '_enable_pan_request',
    ]

    _fields_and_field_types = {
        'target': 'float',
        'speed_ratio': 'float',
        'enable_pan_request': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.target = kwargs.get('target', float())
        self.speed_ratio = kwargs.get('speed_ratio', float())
        self.enable_pan_request = kwargs.get('enable_pan_request', int())

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
        if self.target != other.target:
            return False
        if self.speed_ratio != other.speed_ratio:
            return False
        if self.enable_pan_request != other.enable_pan_request:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def target(self):
        """Message field 'target'."""
        return self._target

    @target.setter
    def target(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target = value

    @builtins.property
    def speed_ratio(self):
        """Message field 'speed_ratio'."""
        return self._speed_ratio

    @speed_ratio.setter
    def speed_ratio(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_ratio' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speed_ratio' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speed_ratio = value

    @builtins.property
    def enable_pan_request(self):
        """Message field 'enable_pan_request'."""
        return self._enable_pan_request

    @enable_pan_request.setter
    def enable_pan_request(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enable_pan_request' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'enable_pan_request' field must be an unsigned integer in [0, 255]"
        self._enable_pan_request = value
