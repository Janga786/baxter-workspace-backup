# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:msg/CameraControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CameraControl(type):
    """Metaclass of message 'CameraControl'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CAMERA_CONTROL_EXPOSURE': 100,
        'CAMERA_CONTROL_GAIN': 101,
        'CAMERA_CONTROL_WHITE_BALANCE_R': 102,
        'CAMERA_CONTROL_WHITE_BALANCE_G': 103,
        'CAMERA_CONTROL_WHITE_BALANCE_B': 104,
        'CAMERA_CONTROL_WINDOW_X': 105,
        'CAMERA_CONTROL_WINDOW_Y': 106,
        'CAMERA_CONTROL_FLIP': 107,
        'CAMERA_CONTROL_MIRROR': 108,
        'CAMERA_CONTROL_RESOLUTION_HALF': 109,
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
                'baxter_core_msgs.msg.CameraControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__camera_control
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__camera_control
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__camera_control
            cls._TYPE_SUPPORT = module.type_support_msg__msg__camera_control
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__camera_control

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CAMERA_CONTROL_EXPOSURE': cls.__constants['CAMERA_CONTROL_EXPOSURE'],
            'CAMERA_CONTROL_GAIN': cls.__constants['CAMERA_CONTROL_GAIN'],
            'CAMERA_CONTROL_WHITE_BALANCE_R': cls.__constants['CAMERA_CONTROL_WHITE_BALANCE_R'],
            'CAMERA_CONTROL_WHITE_BALANCE_G': cls.__constants['CAMERA_CONTROL_WHITE_BALANCE_G'],
            'CAMERA_CONTROL_WHITE_BALANCE_B': cls.__constants['CAMERA_CONTROL_WHITE_BALANCE_B'],
            'CAMERA_CONTROL_WINDOW_X': cls.__constants['CAMERA_CONTROL_WINDOW_X'],
            'CAMERA_CONTROL_WINDOW_Y': cls.__constants['CAMERA_CONTROL_WINDOW_Y'],
            'CAMERA_CONTROL_FLIP': cls.__constants['CAMERA_CONTROL_FLIP'],
            'CAMERA_CONTROL_MIRROR': cls.__constants['CAMERA_CONTROL_MIRROR'],
            'CAMERA_CONTROL_RESOLUTION_HALF': cls.__constants['CAMERA_CONTROL_RESOLUTION_HALF'],
        }

    @property
    def CAMERA_CONTROL_EXPOSURE(self):
        """Message constant 'CAMERA_CONTROL_EXPOSURE'."""
        return Metaclass_CameraControl.__constants['CAMERA_CONTROL_EXPOSURE']

    @property
    def CAMERA_CONTROL_GAIN(self):
        """Message constant 'CAMERA_CONTROL_GAIN'."""
        return Metaclass_CameraControl.__constants['CAMERA_CONTROL_GAIN']

    @property
    def CAMERA_CONTROL_WHITE_BALANCE_R(self):
        """Message constant 'CAMERA_CONTROL_WHITE_BALANCE_R'."""
        return Metaclass_CameraControl.__constants['CAMERA_CONTROL_WHITE_BALANCE_R']

    @property
    def CAMERA_CONTROL_WHITE_BALANCE_G(self):
        """Message constant 'CAMERA_CONTROL_WHITE_BALANCE_G'."""
        return Metaclass_CameraControl.__constants['CAMERA_CONTROL_WHITE_BALANCE_G']

    @property
    def CAMERA_CONTROL_WHITE_BALANCE_B(self):
        """Message constant 'CAMERA_CONTROL_WHITE_BALANCE_B'."""
        return Metaclass_CameraControl.__constants['CAMERA_CONTROL_WHITE_BALANCE_B']

    @property
    def CAMERA_CONTROL_WINDOW_X(self):
        """Message constant 'CAMERA_CONTROL_WINDOW_X'."""
        return Metaclass_CameraControl.__constants['CAMERA_CONTROL_WINDOW_X']

    @property
    def CAMERA_CONTROL_WINDOW_Y(self):
        """Message constant 'CAMERA_CONTROL_WINDOW_Y'."""
        return Metaclass_CameraControl.__constants['CAMERA_CONTROL_WINDOW_Y']

    @property
    def CAMERA_CONTROL_FLIP(self):
        """Message constant 'CAMERA_CONTROL_FLIP'."""
        return Metaclass_CameraControl.__constants['CAMERA_CONTROL_FLIP']

    @property
    def CAMERA_CONTROL_MIRROR(self):
        """Message constant 'CAMERA_CONTROL_MIRROR'."""
        return Metaclass_CameraControl.__constants['CAMERA_CONTROL_MIRROR']

    @property
    def CAMERA_CONTROL_RESOLUTION_HALF(self):
        """Message constant 'CAMERA_CONTROL_RESOLUTION_HALF'."""
        return Metaclass_CameraControl.__constants['CAMERA_CONTROL_RESOLUTION_HALF']


class CameraControl(metaclass=Metaclass_CameraControl):
    """
    Message class 'CameraControl'.

    Constants:
      CAMERA_CONTROL_EXPOSURE
      CAMERA_CONTROL_GAIN
      CAMERA_CONTROL_WHITE_BALANCE_R
      CAMERA_CONTROL_WHITE_BALANCE_G
      CAMERA_CONTROL_WHITE_BALANCE_B
      CAMERA_CONTROL_WINDOW_X
      CAMERA_CONTROL_WINDOW_Y
      CAMERA_CONTROL_FLIP
      CAMERA_CONTROL_MIRROR
      CAMERA_CONTROL_RESOLUTION_HALF
    """

    __slots__ = [
        '_id',
        '_value',
    ]

    _fields_and_field_types = {
        'id': 'int32',
        'value': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        self.value = kwargs.get('value', int())

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
        if self.id != other.id:
            return False
        if self.value != other.value:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'id' field must be an integer in [-2147483648, 2147483647]"
        self._id = value

    @builtins.property
    def value(self):
        """Message field 'value'."""
        return self._value

    @value.setter
    def value(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'value' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'value' field must be an integer in [-2147483648, 2147483647]"
        self._value = value
