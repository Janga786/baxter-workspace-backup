# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:msg/DigitalIOState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DigitalIOState(type):
    """Metaclass of message 'DigitalIOState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'OFF': 0,
        'ON': 1,
        'PRESSED': 1,
        'UNPRESSED': 0,
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
                'baxter_core_msgs.msg.DigitalIOState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__digital_io_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__digital_io_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__digital_io_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__digital_io_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__digital_io_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'OFF': cls.__constants['OFF'],
            'ON': cls.__constants['ON'],
            'PRESSED': cls.__constants['PRESSED'],
            'UNPRESSED': cls.__constants['UNPRESSED'],
        }

    @property
    def OFF(self):
        """Message constant 'OFF'."""
        return Metaclass_DigitalIOState.__constants['OFF']

    @property
    def ON(self):
        """Message constant 'ON'."""
        return Metaclass_DigitalIOState.__constants['ON']

    @property
    def PRESSED(self):
        """Message constant 'PRESSED'."""
        return Metaclass_DigitalIOState.__constants['PRESSED']

    @property
    def UNPRESSED(self):
        """Message constant 'UNPRESSED'."""
        return Metaclass_DigitalIOState.__constants['UNPRESSED']


class DigitalIOState(metaclass=Metaclass_DigitalIOState):
    """
    Message class 'DigitalIOState'.

    Constants:
      OFF
      ON
      PRESSED
      UNPRESSED
    """

    __slots__ = [
        '_state',
        '_is_input_only',
    ]

    _fields_and_field_types = {
        'state': 'int8',
        'is_input_only': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.state = kwargs.get('state', int())
        self.is_input_only = kwargs.get('is_input_only', bool())

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
        if self.state != other.state:
            return False
        if self.is_input_only != other.is_input_only:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'state' field must be an integer in [-128, 127]"
        self._state = value

    @builtins.property
    def is_input_only(self):
        """Message field 'is_input_only'."""
        return self._is_input_only

    @is_input_only.setter
    def is_input_only(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_input_only' field must be of type 'bool'"
        self._is_input_only = value
