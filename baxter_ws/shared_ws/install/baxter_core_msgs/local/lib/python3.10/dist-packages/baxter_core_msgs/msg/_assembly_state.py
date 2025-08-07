# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:msg/AssemblyState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AssemblyState(type):
    """Metaclass of message 'AssemblyState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ESTOP_BUTTON_UNPRESSED': 0,
        'ESTOP_BUTTON_PRESSED': 1,
        'ESTOP_BUTTON_UNKNOWN': 2,
        'ESTOP_BUTTON_RELEASED': 3,
        'ESTOP_SOURCE_NONE': 0,
        'ESTOP_SOURCE_USER': 1,
        'ESTOP_SOURCE_UNKNOWN': 2,
        'ESTOP_SOURCE_FAULT': 3,
        'ESTOP_SOURCE_BRAIN': 4,
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
                'baxter_core_msgs.msg.AssemblyState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__assembly_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__assembly_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__assembly_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__assembly_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__assembly_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ESTOP_BUTTON_UNPRESSED': cls.__constants['ESTOP_BUTTON_UNPRESSED'],
            'ESTOP_BUTTON_PRESSED': cls.__constants['ESTOP_BUTTON_PRESSED'],
            'ESTOP_BUTTON_UNKNOWN': cls.__constants['ESTOP_BUTTON_UNKNOWN'],
            'ESTOP_BUTTON_RELEASED': cls.__constants['ESTOP_BUTTON_RELEASED'],
            'ESTOP_SOURCE_NONE': cls.__constants['ESTOP_SOURCE_NONE'],
            'ESTOP_SOURCE_USER': cls.__constants['ESTOP_SOURCE_USER'],
            'ESTOP_SOURCE_UNKNOWN': cls.__constants['ESTOP_SOURCE_UNKNOWN'],
            'ESTOP_SOURCE_FAULT': cls.__constants['ESTOP_SOURCE_FAULT'],
            'ESTOP_SOURCE_BRAIN': cls.__constants['ESTOP_SOURCE_BRAIN'],
        }

    @property
    def ESTOP_BUTTON_UNPRESSED(self):
        """Message constant 'ESTOP_BUTTON_UNPRESSED'."""
        return Metaclass_AssemblyState.__constants['ESTOP_BUTTON_UNPRESSED']

    @property
    def ESTOP_BUTTON_PRESSED(self):
        """Message constant 'ESTOP_BUTTON_PRESSED'."""
        return Metaclass_AssemblyState.__constants['ESTOP_BUTTON_PRESSED']

    @property
    def ESTOP_BUTTON_UNKNOWN(self):
        """Message constant 'ESTOP_BUTTON_UNKNOWN'."""
        return Metaclass_AssemblyState.__constants['ESTOP_BUTTON_UNKNOWN']

    @property
    def ESTOP_BUTTON_RELEASED(self):
        """Message constant 'ESTOP_BUTTON_RELEASED'."""
        return Metaclass_AssemblyState.__constants['ESTOP_BUTTON_RELEASED']

    @property
    def ESTOP_SOURCE_NONE(self):
        """Message constant 'ESTOP_SOURCE_NONE'."""
        return Metaclass_AssemblyState.__constants['ESTOP_SOURCE_NONE']

    @property
    def ESTOP_SOURCE_USER(self):
        """Message constant 'ESTOP_SOURCE_USER'."""
        return Metaclass_AssemblyState.__constants['ESTOP_SOURCE_USER']

    @property
    def ESTOP_SOURCE_UNKNOWN(self):
        """Message constant 'ESTOP_SOURCE_UNKNOWN'."""
        return Metaclass_AssemblyState.__constants['ESTOP_SOURCE_UNKNOWN']

    @property
    def ESTOP_SOURCE_FAULT(self):
        """Message constant 'ESTOP_SOURCE_FAULT'."""
        return Metaclass_AssemblyState.__constants['ESTOP_SOURCE_FAULT']

    @property
    def ESTOP_SOURCE_BRAIN(self):
        """Message constant 'ESTOP_SOURCE_BRAIN'."""
        return Metaclass_AssemblyState.__constants['ESTOP_SOURCE_BRAIN']


class AssemblyState(metaclass=Metaclass_AssemblyState):
    """
    Message class 'AssemblyState'.

    Constants:
      ESTOP_BUTTON_UNPRESSED
      ESTOP_BUTTON_PRESSED
      ESTOP_BUTTON_UNKNOWN
      ESTOP_BUTTON_RELEASED
      ESTOP_SOURCE_NONE
      ESTOP_SOURCE_USER
      ESTOP_SOURCE_UNKNOWN
      ESTOP_SOURCE_FAULT
      ESTOP_SOURCE_BRAIN
    """

    __slots__ = [
        '_ready',
        '_enabled',
        '_stopped',
        '_error',
        '_estop_button',
        '_estop_source',
    ]

    _fields_and_field_types = {
        'ready': 'boolean',
        'enabled': 'boolean',
        'stopped': 'boolean',
        'error': 'boolean',
        'estop_button': 'uint8',
        'estop_source': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ready = kwargs.get('ready', bool())
        self.enabled = kwargs.get('enabled', bool())
        self.stopped = kwargs.get('stopped', bool())
        self.error = kwargs.get('error', bool())
        self.estop_button = kwargs.get('estop_button', int())
        self.estop_source = kwargs.get('estop_source', int())

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
        if self.ready != other.ready:
            return False
        if self.enabled != other.enabled:
            return False
        if self.stopped != other.stopped:
            return False
        if self.error != other.error:
            return False
        if self.estop_button != other.estop_button:
            return False
        if self.estop_source != other.estop_source:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ready(self):
        """Message field 'ready'."""
        return self._ready

    @ready.setter
    def ready(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'ready' field must be of type 'bool'"
        self._ready = value

    @builtins.property
    def enabled(self):
        """Message field 'enabled'."""
        return self._enabled

    @enabled.setter
    def enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'enabled' field must be of type 'bool'"
        self._enabled = value

    @builtins.property
    def stopped(self):
        """Message field 'stopped'."""
        return self._stopped

    @stopped.setter
    def stopped(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'stopped' field must be of type 'bool'"
        self._stopped = value

    @builtins.property
    def error(self):
        """Message field 'error'."""
        return self._error

    @error.setter
    def error(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'error' field must be of type 'bool'"
        self._error = value

    @builtins.property
    def estop_button(self):
        """Message field 'estop_button'."""
        return self._estop_button

    @estop_button.setter
    def estop_button(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'estop_button' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'estop_button' field must be an unsigned integer in [0, 255]"
        self._estop_button = value

    @builtins.property
    def estop_source(self):
        """Message field 'estop_source'."""
        return self._estop_source

    @estop_source.setter
    def estop_source(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'estop_source' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'estop_source' field must be an unsigned integer in [0, 255]"
        self._estop_source = value
