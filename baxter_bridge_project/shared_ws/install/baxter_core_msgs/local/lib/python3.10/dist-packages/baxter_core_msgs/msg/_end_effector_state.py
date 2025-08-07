# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:msg/EndEffectorState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EndEffectorState(type):
    """Metaclass of message 'EndEffectorState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STATE_FALSE': 0,
        'STATE_TRUE': 1,
        'STATE_UNKNOWN': 2,
        'POSITION_CLOSED': 0.0,
        'POSITION_OPEN': 100.0,
        'FORCE_MIN': 0.0,
        'FORCE_MAX': 100.0,
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
                'baxter_core_msgs.msg.EndEffectorState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__end_effector_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__end_effector_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__end_effector_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__end_effector_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__end_effector_state

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'STATE_FALSE': cls.__constants['STATE_FALSE'],
            'STATE_TRUE': cls.__constants['STATE_TRUE'],
            'STATE_UNKNOWN': cls.__constants['STATE_UNKNOWN'],
            'POSITION_CLOSED': cls.__constants['POSITION_CLOSED'],
            'POSITION_OPEN': cls.__constants['POSITION_OPEN'],
            'FORCE_MIN': cls.__constants['FORCE_MIN'],
            'FORCE_MAX': cls.__constants['FORCE_MAX'],
        }

    @property
    def STATE_FALSE(self):
        """Message constant 'STATE_FALSE'."""
        return Metaclass_EndEffectorState.__constants['STATE_FALSE']

    @property
    def STATE_TRUE(self):
        """Message constant 'STATE_TRUE'."""
        return Metaclass_EndEffectorState.__constants['STATE_TRUE']

    @property
    def STATE_UNKNOWN(self):
        """Message constant 'STATE_UNKNOWN'."""
        return Metaclass_EndEffectorState.__constants['STATE_UNKNOWN']

    @property
    def POSITION_CLOSED(self):
        """Message constant 'POSITION_CLOSED'."""
        return Metaclass_EndEffectorState.__constants['POSITION_CLOSED']

    @property
    def POSITION_OPEN(self):
        """Message constant 'POSITION_OPEN'."""
        return Metaclass_EndEffectorState.__constants['POSITION_OPEN']

    @property
    def FORCE_MIN(self):
        """Message constant 'FORCE_MIN'."""
        return Metaclass_EndEffectorState.__constants['FORCE_MIN']

    @property
    def FORCE_MAX(self):
        """Message constant 'FORCE_MAX'."""
        return Metaclass_EndEffectorState.__constants['FORCE_MAX']


class EndEffectorState(metaclass=Metaclass_EndEffectorState):
    """
    Message class 'EndEffectorState'.

    Constants:
      STATE_FALSE
      STATE_TRUE
      STATE_UNKNOWN
      POSITION_CLOSED
      POSITION_OPEN
      FORCE_MIN
      FORCE_MAX
    """

    __slots__ = [
        '_timestamp',
        '_id',
        '_enabled',
        '_calibrated',
        '_ready',
        '_moving',
        '_gripping',
        '_missed',
        '_error',
        '_reverse',
        '_position',
        '_force',
        '_state',
        '_command',
        '_command_sender',
        '_command_sequence',
    ]

    _fields_and_field_types = {
        'timestamp': 'builtin_interfaces/Time',
        'id': 'uint32',
        'enabled': 'uint8',
        'calibrated': 'uint8',
        'ready': 'uint8',
        'moving': 'uint8',
        'gripping': 'uint8',
        'missed': 'uint8',
        'error': 'uint8',
        'reverse': 'uint8',
        'position': 'float',
        'force': 'float',
        'state': 'string',
        'command': 'string',
        'command_sender': 'string',
        'command_sequence': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Time
        self.timestamp = kwargs.get('timestamp', Time())
        self.id = kwargs.get('id', int())
        self.enabled = kwargs.get('enabled', int())
        self.calibrated = kwargs.get('calibrated', int())
        self.ready = kwargs.get('ready', int())
        self.moving = kwargs.get('moving', int())
        self.gripping = kwargs.get('gripping', int())
        self.missed = kwargs.get('missed', int())
        self.error = kwargs.get('error', int())
        self.reverse = kwargs.get('reverse', int())
        self.position = kwargs.get('position', float())
        self.force = kwargs.get('force', float())
        self.state = kwargs.get('state', str())
        self.command = kwargs.get('command', str())
        self.command_sender = kwargs.get('command_sender', str())
        self.command_sequence = kwargs.get('command_sequence', int())

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
        if self.timestamp != other.timestamp:
            return False
        if self.id != other.id:
            return False
        if self.enabled != other.enabled:
            return False
        if self.calibrated != other.calibrated:
            return False
        if self.ready != other.ready:
            return False
        if self.moving != other.moving:
            return False
        if self.gripping != other.gripping:
            return False
        if self.missed != other.missed:
            return False
        if self.error != other.error:
            return False
        if self.reverse != other.reverse:
            return False
        if self.position != other.position:
            return False
        if self.force != other.force:
            return False
        if self.state != other.state:
            return False
        if self.command != other.command:
            return False
        if self.command_sender != other.command_sender:
            return False
        if self.command_sequence != other.command_sequence:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'timestamp' field must be a sub message of type 'Time'"
        self._timestamp = value

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
            assert value >= 0 and value < 4294967296, \
                "The 'id' field must be an unsigned integer in [0, 4294967295]"
        self._id = value

    @builtins.property
    def enabled(self):
        """Message field 'enabled'."""
        return self._enabled

    @enabled.setter
    def enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enabled' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'enabled' field must be an unsigned integer in [0, 255]"
        self._enabled = value

    @builtins.property
    def calibrated(self):
        """Message field 'calibrated'."""
        return self._calibrated

    @calibrated.setter
    def calibrated(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'calibrated' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'calibrated' field must be an unsigned integer in [0, 255]"
        self._calibrated = value

    @builtins.property
    def ready(self):
        """Message field 'ready'."""
        return self._ready

    @ready.setter
    def ready(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ready' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'ready' field must be an unsigned integer in [0, 255]"
        self._ready = value

    @builtins.property
    def moving(self):
        """Message field 'moving'."""
        return self._moving

    @moving.setter
    def moving(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'moving' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'moving' field must be an unsigned integer in [0, 255]"
        self._moving = value

    @builtins.property
    def gripping(self):
        """Message field 'gripping'."""
        return self._gripping

    @gripping.setter
    def gripping(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gripping' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'gripping' field must be an unsigned integer in [0, 255]"
        self._gripping = value

    @builtins.property
    def missed(self):
        """Message field 'missed'."""
        return self._missed

    @missed.setter
    def missed(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'missed' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'missed' field must be an unsigned integer in [0, 255]"
        self._missed = value

    @builtins.property
    def error(self):
        """Message field 'error'."""
        return self._error

    @error.setter
    def error(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'error' field must be an unsigned integer in [0, 255]"
        self._error = value

    @builtins.property
    def reverse(self):
        """Message field 'reverse'."""
        return self._reverse

    @reverse.setter
    def reverse(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'reverse' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'reverse' field must be an unsigned integer in [0, 255]"
        self._reverse = value

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'position' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._position = value

    @builtins.property
    def force(self):
        """Message field 'force'."""
        return self._force

    @force.setter
    def force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'force' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'force' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._force = value

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'state' field must be of type 'str'"
        self._state = value

    @builtins.property
    def command(self):
        """Message field 'command'."""
        return self._command

    @command.setter
    def command(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'command' field must be of type 'str'"
        self._command = value

    @builtins.property
    def command_sender(self):
        """Message field 'command_sender'."""
        return self._command_sender

    @command_sender.setter
    def command_sender(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'command_sender' field must be of type 'str'"
        self._command_sender = value

    @builtins.property
    def command_sequence(self):
        """Message field 'command_sequence'."""
        return self._command_sequence

    @command_sequence.setter
    def command_sequence(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'command_sequence' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'command_sequence' field must be an unsigned integer in [0, 4294967295]"
        self._command_sequence = value
