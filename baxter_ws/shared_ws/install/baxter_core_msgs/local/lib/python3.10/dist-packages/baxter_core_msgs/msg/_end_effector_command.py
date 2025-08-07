# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:msg/EndEffectorCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EndEffectorCommand(type):
    """Metaclass of message 'EndEffectorCommand'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CMD_NO_OP': 'no_op',
        'CMD_SET': 'set',
        'CMD_CONFIGURE': 'configure',
        'CMD_REBOOT': 'reboot',
        'CMD_RESET': 'reset',
        'CMD_CALIBRATE': 'calibrate',
        'CMD_CLEAR_CALIBRATION': 'clear_calibration',
        'CMD_PREPARE_TO_GRIP': 'prepare_to_grip',
        'CMD_GRIP': 'grip',
        'CMD_RELEASE': 'release',
        'CMD_GO': 'go',
        'CMD_STOP': 'stop',
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
                'baxter_core_msgs.msg.EndEffectorCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__end_effector_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__end_effector_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__end_effector_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__end_effector_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__end_effector_command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CMD_NO_OP': cls.__constants['CMD_NO_OP'],
            'CMD_SET': cls.__constants['CMD_SET'],
            'CMD_CONFIGURE': cls.__constants['CMD_CONFIGURE'],
            'CMD_REBOOT': cls.__constants['CMD_REBOOT'],
            'CMD_RESET': cls.__constants['CMD_RESET'],
            'CMD_CALIBRATE': cls.__constants['CMD_CALIBRATE'],
            'CMD_CLEAR_CALIBRATION': cls.__constants['CMD_CLEAR_CALIBRATION'],
            'CMD_PREPARE_TO_GRIP': cls.__constants['CMD_PREPARE_TO_GRIP'],
            'CMD_GRIP': cls.__constants['CMD_GRIP'],
            'CMD_RELEASE': cls.__constants['CMD_RELEASE'],
            'CMD_GO': cls.__constants['CMD_GO'],
            'CMD_STOP': cls.__constants['CMD_STOP'],
        }

    @property
    def CMD_NO_OP(self):
        """Message constant 'CMD_NO_OP'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_NO_OP']

    @property
    def CMD_SET(self):
        """Message constant 'CMD_SET'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_SET']

    @property
    def CMD_CONFIGURE(self):
        """Message constant 'CMD_CONFIGURE'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_CONFIGURE']

    @property
    def CMD_REBOOT(self):
        """Message constant 'CMD_REBOOT'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_REBOOT']

    @property
    def CMD_RESET(self):
        """Message constant 'CMD_RESET'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_RESET']

    @property
    def CMD_CALIBRATE(self):
        """Message constant 'CMD_CALIBRATE'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_CALIBRATE']

    @property
    def CMD_CLEAR_CALIBRATION(self):
        """Message constant 'CMD_CLEAR_CALIBRATION'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_CLEAR_CALIBRATION']

    @property
    def CMD_PREPARE_TO_GRIP(self):
        """Message constant 'CMD_PREPARE_TO_GRIP'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_PREPARE_TO_GRIP']

    @property
    def CMD_GRIP(self):
        """Message constant 'CMD_GRIP'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_GRIP']

    @property
    def CMD_RELEASE(self):
        """Message constant 'CMD_RELEASE'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_RELEASE']

    @property
    def CMD_GO(self):
        """Message constant 'CMD_GO'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_GO']

    @property
    def CMD_STOP(self):
        """Message constant 'CMD_STOP'."""
        return Metaclass_EndEffectorCommand.__constants['CMD_STOP']


class EndEffectorCommand(metaclass=Metaclass_EndEffectorCommand):
    """
    Message class 'EndEffectorCommand'.

    Constants:
      CMD_NO_OP
      CMD_SET
      CMD_CONFIGURE
      CMD_REBOOT
      CMD_RESET
      CMD_CALIBRATE
      CMD_CLEAR_CALIBRATION
      CMD_PREPARE_TO_GRIP
      CMD_GRIP
      CMD_RELEASE
      CMD_GO
      CMD_STOP
    """

    __slots__ = [
        '_id',
        '_command',
        '_args',
        '_sender',
        '_sequence',
    ]

    _fields_and_field_types = {
        'id': 'uint32',
        'command': 'string',
        'args': 'string',
        'sender': 'string',
        'sequence': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        self.command = kwargs.get('command', str())
        self.args = kwargs.get('args', str())
        self.sender = kwargs.get('sender', str())
        self.sequence = kwargs.get('sequence', int())

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
        if self.command != other.command:
            return False
        if self.args != other.args:
            return False
        if self.sender != other.sender:
            return False
        if self.sequence != other.sequence:
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
            assert value >= 0 and value < 4294967296, \
                "The 'id' field must be an unsigned integer in [0, 4294967295]"
        self._id = value

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
    def args(self):
        """Message field 'args'."""
        return self._args

    @args.setter
    def args(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'args' field must be of type 'str'"
        self._args = value

    @builtins.property
    def sender(self):
        """Message field 'sender'."""
        return self._sender

    @sender.setter
    def sender(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'sender' field must be of type 'str'"
        self._sender = value

    @builtins.property
    def sequence(self):
        """Message field 'sequence'."""
        return self._sequence

    @sequence.setter
    def sequence(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sequence' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'sequence' field must be an unsigned integer in [0, 4294967295]"
        self._sequence = value
