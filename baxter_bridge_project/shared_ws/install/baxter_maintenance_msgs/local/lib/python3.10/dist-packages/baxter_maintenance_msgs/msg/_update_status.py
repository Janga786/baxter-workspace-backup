# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_maintenance_msgs:msg/UpdateStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UpdateStatus(type):
    """Metaclass of message 'UpdateStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STS_IDLE': 0,
        'STS_INVALID': 1,
        'STS_BUSY': 2,
        'STS_CANCELLED': 3,
        'STS_ERR': 4,
        'STS_MOUNT_UPDATE': 5,
        'STS_VERIFY_UPDATE': 6,
        'STS_PREP_STAGING': 7,
        'STS_MOUNT_STAGING': 8,
        'STS_EXTRACT_UPDATE': 9,
        'STS_LOAD_KEXEC': 10,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('baxter_maintenance_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'baxter_maintenance_msgs.msg.UpdateStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__update_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__update_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__update_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__update_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__update_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'STS_IDLE': cls.__constants['STS_IDLE'],
            'STS_INVALID': cls.__constants['STS_INVALID'],
            'STS_BUSY': cls.__constants['STS_BUSY'],
            'STS_CANCELLED': cls.__constants['STS_CANCELLED'],
            'STS_ERR': cls.__constants['STS_ERR'],
            'STS_MOUNT_UPDATE': cls.__constants['STS_MOUNT_UPDATE'],
            'STS_VERIFY_UPDATE': cls.__constants['STS_VERIFY_UPDATE'],
            'STS_PREP_STAGING': cls.__constants['STS_PREP_STAGING'],
            'STS_MOUNT_STAGING': cls.__constants['STS_MOUNT_STAGING'],
            'STS_EXTRACT_UPDATE': cls.__constants['STS_EXTRACT_UPDATE'],
            'STS_LOAD_KEXEC': cls.__constants['STS_LOAD_KEXEC'],
        }

    @property
    def STS_IDLE(self):
        """Message constant 'STS_IDLE'."""
        return Metaclass_UpdateStatus.__constants['STS_IDLE']

    @property
    def STS_INVALID(self):
        """Message constant 'STS_INVALID'."""
        return Metaclass_UpdateStatus.__constants['STS_INVALID']

    @property
    def STS_BUSY(self):
        """Message constant 'STS_BUSY'."""
        return Metaclass_UpdateStatus.__constants['STS_BUSY']

    @property
    def STS_CANCELLED(self):
        """Message constant 'STS_CANCELLED'."""
        return Metaclass_UpdateStatus.__constants['STS_CANCELLED']

    @property
    def STS_ERR(self):
        """Message constant 'STS_ERR'."""
        return Metaclass_UpdateStatus.__constants['STS_ERR']

    @property
    def STS_MOUNT_UPDATE(self):
        """Message constant 'STS_MOUNT_UPDATE'."""
        return Metaclass_UpdateStatus.__constants['STS_MOUNT_UPDATE']

    @property
    def STS_VERIFY_UPDATE(self):
        """Message constant 'STS_VERIFY_UPDATE'."""
        return Metaclass_UpdateStatus.__constants['STS_VERIFY_UPDATE']

    @property
    def STS_PREP_STAGING(self):
        """Message constant 'STS_PREP_STAGING'."""
        return Metaclass_UpdateStatus.__constants['STS_PREP_STAGING']

    @property
    def STS_MOUNT_STAGING(self):
        """Message constant 'STS_MOUNT_STAGING'."""
        return Metaclass_UpdateStatus.__constants['STS_MOUNT_STAGING']

    @property
    def STS_EXTRACT_UPDATE(self):
        """Message constant 'STS_EXTRACT_UPDATE'."""
        return Metaclass_UpdateStatus.__constants['STS_EXTRACT_UPDATE']

    @property
    def STS_LOAD_KEXEC(self):
        """Message constant 'STS_LOAD_KEXEC'."""
        return Metaclass_UpdateStatus.__constants['STS_LOAD_KEXEC']


class UpdateStatus(metaclass=Metaclass_UpdateStatus):
    """
    Message class 'UpdateStatus'.

    Constants:
      STS_IDLE
      STS_INVALID
      STS_BUSY
      STS_CANCELLED
      STS_ERR
      STS_MOUNT_UPDATE
      STS_VERIFY_UPDATE
      STS_PREP_STAGING
      STS_MOUNT_STAGING
      STS_EXTRACT_UPDATE
      STS_LOAD_KEXEC
    """

    __slots__ = [
        '_status',
        '_progress',
        '_long_description',
    ]

    _fields_and_field_types = {
        'status': 'uint16',
        'progress': 'float',
        'long_description': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        self.progress = kwargs.get('progress', float())
        self.long_description = kwargs.get('long_description', str())

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
        if self.status != other.status:
            return False
        if self.progress != other.progress:
            return False
        if self.long_description != other.long_description:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'status' field must be an unsigned integer in [0, 65535]"
        self._status = value

    @builtins.property
    def progress(self):
        """Message field 'progress'."""
        return self._progress

    @progress.setter
    def progress(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'progress' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'progress' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._progress = value

    @builtins.property
    def long_description(self):
        """Message field 'long_description'."""
        return self._long_description

    @long_description.setter
    def long_description(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'long_description' field must be of type 'str'"
        self._long_description = value
