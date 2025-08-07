# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_maintenance_msgs:msg/UpdateSource.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UpdateSource(type):
    """Metaclass of message 'UpdateSource'."""

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
            module = import_type_support('baxter_maintenance_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'baxter_maintenance_msgs.msg.UpdateSource')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__update_source
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__update_source
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__update_source
            cls._TYPE_SUPPORT = module.type_support_msg__msg__update_source
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__update_source

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UpdateSource(metaclass=Metaclass_UpdateSource):
    """Message class 'UpdateSource'."""

    __slots__ = [
        '_devname',
        '_filename',
        '_version',
        '_uuid',
    ]

    _fields_and_field_types = {
        'devname': 'string',
        'filename': 'string',
        'version': 'string',
        'uuid': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.devname = kwargs.get('devname', str())
        self.filename = kwargs.get('filename', str())
        self.version = kwargs.get('version', str())
        self.uuid = kwargs.get('uuid', str())

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
        if self.devname != other.devname:
            return False
        if self.filename != other.filename:
            return False
        if self.version != other.version:
            return False
        if self.uuid != other.uuid:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def devname(self):
        """Message field 'devname'."""
        return self._devname

    @devname.setter
    def devname(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'devname' field must be of type 'str'"
        self._devname = value

    @builtins.property
    def filename(self):
        """Message field 'filename'."""
        return self._filename

    @filename.setter
    def filename(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'filename' field must be of type 'str'"
        self._filename = value

    @builtins.property
    def version(self):
        """Message field 'version'."""
        return self._version

    @version.setter
    def version(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'version' field must be of type 'str'"
        self._version = value

    @builtins.property
    def uuid(self):
        """Message field 'uuid'."""
        return self._uuid

    @uuid.setter
    def uuid(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'uuid' field must be of type 'str'"
        self._uuid = value
