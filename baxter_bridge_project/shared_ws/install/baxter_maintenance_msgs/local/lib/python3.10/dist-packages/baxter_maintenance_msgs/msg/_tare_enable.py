# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_maintenance_msgs:msg/TareEnable.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TareEnable(type):
    """Metaclass of message 'TareEnable'."""

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
                'baxter_maintenance_msgs.msg.TareEnable')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tare_enable
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tare_enable
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tare_enable
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tare_enable
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tare_enable

            from baxter_maintenance_msgs.msg import TareData
            if TareData.__class__._TYPE_SUPPORT is None:
                TareData.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TareEnable(metaclass=Metaclass_TareEnable):
    """Message class 'TareEnable'."""

    __slots__ = [
        '_is_enabled',
        '_uid',
        '_data',
    ]

    _fields_and_field_types = {
        'is_enabled': 'boolean',
        'uid': 'string',
        'data': 'baxter_maintenance_msgs/TareData',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['baxter_maintenance_msgs', 'msg'], 'TareData'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.is_enabled = kwargs.get('is_enabled', bool())
        self.uid = kwargs.get('uid', str())
        from baxter_maintenance_msgs.msg import TareData
        self.data = kwargs.get('data', TareData())

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
        if self.is_enabled != other.is_enabled:
            return False
        if self.uid != other.uid:
            return False
        if self.data != other.data:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def is_enabled(self):
        """Message field 'is_enabled'."""
        return self._is_enabled

    @is_enabled.setter
    def is_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_enabled' field must be of type 'bool'"
        self._is_enabled = value

    @builtins.property
    def uid(self):
        """Message field 'uid'."""
        return self._uid

    @uid.setter
    def uid(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'uid' field must be of type 'str'"
        self._uid = value

    @builtins.property
    def data(self):
        """Message field 'data'."""
        return self._data

    @data.setter
    def data(self, value):
        if __debug__:
            from baxter_maintenance_msgs.msg import TareData
            assert \
                isinstance(value, TareData), \
                "The 'data' field must be a sub message of type 'TareData'"
        self._data = value
