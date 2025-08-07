# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:srv/OpenCamera.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OpenCamera_Request(type):
    """Metaclass of message 'OpenCamera_Request'."""

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
                'baxter_core_msgs.srv.OpenCamera_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__open_camera__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__open_camera__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__open_camera__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__open_camera__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__open_camera__request

            from baxter_core_msgs.msg import CameraSettings
            if CameraSettings.__class__._TYPE_SUPPORT is None:
                CameraSettings.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OpenCamera_Request(metaclass=Metaclass_OpenCamera_Request):
    """Message class 'OpenCamera_Request'."""

    __slots__ = [
        '_name',
        '_settings',
    ]

    _fields_and_field_types = {
        'name': 'string',
        'settings': 'baxter_core_msgs/CameraSettings',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['baxter_core_msgs', 'msg'], 'CameraSettings'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.name = kwargs.get('name', str())
        from baxter_core_msgs.msg import CameraSettings
        self.settings = kwargs.get('settings', CameraSettings())

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
        if self.name != other.name:
            return False
        if self.settings != other.settings:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @builtins.property
    def settings(self):
        """Message field 'settings'."""
        return self._settings

    @settings.setter
    def settings(self, value):
        if __debug__:
            from baxter_core_msgs.msg import CameraSettings
            assert \
                isinstance(value, CameraSettings), \
                "The 'settings' field must be a sub message of type 'CameraSettings'"
        self._settings = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_OpenCamera_Response(type):
    """Metaclass of message 'OpenCamera_Response'."""

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
                'baxter_core_msgs.srv.OpenCamera_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__open_camera__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__open_camera__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__open_camera__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__open_camera__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__open_camera__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OpenCamera_Response(metaclass=Metaclass_OpenCamera_Response):
    """Message class 'OpenCamera_Response'."""

    __slots__ = [
        '_err',
    ]

    _fields_and_field_types = {
        'err': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.err = kwargs.get('err', int())

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
        if self.err != other.err:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def err(self):
        """Message field 'err'."""
        return self._err

    @err.setter
    def err(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'err' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'err' field must be an integer in [-2147483648, 2147483647]"
        self._err = value


class Metaclass_OpenCamera(type):
    """Metaclass of service 'OpenCamera'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('baxter_core_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'baxter_core_msgs.srv.OpenCamera')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__open_camera

            from baxter_core_msgs.srv import _open_camera
            if _open_camera.Metaclass_OpenCamera_Request._TYPE_SUPPORT is None:
                _open_camera.Metaclass_OpenCamera_Request.__import_type_support__()
            if _open_camera.Metaclass_OpenCamera_Response._TYPE_SUPPORT is None:
                _open_camera.Metaclass_OpenCamera_Response.__import_type_support__()


class OpenCamera(metaclass=Metaclass_OpenCamera):
    from baxter_core_msgs.srv._open_camera import OpenCamera_Request as Request
    from baxter_core_msgs.srv._open_camera import OpenCamera_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
