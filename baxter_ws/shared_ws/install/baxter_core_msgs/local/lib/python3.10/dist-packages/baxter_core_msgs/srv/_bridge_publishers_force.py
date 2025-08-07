# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:srv/BridgePublishersForce.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BridgePublishersForce_Request(type):
    """Metaclass of message 'BridgePublishersForce_Request'."""

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
                'baxter_core_msgs.srv.BridgePublishersForce_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__bridge_publishers_force__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__bridge_publishers_force__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__bridge_publishers_force__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__bridge_publishers_force__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__bridge_publishers_force__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BridgePublishersForce_Request(metaclass=Metaclass_BridgePublishersForce_Request):
    """Message class 'BridgePublishersForce_Request'."""

    __slots__ = [
        '_left_user',
        '_right_user',
    ]

    _fields_and_field_types = {
        'left_user': 'string',
        'right_user': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.left_user = kwargs.get('left_user', str())
        self.right_user = kwargs.get('right_user', str())

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
        if self.left_user != other.left_user:
            return False
        if self.right_user != other.right_user:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def left_user(self):
        """Message field 'left_user'."""
        return self._left_user

    @left_user.setter
    def left_user(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'left_user' field must be of type 'str'"
        self._left_user = value

    @builtins.property
    def right_user(self):
        """Message field 'right_user'."""
        return self._right_user

    @right_user.setter
    def right_user(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'right_user' field must be of type 'str'"
        self._right_user = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_BridgePublishersForce_Response(type):
    """Metaclass of message 'BridgePublishersForce_Response'."""

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
                'baxter_core_msgs.srv.BridgePublishersForce_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__bridge_publishers_force__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__bridge_publishers_force__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__bridge_publishers_force__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__bridge_publishers_force__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__bridge_publishers_force__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BridgePublishersForce_Response(metaclass=Metaclass_BridgePublishersForce_Response):
    """Message class 'BridgePublishersForce_Response'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


class Metaclass_BridgePublishersForce(type):
    """Metaclass of service 'BridgePublishersForce'."""

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
                'baxter_core_msgs.srv.BridgePublishersForce')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__bridge_publishers_force

            from baxter_core_msgs.srv import _bridge_publishers_force
            if _bridge_publishers_force.Metaclass_BridgePublishersForce_Request._TYPE_SUPPORT is None:
                _bridge_publishers_force.Metaclass_BridgePublishersForce_Request.__import_type_support__()
            if _bridge_publishers_force.Metaclass_BridgePublishersForce_Response._TYPE_SUPPORT is None:
                _bridge_publishers_force.Metaclass_BridgePublishersForce_Response.__import_type_support__()


class BridgePublishersForce(metaclass=Metaclass_BridgePublishersForce):
    from baxter_core_msgs.srv._bridge_publishers_force import BridgePublishersForce_Request as Request
    from baxter_core_msgs.srv._bridge_publishers_force import BridgePublishersForce_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
