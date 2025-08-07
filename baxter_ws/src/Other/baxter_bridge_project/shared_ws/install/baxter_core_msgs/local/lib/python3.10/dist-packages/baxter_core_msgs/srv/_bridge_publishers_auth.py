# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:srv/BridgePublishersAuth.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BridgePublishersAuth_Request(type):
    """Metaclass of message 'BridgePublishersAuth_Request'."""

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
                'baxter_core_msgs.srv.BridgePublishersAuth_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__bridge_publishers_auth__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__bridge_publishers_auth__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__bridge_publishers_auth__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__bridge_publishers_auth__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__bridge_publishers_auth__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BridgePublishersAuth_Request(metaclass=Metaclass_BridgePublishersAuth_Request):
    """Message class 'BridgePublishersAuth_Request'."""

    __slots__ = [
        '_topic',
        '_user',
    ]

    _fields_and_field_types = {
        'topic': 'string',
        'user': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.topic = kwargs.get('topic', str())
        self.user = kwargs.get('user', str())

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
        if self.topic != other.topic:
            return False
        if self.user != other.user:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def topic(self):
        """Message field 'topic'."""
        return self._topic

    @topic.setter
    def topic(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'topic' field must be of type 'str'"
        self._topic = value

    @builtins.property
    def user(self):
        """Message field 'user'."""
        return self._user

    @user.setter
    def user(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'user' field must be of type 'str'"
        self._user = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_BridgePublishersAuth_Response(type):
    """Metaclass of message 'BridgePublishersAuth_Response'."""

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
                'baxter_core_msgs.srv.BridgePublishersAuth_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__bridge_publishers_auth__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__bridge_publishers_auth__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__bridge_publishers_auth__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__bridge_publishers_auth__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__bridge_publishers_auth__response

            from baxter_core_msgs.msg import BridgePublisher
            if BridgePublisher.__class__._TYPE_SUPPORT is None:
                BridgePublisher.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BridgePublishersAuth_Response(metaclass=Metaclass_BridgePublishersAuth_Response):
    """Message class 'BridgePublishersAuth_Response'."""

    __slots__ = [
        '_publishers',
        '_forced_left',
        '_forced_right',
    ]

    _fields_and_field_types = {
        'publishers': 'sequence<baxter_core_msgs/BridgePublisher>',
        'forced_left': 'string',
        'forced_right': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['baxter_core_msgs', 'msg'], 'BridgePublisher')),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.publishers = kwargs.get('publishers', [])
        self.forced_left = kwargs.get('forced_left', str())
        self.forced_right = kwargs.get('forced_right', str())

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
        if self.publishers != other.publishers:
            return False
        if self.forced_left != other.forced_left:
            return False
        if self.forced_right != other.forced_right:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def publishers(self):
        """Message field 'publishers'."""
        return self._publishers

    @publishers.setter
    def publishers(self, value):
        if __debug__:
            from baxter_core_msgs.msg import BridgePublisher
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, BridgePublisher) for v in value) and
                 True), \
                "The 'publishers' field must be a set or sequence and each value of type 'BridgePublisher'"
        self._publishers = value

    @builtins.property
    def forced_left(self):
        """Message field 'forced_left'."""
        return self._forced_left

    @forced_left.setter
    def forced_left(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'forced_left' field must be of type 'str'"
        self._forced_left = value

    @builtins.property
    def forced_right(self):
        """Message field 'forced_right'."""
        return self._forced_right

    @forced_right.setter
    def forced_right(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'forced_right' field must be of type 'str'"
        self._forced_right = value


class Metaclass_BridgePublishersAuth(type):
    """Metaclass of service 'BridgePublishersAuth'."""

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
                'baxter_core_msgs.srv.BridgePublishersAuth')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__bridge_publishers_auth

            from baxter_core_msgs.srv import _bridge_publishers_auth
            if _bridge_publishers_auth.Metaclass_BridgePublishersAuth_Request._TYPE_SUPPORT is None:
                _bridge_publishers_auth.Metaclass_BridgePublishersAuth_Request.__import_type_support__()
            if _bridge_publishers_auth.Metaclass_BridgePublishersAuth_Response._TYPE_SUPPORT is None:
                _bridge_publishers_auth.Metaclass_BridgePublishersAuth_Response.__import_type_support__()


class BridgePublishersAuth(metaclass=Metaclass_BridgePublishersAuth):
    from baxter_core_msgs.srv._bridge_publishers_auth import BridgePublishersAuth_Request as Request
    from baxter_core_msgs.srv._bridge_publishers_auth import BridgePublishersAuth_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
