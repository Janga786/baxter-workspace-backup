# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:srv/SolvePositionIK.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SolvePositionIK_Request(type):
    """Metaclass of message 'SolvePositionIK_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SEED_AUTO': 0,
        'SEED_USER': 1,
        'SEED_CURRENT': 2,
        'SEED_NS_MAP': 3,
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
                'baxter_core_msgs.srv.SolvePositionIK_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__solve_position_ik__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__solve_position_ik__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__solve_position_ik__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__solve_position_ik__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__solve_position_ik__request

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

            from sensor_msgs.msg import JointState
            if JointState.__class__._TYPE_SUPPORT is None:
                JointState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SEED_AUTO': cls.__constants['SEED_AUTO'],
            'SEED_USER': cls.__constants['SEED_USER'],
            'SEED_CURRENT': cls.__constants['SEED_CURRENT'],
            'SEED_NS_MAP': cls.__constants['SEED_NS_MAP'],
        }

    @property
    def SEED_AUTO(self):
        """Message constant 'SEED_AUTO'."""
        return Metaclass_SolvePositionIK_Request.__constants['SEED_AUTO']

    @property
    def SEED_USER(self):
        """Message constant 'SEED_USER'."""
        return Metaclass_SolvePositionIK_Request.__constants['SEED_USER']

    @property
    def SEED_CURRENT(self):
        """Message constant 'SEED_CURRENT'."""
        return Metaclass_SolvePositionIK_Request.__constants['SEED_CURRENT']

    @property
    def SEED_NS_MAP(self):
        """Message constant 'SEED_NS_MAP'."""
        return Metaclass_SolvePositionIK_Request.__constants['SEED_NS_MAP']


class SolvePositionIK_Request(metaclass=Metaclass_SolvePositionIK_Request):
    """
    Message class 'SolvePositionIK_Request'.

    Constants:
      SEED_AUTO
      SEED_USER
      SEED_CURRENT
      SEED_NS_MAP
    """

    __slots__ = [
        '_pose_stamp',
        '_seed_angles',
        '_seed_mode',
    ]

    _fields_and_field_types = {
        'pose_stamp': 'sequence<geometry_msgs/PoseStamped>',
        'seed_angles': 'sequence<sensor_msgs/JointState>',
        'seed_mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'JointState')),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.pose_stamp = kwargs.get('pose_stamp', [])
        self.seed_angles = kwargs.get('seed_angles', [])
        self.seed_mode = kwargs.get('seed_mode', int())

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
        if self.pose_stamp != other.pose_stamp:
            return False
        if self.seed_angles != other.seed_angles:
            return False
        if self.seed_mode != other.seed_mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pose_stamp(self):
        """Message field 'pose_stamp'."""
        return self._pose_stamp

    @pose_stamp.setter
    def pose_stamp(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
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
                 all(isinstance(v, PoseStamped) for v in value) and
                 True), \
                "The 'pose_stamp' field must be a set or sequence and each value of type 'PoseStamped'"
        self._pose_stamp = value

    @builtins.property
    def seed_angles(self):
        """Message field 'seed_angles'."""
        return self._seed_angles

    @seed_angles.setter
    def seed_angles(self, value):
        if __debug__:
            from sensor_msgs.msg import JointState
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
                 all(isinstance(v, JointState) for v in value) and
                 True), \
                "The 'seed_angles' field must be a set or sequence and each value of type 'JointState'"
        self._seed_angles = value

    @builtins.property
    def seed_mode(self):
        """Message field 'seed_mode'."""
        return self._seed_mode

    @seed_mode.setter
    def seed_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'seed_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'seed_mode' field must be an unsigned integer in [0, 255]"
        self._seed_mode = value


# Import statements for member types

# Member 'result_type'
import array  # noqa: E402, I100

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SolvePositionIK_Response(type):
    """Metaclass of message 'SolvePositionIK_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'RESULT_INVALID': 0,
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
                'baxter_core_msgs.srv.SolvePositionIK_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__solve_position_ik__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__solve_position_ik__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__solve_position_ik__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__solve_position_ik__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__solve_position_ik__response

            from sensor_msgs.msg import JointState
            if JointState.__class__._TYPE_SUPPORT is None:
                JointState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'RESULT_INVALID': cls.__constants['RESULT_INVALID'],
        }

    @property
    def RESULT_INVALID(self):
        """Message constant 'RESULT_INVALID'."""
        return Metaclass_SolvePositionIK_Response.__constants['RESULT_INVALID']


class SolvePositionIK_Response(metaclass=Metaclass_SolvePositionIK_Response):
    """
    Message class 'SolvePositionIK_Response'.

    Constants:
      RESULT_INVALID
    """

    __slots__ = [
        '_joints',
        '_is_valid',
        '_result_type',
    ]

    _fields_and_field_types = {
        'joints': 'sequence<sensor_msgs/JointState>',
        'is_valid': 'sequence<boolean>',
        'result_type': 'sequence<uint8>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'JointState')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.joints = kwargs.get('joints', [])
        self.is_valid = kwargs.get('is_valid', [])
        self.result_type = array.array('B', kwargs.get('result_type', []))

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
        if self.joints != other.joints:
            return False
        if self.is_valid != other.is_valid:
            return False
        if self.result_type != other.result_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def joints(self):
        """Message field 'joints'."""
        return self._joints

    @joints.setter
    def joints(self, value):
        if __debug__:
            from sensor_msgs.msg import JointState
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
                 all(isinstance(v, JointState) for v in value) and
                 True), \
                "The 'joints' field must be a set or sequence and each value of type 'JointState'"
        self._joints = value

    @builtins.property
    def is_valid(self):
        """Message field 'is_valid'."""
        return self._is_valid

    @is_valid.setter
    def is_valid(self, value):
        if __debug__:
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
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'is_valid' field must be a set or sequence and each value of type 'bool'"
        self._is_valid = value

    @builtins.property
    def result_type(self):
        """Message field 'result_type'."""
        return self._result_type

    @result_type.setter
    def result_type(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'result_type' array.array() must have the type code of 'B'"
            self._result_type = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'result_type' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._result_type = array.array('B', value)


class Metaclass_SolvePositionIK(type):
    """Metaclass of service 'SolvePositionIK'."""

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
                'baxter_core_msgs.srv.SolvePositionIK')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__solve_position_ik

            from baxter_core_msgs.srv import _solve_position_ik
            if _solve_position_ik.Metaclass_SolvePositionIK_Request._TYPE_SUPPORT is None:
                _solve_position_ik.Metaclass_SolvePositionIK_Request.__import_type_support__()
            if _solve_position_ik.Metaclass_SolvePositionIK_Response._TYPE_SUPPORT is None:
                _solve_position_ik.Metaclass_SolvePositionIK_Response.__import_type_support__()


class SolvePositionIK(metaclass=Metaclass_SolvePositionIK):
    from baxter_core_msgs.srv._solve_position_ik import SolvePositionIK_Request as Request
    from baxter_core_msgs.srv._solve_position_ik import SolvePositionIK_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
