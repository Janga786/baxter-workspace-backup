# generated from rosidl_generator_py/resource/_idl.py.em
# with input from baxter_core_msgs:msg/EndEffectorProperties.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EndEffectorProperties(type):
    """Metaclass of message 'EndEffectorProperties'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NO_GRIPPER': 0,
        'SUCTION_CUP_GRIPPER': 1,
        'ELECTRIC_GRIPPER': 2,
        'PASSIVE_GRIPPER': 3,
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
                'baxter_core_msgs.msg.EndEffectorProperties')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__end_effector_properties
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__end_effector_properties
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__end_effector_properties
            cls._TYPE_SUPPORT = module.type_support_msg__msg__end_effector_properties
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__end_effector_properties

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NO_GRIPPER': cls.__constants['NO_GRIPPER'],
            'SUCTION_CUP_GRIPPER': cls.__constants['SUCTION_CUP_GRIPPER'],
            'ELECTRIC_GRIPPER': cls.__constants['ELECTRIC_GRIPPER'],
            'PASSIVE_GRIPPER': cls.__constants['PASSIVE_GRIPPER'],
        }

    @property
    def NO_GRIPPER(self):
        """Message constant 'NO_GRIPPER'."""
        return Metaclass_EndEffectorProperties.__constants['NO_GRIPPER']

    @property
    def SUCTION_CUP_GRIPPER(self):
        """Message constant 'SUCTION_CUP_GRIPPER'."""
        return Metaclass_EndEffectorProperties.__constants['SUCTION_CUP_GRIPPER']

    @property
    def ELECTRIC_GRIPPER(self):
        """Message constant 'ELECTRIC_GRIPPER'."""
        return Metaclass_EndEffectorProperties.__constants['ELECTRIC_GRIPPER']

    @property
    def PASSIVE_GRIPPER(self):
        """Message constant 'PASSIVE_GRIPPER'."""
        return Metaclass_EndEffectorProperties.__constants['PASSIVE_GRIPPER']


class EndEffectorProperties(metaclass=Metaclass_EndEffectorProperties):
    """
    Message class 'EndEffectorProperties'.

    Constants:
      NO_GRIPPER
      SUCTION_CUP_GRIPPER
      ELECTRIC_GRIPPER
      PASSIVE_GRIPPER
    """

    __slots__ = [
        '_id',
        '_ui_type',
        '_manufacturer',
        '_product',
        '_serial_number',
        '_hardware_rev',
        '_firmware_rev',
        '_firmware_date',
        '_has_calibration',
        '_controls_grip',
        '_senses_grip',
        '_reverses_grip',
        '_controls_force',
        '_senses_force',
        '_controls_position',
        '_senses_position',
        '_properties',
    ]

    _fields_and_field_types = {
        'id': 'uint32',
        'ui_type': 'uint8',
        'manufacturer': 'string',
        'product': 'string',
        'serial_number': 'string',
        'hardware_rev': 'string',
        'firmware_rev': 'string',
        'firmware_date': 'string',
        'has_calibration': 'boolean',
        'controls_grip': 'boolean',
        'senses_grip': 'boolean',
        'reverses_grip': 'boolean',
        'controls_force': 'boolean',
        'senses_force': 'boolean',
        'controls_position': 'boolean',
        'senses_position': 'boolean',
        'properties': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        self.ui_type = kwargs.get('ui_type', int())
        self.manufacturer = kwargs.get('manufacturer', str())
        self.product = kwargs.get('product', str())
        self.serial_number = kwargs.get('serial_number', str())
        self.hardware_rev = kwargs.get('hardware_rev', str())
        self.firmware_rev = kwargs.get('firmware_rev', str())
        self.firmware_date = kwargs.get('firmware_date', str())
        self.has_calibration = kwargs.get('has_calibration', bool())
        self.controls_grip = kwargs.get('controls_grip', bool())
        self.senses_grip = kwargs.get('senses_grip', bool())
        self.reverses_grip = kwargs.get('reverses_grip', bool())
        self.controls_force = kwargs.get('controls_force', bool())
        self.senses_force = kwargs.get('senses_force', bool())
        self.controls_position = kwargs.get('controls_position', bool())
        self.senses_position = kwargs.get('senses_position', bool())
        self.properties = kwargs.get('properties', str())

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
        if self.ui_type != other.ui_type:
            return False
        if self.manufacturer != other.manufacturer:
            return False
        if self.product != other.product:
            return False
        if self.serial_number != other.serial_number:
            return False
        if self.hardware_rev != other.hardware_rev:
            return False
        if self.firmware_rev != other.firmware_rev:
            return False
        if self.firmware_date != other.firmware_date:
            return False
        if self.has_calibration != other.has_calibration:
            return False
        if self.controls_grip != other.controls_grip:
            return False
        if self.senses_grip != other.senses_grip:
            return False
        if self.reverses_grip != other.reverses_grip:
            return False
        if self.controls_force != other.controls_force:
            return False
        if self.senses_force != other.senses_force:
            return False
        if self.controls_position != other.controls_position:
            return False
        if self.senses_position != other.senses_position:
            return False
        if self.properties != other.properties:
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
    def ui_type(self):
        """Message field 'ui_type'."""
        return self._ui_type

    @ui_type.setter
    def ui_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ui_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'ui_type' field must be an unsigned integer in [0, 255]"
        self._ui_type = value

    @builtins.property
    def manufacturer(self):
        """Message field 'manufacturer'."""
        return self._manufacturer

    @manufacturer.setter
    def manufacturer(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'manufacturer' field must be of type 'str'"
        self._manufacturer = value

    @builtins.property
    def product(self):
        """Message field 'product'."""
        return self._product

    @product.setter
    def product(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'product' field must be of type 'str'"
        self._product = value

    @builtins.property
    def serial_number(self):
        """Message field 'serial_number'."""
        return self._serial_number

    @serial_number.setter
    def serial_number(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'serial_number' field must be of type 'str'"
        self._serial_number = value

    @builtins.property
    def hardware_rev(self):
        """Message field 'hardware_rev'."""
        return self._hardware_rev

    @hardware_rev.setter
    def hardware_rev(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'hardware_rev' field must be of type 'str'"
        self._hardware_rev = value

    @builtins.property
    def firmware_rev(self):
        """Message field 'firmware_rev'."""
        return self._firmware_rev

    @firmware_rev.setter
    def firmware_rev(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'firmware_rev' field must be of type 'str'"
        self._firmware_rev = value

    @builtins.property
    def firmware_date(self):
        """Message field 'firmware_date'."""
        return self._firmware_date

    @firmware_date.setter
    def firmware_date(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'firmware_date' field must be of type 'str'"
        self._firmware_date = value

    @builtins.property
    def has_calibration(self):
        """Message field 'has_calibration'."""
        return self._has_calibration

    @has_calibration.setter
    def has_calibration(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'has_calibration' field must be of type 'bool'"
        self._has_calibration = value

    @builtins.property
    def controls_grip(self):
        """Message field 'controls_grip'."""
        return self._controls_grip

    @controls_grip.setter
    def controls_grip(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'controls_grip' field must be of type 'bool'"
        self._controls_grip = value

    @builtins.property
    def senses_grip(self):
        """Message field 'senses_grip'."""
        return self._senses_grip

    @senses_grip.setter
    def senses_grip(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'senses_grip' field must be of type 'bool'"
        self._senses_grip = value

    @builtins.property
    def reverses_grip(self):
        """Message field 'reverses_grip'."""
        return self._reverses_grip

    @reverses_grip.setter
    def reverses_grip(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'reverses_grip' field must be of type 'bool'"
        self._reverses_grip = value

    @builtins.property
    def controls_force(self):
        """Message field 'controls_force'."""
        return self._controls_force

    @controls_force.setter
    def controls_force(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'controls_force' field must be of type 'bool'"
        self._controls_force = value

    @builtins.property
    def senses_force(self):
        """Message field 'senses_force'."""
        return self._senses_force

    @senses_force.setter
    def senses_force(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'senses_force' field must be of type 'bool'"
        self._senses_force = value

    @builtins.property
    def controls_position(self):
        """Message field 'controls_position'."""
        return self._controls_position

    @controls_position.setter
    def controls_position(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'controls_position' field must be of type 'bool'"
        self._controls_position = value

    @builtins.property
    def senses_position(self):
        """Message field 'senses_position'."""
        return self._senses_position

    @senses_position.setter
    def senses_position(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'senses_position' field must be of type 'bool'"
        self._senses_position = value

    @builtins.property
    def properties(self):
        """Message field 'properties'."""
        return self._properties

    @properties.setter
    def properties(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'properties' field must be of type 'str'"
        self._properties = value
