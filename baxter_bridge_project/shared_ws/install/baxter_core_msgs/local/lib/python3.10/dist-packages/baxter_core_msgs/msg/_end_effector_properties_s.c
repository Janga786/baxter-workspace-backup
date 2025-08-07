// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from baxter_core_msgs:msg/EndEffectorProperties.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "baxter_core_msgs/msg/detail/end_effector_properties__struct.h"
#include "baxter_core_msgs/msg/detail/end_effector_properties__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool baxter_core_msgs__msg__end_effector_properties__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[68];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("baxter_core_msgs.msg._end_effector_properties.EndEffectorProperties", full_classname_dest, 67) == 0);
  }
  baxter_core_msgs__msg__EndEffectorProperties * ros_message = _ros_message;
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // ui_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "ui_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ui_type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // manufacturer
    PyObject * field = PyObject_GetAttrString(_pymsg, "manufacturer");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->manufacturer, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // product
    PyObject * field = PyObject_GetAttrString(_pymsg, "product");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->product, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // serial_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "serial_number");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->serial_number, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // hardware_rev
    PyObject * field = PyObject_GetAttrString(_pymsg, "hardware_rev");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->hardware_rev, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // firmware_rev
    PyObject * field = PyObject_GetAttrString(_pymsg, "firmware_rev");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->firmware_rev, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // firmware_date
    PyObject * field = PyObject_GetAttrString(_pymsg, "firmware_date");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->firmware_date, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // has_calibration
    PyObject * field = PyObject_GetAttrString(_pymsg, "has_calibration");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->has_calibration = (Py_True == field);
    Py_DECREF(field);
  }
  {  // controls_grip
    PyObject * field = PyObject_GetAttrString(_pymsg, "controls_grip");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->controls_grip = (Py_True == field);
    Py_DECREF(field);
  }
  {  // senses_grip
    PyObject * field = PyObject_GetAttrString(_pymsg, "senses_grip");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->senses_grip = (Py_True == field);
    Py_DECREF(field);
  }
  {  // reverses_grip
    PyObject * field = PyObject_GetAttrString(_pymsg, "reverses_grip");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->reverses_grip = (Py_True == field);
    Py_DECREF(field);
  }
  {  // controls_force
    PyObject * field = PyObject_GetAttrString(_pymsg, "controls_force");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->controls_force = (Py_True == field);
    Py_DECREF(field);
  }
  {  // senses_force
    PyObject * field = PyObject_GetAttrString(_pymsg, "senses_force");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->senses_force = (Py_True == field);
    Py_DECREF(field);
  }
  {  // controls_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "controls_position");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->controls_position = (Py_True == field);
    Py_DECREF(field);
  }
  {  // senses_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "senses_position");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->senses_position = (Py_True == field);
    Py_DECREF(field);
  }
  {  // properties
    PyObject * field = PyObject_GetAttrString(_pymsg, "properties");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->properties, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * baxter_core_msgs__msg__end_effector_properties__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of EndEffectorProperties */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("baxter_core_msgs.msg._end_effector_properties");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "EndEffectorProperties");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  baxter_core_msgs__msg__EndEffectorProperties * ros_message = (baxter_core_msgs__msg__EndEffectorProperties *)raw_ros_message;
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ui_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->ui_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ui_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // manufacturer
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->manufacturer.data,
      strlen(ros_message->manufacturer.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "manufacturer", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // product
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->product.data,
      strlen(ros_message->product.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "product", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // serial_number
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->serial_number.data,
      strlen(ros_message->serial_number.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "serial_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hardware_rev
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->hardware_rev.data,
      strlen(ros_message->hardware_rev.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "hardware_rev", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // firmware_rev
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->firmware_rev.data,
      strlen(ros_message->firmware_rev.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "firmware_rev", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // firmware_date
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->firmware_date.data,
      strlen(ros_message->firmware_date.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "firmware_date", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // has_calibration
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->has_calibration ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "has_calibration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // controls_grip
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->controls_grip ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "controls_grip", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // senses_grip
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->senses_grip ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "senses_grip", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reverses_grip
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->reverses_grip ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "reverses_grip", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // controls_force
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->controls_force ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "controls_force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // senses_force
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->senses_force ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "senses_force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // controls_position
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->controls_position ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "controls_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // senses_position
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->senses_position ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "senses_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // properties
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->properties.data,
      strlen(ros_message->properties.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "properties", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
