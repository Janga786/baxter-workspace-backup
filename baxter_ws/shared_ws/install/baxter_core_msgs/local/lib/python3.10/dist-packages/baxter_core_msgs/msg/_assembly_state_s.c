// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from baxter_core_msgs:msg/AssemblyState.idl
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
#include "baxter_core_msgs/msg/detail/assembly_state__struct.h"
#include "baxter_core_msgs/msg/detail/assembly_state__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool baxter_core_msgs__msg__assembly_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
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
    assert(strncmp("baxter_core_msgs.msg._assembly_state.AssemblyState", full_classname_dest, 50) == 0);
  }
  baxter_core_msgs__msg__AssemblyState * ros_message = _ros_message;
  {  // ready
    PyObject * field = PyObject_GetAttrString(_pymsg, "ready");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->ready = (Py_True == field);
    Py_DECREF(field);
  }
  {  // enabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "enabled");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->enabled = (Py_True == field);
    Py_DECREF(field);
  }
  {  // stopped
    PyObject * field = PyObject_GetAttrString(_pymsg, "stopped");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->stopped = (Py_True == field);
    Py_DECREF(field);
  }
  {  // error
    PyObject * field = PyObject_GetAttrString(_pymsg, "error");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->error = (Py_True == field);
    Py_DECREF(field);
  }
  {  // estop_button
    PyObject * field = PyObject_GetAttrString(_pymsg, "estop_button");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->estop_button = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // estop_source
    PyObject * field = PyObject_GetAttrString(_pymsg, "estop_source");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->estop_source = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * baxter_core_msgs__msg__assembly_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of AssemblyState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("baxter_core_msgs.msg._assembly_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "AssemblyState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  baxter_core_msgs__msg__AssemblyState * ros_message = (baxter_core_msgs__msg__AssemblyState *)raw_ros_message;
  {  // ready
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->ready ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ready", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enabled
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->enabled ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stopped
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->stopped ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stopped", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->error ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // estop_button
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->estop_button);
    {
      int rc = PyObject_SetAttrString(_pymessage, "estop_button", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // estop_source
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->estop_source);
    {
      int rc = PyObject_SetAttrString(_pymessage, "estop_source", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
