// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from baxter_core_msgs:msg/CameraSettings.idl
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
#include "baxter_core_msgs/msg/detail/camera_settings__struct.h"
#include "baxter_core_msgs/msg/detail/camera_settings__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "baxter_core_msgs/msg/detail/camera_control__functions.h"
// end nested array functions include
bool baxter_core_msgs__msg__camera_control__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * baxter_core_msgs__msg__camera_control__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool baxter_core_msgs__msg__camera_settings__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[53];
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
    assert(strncmp("baxter_core_msgs.msg._camera_settings.CameraSettings", full_classname_dest, 52) == 0);
  }
  baxter_core_msgs__msg__CameraSettings * ros_message = _ros_message;
  {  // width
    PyObject * field = PyObject_GetAttrString(_pymsg, "width");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->width = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // height
    PyObject * field = PyObject_GetAttrString(_pymsg, "height");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->height = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // fps
    PyObject * field = PyObject_GetAttrString(_pymsg, "fps");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fps = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // controls
    PyObject * field = PyObject_GetAttrString(_pymsg, "controls");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'controls'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!baxter_core_msgs__msg__CameraControl__Sequence__init(&(ros_message->controls), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create baxter_core_msgs__msg__CameraControl__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    baxter_core_msgs__msg__CameraControl * dest = ros_message->controls.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!baxter_core_msgs__msg__camera_control__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * baxter_core_msgs__msg__camera_settings__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CameraSettings */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("baxter_core_msgs.msg._camera_settings");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CameraSettings");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  baxter_core_msgs__msg__CameraSettings * ros_message = (baxter_core_msgs__msg__CameraSettings *)raw_ros_message;
  {  // width
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fps
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // controls
    PyObject * field = NULL;
    size_t size = ros_message->controls.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    baxter_core_msgs__msg__CameraControl * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->controls.data[i]);
      PyObject * pyitem = baxter_core_msgs__msg__camera_control__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "controls", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
