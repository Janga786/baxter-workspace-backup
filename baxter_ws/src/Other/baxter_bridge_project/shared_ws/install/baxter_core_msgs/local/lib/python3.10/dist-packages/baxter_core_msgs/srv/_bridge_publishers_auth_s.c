// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from baxter_core_msgs:srv/BridgePublishersAuth.idl
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
#include "baxter_core_msgs/srv/detail/bridge_publishers_auth__struct.h"
#include "baxter_core_msgs/srv/detail/bridge_publishers_auth__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool baxter_core_msgs__srv__bridge_publishers_auth__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[74];
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
    assert(strncmp("baxter_core_msgs.srv._bridge_publishers_auth.BridgePublishersAuth_Request", full_classname_dest, 73) == 0);
  }
  baxter_core_msgs__srv__BridgePublishersAuth_Request * ros_message = _ros_message;
  {  // topic
    PyObject * field = PyObject_GetAttrString(_pymsg, "topic");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->topic, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // user
    PyObject * field = PyObject_GetAttrString(_pymsg, "user");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->user, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * baxter_core_msgs__srv__bridge_publishers_auth__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BridgePublishersAuth_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("baxter_core_msgs.srv._bridge_publishers_auth");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BridgePublishersAuth_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  baxter_core_msgs__srv__BridgePublishersAuth_Request * ros_message = (baxter_core_msgs__srv__BridgePublishersAuth_Request *)raw_ros_message;
  {  // topic
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->topic.data,
      strlen(ros_message->topic.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "topic", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // user
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->user.data,
      strlen(ros_message->user.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "user", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "baxter_core_msgs/srv/detail/bridge_publishers_auth__struct.h"
// already included above
// #include "baxter_core_msgs/srv/detail/bridge_publishers_auth__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// already included above
// #include "rosidl_runtime_c/string.h"
// already included above
// #include "rosidl_runtime_c/string_functions.h"

// Nested array functions includes
#include "baxter_core_msgs/msg/detail/bridge_publisher__functions.h"
// end nested array functions include
bool baxter_core_msgs__msg__bridge_publisher__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * baxter_core_msgs__msg__bridge_publisher__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool baxter_core_msgs__srv__bridge_publishers_auth__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[75];
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
    assert(strncmp("baxter_core_msgs.srv._bridge_publishers_auth.BridgePublishersAuth_Response", full_classname_dest, 74) == 0);
  }
  baxter_core_msgs__srv__BridgePublishersAuth_Response * ros_message = _ros_message;
  {  // publishers
    PyObject * field = PyObject_GetAttrString(_pymsg, "publishers");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'publishers'");
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
    if (!baxter_core_msgs__msg__BridgePublisher__Sequence__init(&(ros_message->publishers), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create baxter_core_msgs__msg__BridgePublisher__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    baxter_core_msgs__msg__BridgePublisher * dest = ros_message->publishers.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!baxter_core_msgs__msg__bridge_publisher__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // forced_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "forced_left");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->forced_left, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // forced_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "forced_right");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->forced_right, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * baxter_core_msgs__srv__bridge_publishers_auth__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BridgePublishersAuth_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("baxter_core_msgs.srv._bridge_publishers_auth");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BridgePublishersAuth_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  baxter_core_msgs__srv__BridgePublishersAuth_Response * ros_message = (baxter_core_msgs__srv__BridgePublishersAuth_Response *)raw_ros_message;
  {  // publishers
    PyObject * field = NULL;
    size_t size = ros_message->publishers.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    baxter_core_msgs__msg__BridgePublisher * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->publishers.data[i]);
      PyObject * pyitem = baxter_core_msgs__msg__bridge_publisher__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "publishers", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // forced_left
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->forced_left.data,
      strlen(ros_message->forced_left.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "forced_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // forced_right
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->forced_right.data,
      strlen(ros_message->forced_right.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "forced_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
