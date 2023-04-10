// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef PUB_CPP__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define PUB_CPP__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_pub_cpp __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_pub_cpp __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_pub_cpp __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_pub_cpp __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_pub_cpp
    #define ROSIDL_GENERATOR_C_PUBLIC_pub_cpp ROSIDL_GENERATOR_C_EXPORT_pub_cpp
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_pub_cpp ROSIDL_GENERATOR_C_IMPORT_pub_cpp
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_pub_cpp __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_pub_cpp
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_pub_cpp __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // PUB_CPP__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
