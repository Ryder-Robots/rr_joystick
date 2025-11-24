#ifndef RR_JOYSTICK__VISIBILITY_CONTROL_H_
#define RR_JOYSTICK__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RR_JOYSTICK_EXPORT __attribute__ ((dllexport))
    #define RR_JOYSTICK_IMPORT __attribute__ ((dllimport))
  #else
    #define RR_JOYSTICK_EXPORT __declspec(dllexport)
    #define RR_JOYSTICK_IMPORT __declspec(dllimport)
  #endif
  #ifdef RR_JOYSTICK_BUILDING_LIBRARY
    #define RR_JOYSTICK_PUBLIC RR_JOYSTICK_EXPORT
  #else
    #define RR_JOYSTICK_PUBLIC RR_JOYSTICK_IMPORT
  #endif
  #define RR_JOYSTICK_PUBLIC_TYPE RR_JOYSTICK_PUBLIC
  #define RR_JOYSTICK_LOCAL
#else
  #define RR_JOYSTICK_EXPORT __attribute__ ((visibility("default")))
  #define RR_JOYSTICK_IMPORT
  #if __GNUC__ >= 4
    #define RR_JOYSTICK_PUBLIC __attribute__ ((visibility("default")))
    #define RR_JOYSTICK_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RR_JOYSTICK_PUBLIC
    #define RR_JOYSTICK_LOCAL
  #endif
  #define RR_JOYSTICK_PUBLIC_TYPE
#endif

#endif  // RR_JOYSTICK__VISIBILITY_CONTROL_H_
