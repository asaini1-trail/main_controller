#ifndef ROVER_SW_PWRTRAIN_24B__VISIBILITY_CONTROL_H_
#define ROVER_SW_PWRTRAIN_24B__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROVER_SW_PWRTRAIN_24B_EXPORT __attribute__ ((dllexport))
    #define ROVER_SW_PWRTRAIN_24B_IMPORT __attribute__ ((dllimport))
  #else
    #define ROVER_SW_PWRTRAIN_24B_EXPORT __declspec(dllexport)
    #define ROVER_SW_PWRTRAIN_24B_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROVER_SW_PWRTRAIN_24B_BUILDING_LIBRARY
    #define ROVER_SW_PWRTRAIN_24B_PUBLIC ROVER_SW_PWRTRAIN_24B_EXPORT
  #else
    #define ROVER_SW_PWRTRAIN_24B_PUBLIC ROVER_SW_PWRTRAIN_24B_IMPORT
  #endif
  #define ROVER_SW_PWRTRAIN_24B_PUBLIC_TYPE ROVER_SW_PWRTRAIN_24B_PUBLIC
  #define ROVER_SW_PWRTRAIN_24B_LOCAL
#else
  #define ROVER_SW_PWRTRAIN_24B_EXPORT __attribute__ ((visibility("default")))
  #define ROVER_SW_PWRTRAIN_24B_IMPORT
  #if __GNUC__ >= 4
    #define ROVER_SW_PWRTRAIN_24B_PUBLIC __attribute__ ((visibility("default")))
    #define ROVER_SW_PWRTRAIN_24B_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROVER_SW_PWRTRAIN_24B_PUBLIC
    #define ROVER_SW_PWRTRAIN_24B_LOCAL
  #endif
  #define ROVER_SW_PWRTRAIN_24B_PUBLIC_TYPE
#endif
#endif  // ROVER_SW_PWRTRAIN_24B__VISIBILITY_CONTROL_H_
// Generated 16-Jan-2026 23:21:45
 