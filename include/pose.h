#ifndef LIBPOSE_H
#define LIBPOSE_H

#ifdef __cplusplus
extern "C"{
#endif

/// If Win32, export all functions for DLL usage
#ifndef _WIN32
  #define POSEAPI /**< DLLExport information for windows, set to nothing on other platforms */
#else
  /**< DLLExport information for windows, set to nothing on other platforms */
  #ifdef __cplusplus
    #define POSEAPI extern "C" __declspec(dllexport)
  #else
    // this is required when building from a Win32 port of gcc without being
    // forced to compile all of the library files (.c) with g++...
    #define POSEAPI __declspec(dllexport)
  #endif
#endif

enum PoseResultCode
{
    POSE_SUCCESS = 0,
    POSE_FAIL
};

struct _IPoseContext;
typedef struct _IPoseContext IPoseContext;

POSEAPI int PoseInit(IPoseContext** context);

POSEAPI int PoseShutdown(IPoseContext* context);

#ifdef __cplusplus
}
#endif

#endif // LIBPOSE_H
