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

typedef struct
{
    float x;
    float y;
    float z;
} PoseVector3;

typedef struct
{
    float x;
    float y;
} PoseVector2;

typedef enum
{
    RESULT_SUCCESS = 0,
    RESULT_OUTOFMEMORY = -1,
    RESULT_INVALIDCONTEXT = -2,
    RESULT_INVALIDPARAMETERS = -3,
    RESULT_INTERNALERROR = -4,
    RESULT_UNHANDLEDEXCEPTION = -5,
    RESULT_FINISHED = -6
} PoseResult;

typedef enum
{
    IMAGE_DEPTH = 0,
    IMAGE_POINTS,
    IMAGE_USERSEGMENTATION,
    IMAGE_BACKGROUND,
    IMAGE_FOREGROUND,
    IMAGE_REGIONS,
    IMAGE_NUMTYPES
} PoseImageType;

typedef enum
{
    JT_HEAD = 0,
    JT_NECK,
    JT_LEFTSHOULDER,
    JT_LEFTELBOW,
    JT_LEFTHAND,
    JT_RIGHTSHOULDER,
    JT_RIGHTELBOW,
    JT_RIGHTHAND,
    JT_TORSO,
    JT_HIPS,
    JT_LEFTHIP,
    JT_LEFTKNEE,
    JT_LEFTFOOT,
    JT_RIGHTHIP,
    JT_RIGHTKNEE,
    JT_RIGHTFOOT
} PoseJointType;

struct _PoseContext;
typedef struct _PoseContext PoseContext;

struct _PoseJoint
{
    PoseJointType jointType;
    PoseVector3 position3d;
    PoseVector2 position2d;
};

struct _PoseSkeleton
{
    int id;
};
typedef struct _PoseSkeleton PoseSkeleton;

struct _PoseScene
{
    PoseSkeleton* skeletons;
    int numSkeletons;
};
typedef struct _PoseScene PoseScene;

POSEAPI PoseResult poseInit(PoseContext** context, int width, int height);

POSEAPI PoseResult poseShutdown(PoseContext* context);

POSEAPI PoseResult poseSetInput(PoseContext* context, float* depthData, int depthDataSize, float* pointsData, int pointsDataSize);

POSEAPI PoseResult poseGetScene(PoseContext* context, PoseScene** scene);

POSEAPI PoseResult poseFreeScene(PoseScene* scene);

POSEAPI PoseResult poseGetImage(PoseContext* context, PoseImageType type, int* width, int* height, int* size, void** data);

#ifdef __cplusplus
}
#endif

#endif // LIBPOSE_H
