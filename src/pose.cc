#include <stdlib.h>
#include <string.h>

#include <pose.h>
#include "algorithm.h"
#include "internal.h"
#include <utils/exception.h>

POSEAPI PoseResult poseInit(PoseContext** context, int width, int height)
{
    *context = (PoseContext*)malloc(sizeof(PoseContext));
    if (*context == NULL)
        return RESULT_OUTOFMEMORY;

    memset(*context, 0, sizeof(PoseContext));

    (*context)->width = width;
    (*context)->height = height;
    (*context)->depthFrameSize = (*context)->width * (*context)->height;
    (*context)->pointsFrameSize = (*context)->width * (*context)->height * 3;
    (*context)->algorithm = (CAlgorithm*)(new pose::Algorithm(width, height));

    if ((*context)->algorithm == NULL)
        return RESULT_OUTOFMEMORY;

    return RESULT_SUCCESS;
}

POSEAPI PoseResult poseShutdown(PoseContext* context)
{
    if (context == NULL)
        return RESULT_INVALIDCONTEXT;

    delete (pose::Algorithm*)(context->algorithm);
    free(context);
    return RESULT_SUCCESS;
}

POSEAPI PoseResult poseSetInput(PoseContext* context, float* depthData, int depthDataSize, float* pointsData, int pointsDataSize)
{
    if (context == NULL)
        return RESULT_INVALIDCONTEXT;

    if (depthData == NULL ||
        pointsData == NULL ||
        depthDataSize != context->depthFrameSize ||
        pointsDataSize != context->pointsFrameSize)
        return RESULT_INVALIDPARAMETERS;

    try {
        if (!((pose::Algorithm*)(context->algorithm))->process(depthData, depthDataSize, pointsData, pointsDataSize))
            return RESULT_FINISHED;
    }
    catch (const pose::Exception& exception) {
        printf("Exception: %s", exception.what());
        return RESULT_INTERNALERROR;
    }
    catch (...) {
        printf("Unhandled Exception");
        return RESULT_UNHANDLEDEXCEPTION;
    }

    return RESULT_SUCCESS;
}

POSEAPI PoseResult poseGetScene(PoseContext* context, PoseScene** scene)
{
    if (context == NULL)
        return RESULT_INVALIDCONTEXT;

    if (*scene != NULL)
        return RESULT_INVALIDPARAMETERS;

    *scene = (PoseScene*)malloc(sizeof(PoseScene));
    if (*scene == NULL)
        return RESULT_OUTOFMEMORY;

    memset(*scene, 0, sizeof(PoseScene));

    (*scene)->skeletons = (PoseSkeleton*)malloc((*scene)->numSkeletons * sizeof(PoseSkeleton));

    return RESULT_SUCCESS;
}

POSEAPI PoseResult poseFreeScene(PoseScene* scene)
{
    if (scene == NULL)
        return RESULT_INVALIDCONTEXT;

    free(scene->skeletons);
    return RESULT_SUCCESS;
}

POSEAPI PoseResult poseGetImage(PoseContext* context, PoseImageType type, int* width, int* height, int* size, void* data)
{
    if (context == NULL)
        return RESULT_INVALIDCONTEXT;

    return RESULT_SUCCESS;
}
