#include <pose.h>

POSEAPI int PoseInit(IPoseContext** context)
{
    *context = 0;
    return 0;
}

POSEAPI int PoseShutdown(IPoseContext* context)
{
    return 0;
}
