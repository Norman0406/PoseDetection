#ifndef INTERNAL_H
#define INTERNAL_H

typedef void CAlgorithm;

struct _PoseContext
{
    int width;
    int height;
    int depthFrameSize;
    int pointsFrameSize;
    CAlgorithm* algorithm;
};

#endif // INTERNAL_H
