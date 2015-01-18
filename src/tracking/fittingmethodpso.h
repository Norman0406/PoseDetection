#ifndef FITTINGMETHODPSO_H
#define FITTINGMETHODPSO_H

#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "fittingmethod.h"

namespace pose
{

// see http://www.ints.info.hiroshima-cu.ac.jp/~takahama/download/PSO.html

// particle swarm optimization
class FittingMethodPSO
        : public FittingMethod
{
public:
    FittingMethodPSO();
    ~FittingMethodPSO();

protected:
    void iProcess(const cv::Mat& depthMap,
                  const cv::Mat& pointCloud,
                  std::shared_ptr<Skeleton> skeleton,
                  cv::Point3f centerOfMass,
                  const cv::Mat& projectionMatrix);

private:
    struct Particle
    {
        Particle(int numVar) {
            numVar = numVar;
            x = new float[numVar];
            v = new float[numVar];
            xStar = new float[numVar];
            f = 0;
            pBest = 0;
        }

        ~Particle() {
            delete[] x;
            delete[] v;
            delete[] xStar;
        }

        int numVar;     // number of variables
        float* x;       // state
        float* v;       // velocity
        float f;        // function to evaluate
        float pBest;
        float* xStar;

        void evaluate() {
            f = 0;
            for (int i = 0; i < numVar; i++)
                f += (x[i] - 1) * (x[i] - 1);
        }

        void updatePBest() {
            for (int i = 0; i < numVar; i++)
                xStar[i] = x[i];
            pBest = f;
        }
    };

    const Particle* initialize();

    std::vector<Particle*> m_particles;

    int m_numParticles;
    int m_numIterations;
    const int m_numVariables;

    // The value of inertia weight at t=0 (W_0) and t=T_MAX (W_T)
    const float m_w0;
    const float m_wt;
    const float m_maxV;
    // The cognitive parameter (c1) and the social parameter (c2)
    const float m_c1;
    const float m_c2;
};
}

#endif // FITTINGMETHODPSO_H
