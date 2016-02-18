#include "fittingmethodpso.h"

namespace pose
{
FittingMethodPSO::FittingMethodPSO()
    : m_numParticles(10),
      m_numIterations(2),
      m_numVariables(3),
      m_w0(0.9f),
      m_wt(0.4f),
      m_maxV(2.0f),
      m_c1(2.0f),
      m_c2(2.0f)
{
    srand(time(NULL));

    // create and initialize particles
    m_particles.resize(m_numParticles);
    for (size_t i = 0; i < m_particles.size(); i++)
        m_particles[i] = new Particle(m_numVariables);
}

FittingMethodPSO::~FittingMethodPSO()
{
    for (size_t i = 0; i < m_particles.size(); i++)
        delete m_particles[i];
    m_particles.clear();
}

void FittingMethodPSO::iProcess(const cv::Mat& depthMap,
                                const cv::Mat& pointCloud,
                                std::shared_ptr<Skeleton> skeleton,
                                const cv::Mat& projectionMatrix)
{
    // just temporary
    const cv::Point3f& pos = skeleton->getPosition();

    // TODO: define the function and the actual parameters that are to be optimized.
    // The optimization is run for the 3d root joint position and every subsequent
    // bone that is not fixed. The energy is computed from the current bone to the
    // end of the skeleton hierarchy as the squared distance from the skeleton to
    // the nearest points in the point cloud.

    const Particle* bestParticle = initialize(pos, skeleton, pointCloud, projectionMatrix);

    float w = m_w0;

    // run particle swarm optimization
    for (int i = 0; i < m_numIterations; i++) {
        for (int j = 0; j < m_numParticles; j++) {
            Particle* particle = m_particles[j];

            // update velocity
            for (int k = 0; k < m_numVariables; k++) {
                particle->v[k] = w * particle->v[k] +
                        m_c1 * (rand() / (float)RAND_MAX) * (particle->xStar[k] - particle->x[k]) +
                        m_c2 * (rand() / (float)RAND_MAX) * (bestParticle->xStar[k] - particle->x[k]);

                if (particle->v[k] < -m_maxV)
                    particle->v[k] = -m_maxV;
                else if (particle->v[k] > m_maxV)
                    particle->v[k] = m_maxV;

                particle->x[k] += particle->v[k];
            }

            //particle->evaluate();

            cv::Point3f pos(particle->x[0], particle->x[1], particle->x[2]);
            skeleton->setPosition(pos);
            particle->f = updateSkeleton(skeleton, pointCloud, projectionMatrix);

            if (particle->f < particle->pBest) {
                if (particle->f < bestParticle->pBest)
                    bestParticle = particle;
                particle->updatePBest();
            }
        }

        w -= (m_w0 - m_wt) / m_numIterations;
    }

    cv::Point3f bestPos(bestParticle->x[0], bestParticle->x[1], bestParticle->x[2]);
    skeleton->setPosition(bestPos);
    updateSkeleton(skeleton, pointCloud, projectionMatrix);
}

const FittingMethodPSO::Particle* FittingMethodPSO::initialize(const cv::Point3f& pos, std::shared_ptr<Skeleton> skeleton, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix)
{
    Particle* bestParticle = 0;

    for (size_t i = 0; i < m_particles.size(); i++) {
        Particle* particle = m_particles[i];

        // NOTE: problem dependent

        cv::Point3f randomOffset((rand() / (float)RAND_MAX) - 0.5f,
                (rand() / (float)RAND_MAX) - 0.5f,
                (rand() / (float)RAND_MAX) - 0.5f);
        cv::Point3f newPos = randomOffset * 0.01f + pos;

        particle->x[0] = newPos.x;
        particle->x[1] = newPos.y;
        particle->x[2] = newPos.z;

        for (int j = 0; j < m_numVariables; j++) {
            //particle->x[j] = (float)rand();
            particle->v[j] = 0.0f;
        }

        //particle->evaluate();

        skeleton->setPosition(newPos);
        particle->f = updateSkeleton(skeleton, pointCloud, projectionMatrix);

        particle->updatePBest();

        if (!bestParticle || particle->f < bestParticle->f)
            bestParticle = particle;
    }

    return bestParticle;
}

void FittingMethodPSO::optimizeBone(std::shared_ptr<Bone> bone)
{

}
}
