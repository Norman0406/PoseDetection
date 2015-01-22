#include "module.h"
#include <iostream>

namespace pose
{
Module::Module(std::string name)
    : m_name(name),
      m_lastTime(0),
      m_sumTime(0),
      m_minTime(-1),
      m_maxTime(-1),
      m_iterations(0)
{
}

Module::~Module()
{
    if (m_iterations > 0) {
        float avg = m_sumTime / (float)m_iterations;
        std::cout << "[" << m_name << "]: (Avg, Min, Max) = (" << avg << ", " << m_minTime << ", " << m_maxTime << ")" << std::endl;
    }
}

float Module::getLastTime() const
{
    return m_lastTime;
}

void Module::begin()
{
    m_timer.reset();
    m_timer.start();
}

void Module::end()
{
    m_timer.stop();
    m_lastTime = m_timer.getDiffMS();
    m_sumTime += m_lastTime;

    if (m_minTime < 0 || m_lastTime < m_minTime)
        m_minTime = m_lastTime;

    if (m_maxTime < 0 || m_lastTime > m_maxTime)
        m_maxTime = m_lastTime;

    m_iterations++;
}
}
