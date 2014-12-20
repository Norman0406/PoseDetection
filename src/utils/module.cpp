#include "module.h"

namespace pose
{
Module::Module()
    : m_lastTime(0)
{
}

Module::~Module()
{
}

float Module::getLastTime() const
{
    return m_lastTime;
}

void Module::process()
{
    m_timer.reset();
    m_timer.start();

    iProcess();

    m_timer.stop();
    m_lastTime = m_timer.getDiffMS();
}
}
