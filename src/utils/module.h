#ifndef MODULE_H
#define MODULE_H

#include "timer.h"

namespace pose
{
class Module
{
public:
    Module();
    ~Module();

    float getLastTime() const;
    void process();

protected:
    virtual void iProcess() = 0;

private:
    Timer m_timer;
    float m_lastTime;
};
}

#endif // MODULE_H
