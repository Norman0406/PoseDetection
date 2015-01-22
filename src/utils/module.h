#ifndef MODULE_H
#define MODULE_H

#include "timer.h"
#include <string>

namespace pose
{
class Module
{
public:
    virtual ~Module();

    float getLastTime() const;

protected:
    Module(std::string name);

    void begin();
    void end();

private:
    std::string m_name;
    Timer m_timer;
    float m_lastTime;
    float m_sumTime;
    float m_minTime;
    float m_maxTime;
    int m_iterations;
};
}

#endif // MODULE_H
