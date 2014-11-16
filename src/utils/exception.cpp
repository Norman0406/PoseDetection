#include "exception.h"
#include <iostream>

namespace pose
{
Exception::Exception(std::string msg)
    : m_message(msg)
{
    std::cerr << "Exception: " << m_message << std::endl;
}

Exception::~Exception(void) throw()
{
}

const char* Exception::what() const throw()
{
    return m_message.c_str();
}
}
