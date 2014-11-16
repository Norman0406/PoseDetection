#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>
#include <string>

namespace pose
{
class Exception
        : std::exception
{
public:
    Exception(std::string);
    virtual ~Exception() throw();

    virtual const char* what() const throw();

private:
    const std::string m_message;
};
}

#endif // CONNECTEDCOMPONENT_H
