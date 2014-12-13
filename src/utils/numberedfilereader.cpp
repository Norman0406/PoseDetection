#include "numberedfilereader.h"
#include "utils.h"

namespace pose
{
NumberedFileReader::NumberedFileReader(std::string format)
    : m_format(format)
{
    reset();
}

NumberedFileReader::NumberedFileReader(std::string format, bool loop)
    : m_format(format)
{
    reset();
    setLoop(loop);
}

NumberedFileReader::NumberedFileReader(std::string format, int startFrame)
    : m_format(format)
{
    reset();
    setStartFrame(startFrame);
}

NumberedFileReader::NumberedFileReader(std::string format, int startFrame, int endFrame)
    : m_format(format)
{
    reset();
    setStartFrame(startFrame);
    setEndFrame(endFrame);
}

NumberedFileReader::NumberedFileReader(std::string format, int startFrame, int endFrame, bool loop)
    : m_format(format)
{
    reset();
    setStartFrame(startFrame);
    setEndFrame(endFrame);
    setLoop(loop);
}

void NumberedFileReader::setLoop(bool loop)
{
    m_loop = loop;
}

void NumberedFileReader::setStartFrame(int frame)
{
    m_startFrame = frame;
    m_fileIndex = m_startFrame;
}

void NumberedFileReader::setEndFrame(int frame)
{
    m_endFrame = frame;
}

int NumberedFileReader::getFrameIndex() const
{
    return m_fileIndex;
}

void NumberedFileReader::reset()
{
    m_loop = false;
    m_startFrame = 0;
    m_endFrame = -1;
    m_fileIndex = 0;
}

bool NumberedFileReader::read(cv::Mat& image)
{
    if (m_fileIndex == m_endFrame) {
        if (m_loop)
            m_fileIndex = m_startFrame;
        else
            return false;
    }

    sprintf_s(m_buffer, m_format.c_str(), m_fileIndex);

    bool fileFound = Utils::loadCvMat(m_buffer, image);

    if (!fileFound) {
        if (m_loop)
            m_fileIndex = m_startFrame;
        else
            return false;
    }
    else
        m_fileIndex++;

    return true;
}
}
