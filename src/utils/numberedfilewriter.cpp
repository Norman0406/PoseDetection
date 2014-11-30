#include "numberedfilewriter.h"
#include "utils.h"

namespace pose
{
NumberedFileWriter::NumberedFileWriter(std::string format)
    : m_format(format)
{
    reset();
}

void NumberedFileWriter::reset()
{
    m_fileIndex = 0;
}

void NumberedFileWriter::write(const cv::Mat& image)
{
    sprintf_s(m_buffer, m_format.c_str(), m_fileIndex);
    Utils::saveCvMat(m_buffer, image);
    m_fileIndex++;
}

}
