#ifndef NUMBEREDFILEWRITER_H
#define NUMBEREDFILEWRITER_H

#include <string>
#include <opencv2/opencv.hpp>

namespace pose
{
class NumberedFileWriter
{
public:
    NumberedFileWriter(std::string format);

    void reset();
    void write(const cv::Mat&);

private:
    std::string m_format;
    int m_fileIndex;
    char m_buffer[512];
};
}

#endif // NUMBEREDFILEWRITER_H
