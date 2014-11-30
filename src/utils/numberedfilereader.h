#ifndef NUMBEREDFILEREADER_H
#define NUMBEREDFILEREADER_H

#include <string>
#include <opencv2/opencv.hpp>

namespace pose
{
class NumberedFileReader
{
public:
    NumberedFileReader(std::string format);
    NumberedFileReader(std::string format, bool loop);
    NumberedFileReader(std::string format, int startFrame);
    NumberedFileReader(std::string format, int startFrame, int endFrame);
    NumberedFileReader(std::string format, int startFrame, int endFrame, bool loop);

    void setLoop(bool loop);
    void setStartFrame(int frame);
    void setEndFrame(int frame);

    void reset();
    bool read(cv::Mat&);

private:
    bool m_loop;
    int m_startFrame;
    int m_endFrame;
    std::string m_format;
    int m_fileIndex;
    char m_buffer[512];
};
}

#endif // NUMBEREDFILEREADER_H
