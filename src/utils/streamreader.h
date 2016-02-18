#ifndef STREAMREADER_H
#define STREAMREADER_H

#include <string>
#include <opencv2/opencv.hpp>

namespace pose
{
class StreamReader
{
public:
    StreamReader(std::string filename);
    StreamReader(std::string filename, bool loop);
    StreamReader(std::string filename, int startFrame);
    StreamReader(std::string filename, int startFrame, int endFrame);
    StreamReader(std::string filename, int startFrame, int endFrame, bool loop);

    void setLoop(bool loop);
    void setStartFrame(int frame);
    void setEndFrame(int frame);

    int getFrameIndex() const;

    void reset();
    bool read(cv::Mat&);

private:
    void seekToFrame(int frame);

    bool m_loop;
    int m_startFrame;
    int m_endFrame;
    std::string m_filename;
    int m_index;
    int m_frameCount;
    FILE* m_stream;
};
}

#endif // STREAMREADER_H
