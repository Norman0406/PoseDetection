#ifndef NUMBEREDFILEWRITER_H
#define NUMBEREDFILEWRITER_H

#include <string>
#include <queue>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace pose
{
class NumberedFileWriter
{
public:
    NumberedFileWriter(std::string format);
    ~NumberedFileWriter();

    void reset();
    void write(const cv::Mat&);

private:
    void writeLoop();

    std::string m_format;
    int m_fileIndex;
    char m_buffer[512];

    std::queue<cv::Mat> m_queue;
    boost::thread* m_thread;
    boost::mutex m_mutex;
    boost::condition_variable m_condition;
    bool m_terminateThread;
};
}

#endif // NUMBEREDFILEWRITER_H
