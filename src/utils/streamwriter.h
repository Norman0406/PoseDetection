#ifndef STREAMWRITER_H
#define STREAMWRITER_H

#include <string>
#include <queue>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace pose
{
class StreamWriter
{
public:
    StreamWriter(std::string filename);
    ~StreamWriter();

    void reset();
    void write(const cv::Mat&);

private:
    void writeLoop();

    std::string m_filename;
    int m_index;

    FILE* m_stream;

    std::queue<cv::Mat> m_queue;
    boost::thread* m_thread;
    boost::mutex m_mutex;
    boost::condition_variable m_condition;
    bool m_terminateThread;
};
}

#endif // STREAMWRITER_H
