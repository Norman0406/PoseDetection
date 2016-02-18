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
    void write(const std::vector<cv::Mat>& mats);
    void write(const cv::Mat& mat);

private:
    void writeLoop();
    void writeMat(const cv::Mat& mat);

    std::string m_filename;
    int m_index;

    FILE* m_stream;

    std::queue<std::vector<cv::Mat>> m_queue;
    boost::thread* m_thread;
    boost::mutex m_mutex;
    boost::condition_variable m_condition;
    bool m_terminateThread;
};
}

#endif // STREAMWRITER_H
