#include "streamwriter.h"
#include "utils.h"

namespace pose
{
StreamWriter::StreamWriter(std::string filename)
    : m_filename(filename),
      m_stream(0)
{
    m_terminateThread = false;
    m_thread = new boost::thread(&StreamWriter::writeLoop, this);

    reset();
}

StreamWriter::~StreamWriter()
{
    // signal the thread to exit and free memory
    m_mutex.lock();
    m_terminateThread = true;
    m_condition.notify_one();
    m_mutex.unlock();
    m_thread->join();
    delete m_thread;

    // write frame count to beginning
    fseek(m_stream, 0, SEEK_SET);
    fwrite(&m_index, sizeof(int), 1, m_stream);

    fclose(m_stream);
    m_stream = 0;
}

void StreamWriter::reset()
{
    boost::mutex::scoped_lock(m_mutex);
    if (m_stream)
        fclose(m_stream);
    m_stream = fopen(m_filename.c_str(), "wb");
    int frameCount = 0;
    fwrite(&frameCount, sizeof(int), 1, m_stream);  // write initial (empty) frame count
    m_index = 0;
    while (!m_queue.empty())
        m_queue.pop();
}

void StreamWriter::write(const std::vector<cv::Mat>& mats)
{
    // push a clone of the images onto the queue
    boost::mutex::scoped_lock(m_mutex);

    std::vector<cv::Mat> clonedMats(mats.size());
    for (size_t i = 0; i < mats.size(); i++)
        clonedMats[i] = mats[i].clone();

    m_queue.push(clonedMats);

    m_condition.notify_one();
}

void StreamWriter::write(const cv::Mat& mat)
{
    // push a clone of the image onto the queue
    boost::mutex::scoped_lock(m_mutex);

    std::vector<cv::Mat> clonedMats(1);
    clonedMats[0] = mat.clone();

    m_queue.push(clonedMats);

    m_condition.notify_one();
}

void StreamWriter::writeLoop()
{
    while (!m_terminateThread) {
        // wait until there is data in the queue
        boost::mutex::scoped_lock lock(m_mutex);
        while (m_queue.empty() && !m_terminateThread)
            m_condition.wait(lock);

        // if termiation flag is set, exit the loop
        if (m_queue.empty() && m_terminateThread)
            continue;

        // get next image off the queue

        std::vector<cv::Mat> mats = m_queue.front();
        m_queue.pop();
        lock.unlock();

        int matCount = mats.size();

        fwrite(&m_index, sizeof(int), 1, m_stream);
        fwrite(&matCount, sizeof(int), 1, m_stream);

        for (size_t i = 0; i < mats.size(); i++)
            writeMat(mats[i]);

        m_index++;
    }
}

void StreamWriter::writeMat(const cv::Mat& mat)
{
    int size = mat.elemSize() * mat.cols * mat.rows;

    fwrite(&mat.cols, sizeof(int), 1, m_stream);
    fwrite(&mat.rows, sizeof(int), 1, m_stream);
    fwrite(&mat.flags, sizeof(int), 1, m_stream);
    fwrite(&size, sizeof(int), 1, m_stream);
    fwrite(mat.data, sizeof(uchar), size, m_stream);
}

}
