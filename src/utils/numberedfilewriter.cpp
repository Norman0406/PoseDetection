#include "numberedfilewriter.h"
#include "utils.h"

namespace pose
{
NumberedFileWriter::NumberedFileWriter(std::string format)
    : m_format(format)
{
    m_terminateThread = false;
    m_thread = new boost::thread(&NumberedFileWriter::writeLoop, this);

    reset();
}

NumberedFileWriter::~NumberedFileWriter()
{
    // signal the thread to exit and free memory
    m_mutex.lock();
    m_terminateThread = true;
    m_condition.notify_one();
    m_mutex.unlock();
    m_thread->join();
    delete m_thread;
}

void NumberedFileWriter::reset()
{
    m_fileIndex = 0;
}

void NumberedFileWriter::write(const cv::Mat& image)
{
    // push a clone of the image onto the queue
    boost::mutex::scoped_lock(m_mutex);
    m_queue.push(image.clone());
    m_condition.notify_one();
}

void NumberedFileWriter::writeLoop()
{
    while (!m_terminateThread) {
        // wait until there is data in the queue
        boost::mutex::scoped_lock lock(m_mutex);
        while (m_queue.empty() && !m_terminateThread)
            m_condition.wait(lock);

        // if termiation flag is set, exit the loop
        if (m_terminateThread)
            continue;

        // get next image off the queue
        cv::Mat image = m_queue.front();
        m_queue.pop();
        lock.unlock();

        // actually write the image and increase the image counter
        sprintf_s(m_buffer, m_format.c_str(), m_fileIndex);
        Utils::saveCvMat(m_buffer, image);
        m_fileIndex++;
    }
}

}
