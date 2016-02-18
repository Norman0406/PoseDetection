#include "streamreader.h"
#include "exception.h"
#include "utils.h"

namespace pose
{
StreamReader::StreamReader(std::string filename)
    : m_filename(filename),
      m_stream(0)
{
    reset();
}

StreamReader::StreamReader(std::string filename, bool loop)
    : m_filename(filename),
      m_stream(0)
{
    reset();
    setLoop(loop);
}

StreamReader::StreamReader(std::string filename, int startFrame)
    : m_filename(filename),
      m_stream(0)
{
    reset();
    setStartFrame(startFrame);
}

StreamReader::StreamReader(std::string filename, int startFrame, int endFrame)
    : m_filename(filename),
      m_stream(0)
{
    reset();
    setStartFrame(startFrame);
    setEndFrame(endFrame);
}

StreamReader::StreamReader(std::string filename, int startFrame, int endFrame, bool loop)
    : m_filename(filename),
      m_stream(0)
{
    reset();
    setStartFrame(startFrame);
    setEndFrame(endFrame);
    setLoop(loop);
}

void StreamReader::setLoop(bool loop)
{
    m_loop = loop;
}

void StreamReader::setStartFrame(int frame)
{
    m_startFrame = frame;
    m_index = m_startFrame;
    seekToFrame(frame);
}

void StreamReader::setEndFrame(int frame)
{
    m_endFrame = frame;
}

int StreamReader::getFrameIndex() const
{
    return m_index;
}

void StreamReader::reset()
{
    if (m_stream)
        fclose(m_stream);

    m_stream = fopen(m_filename.c_str(), "rb");
    if (!m_stream)
        throw Exception("Could not open stream");

    fread(&m_frameCount, sizeof(int), 1, m_stream);

    m_loop = false;
    m_startFrame = 0;
    m_endFrame = -1;
    m_index = 0;
}

void StreamReader::seekToFrame(int frame)
{
    // skip frame count at the beginning
    fseek(m_stream, sizeof(int), SEEK_SET);

    if (frame >= m_frameCount)
        throw Exception("invalid frame number");

    int curFrame = 0;
    while (curFrame < frame - 1) {
        fread(&curFrame, sizeof(int), 1, m_stream);

        int matCount = 0;
        fread(&matCount, sizeof(int), 1, m_stream);

        // skip image
        for (int i = 0; i < matCount; i++) {
            fseek(m_stream, 3 * sizeof(int), SEEK_CUR);

            int size = 0;
            fread(&size, sizeof(int), 1, m_stream);

            fseek(m_stream, size, SEEK_CUR);
        }

    }
}

bool StreamReader::read(cv::Mat& image)
{
    std::vector<cv::Mat> mats;
    if (!read(mats))
        return false;

    if (mats.size() == 0)
        return false;

    image = mats[0];

    return true;
}

bool StreamReader::read(std::vector<cv::Mat>& mats)
{
    if (m_index == m_endFrame || m_index == m_frameCount - 1) {
        if (m_loop)
            seekToFrame(m_startFrame);
        else
            return false;
    }

    int matCount = 0;
    fread(&m_index, sizeof(int), 1, m_stream);
    fread(&matCount, sizeof(int), 1, m_stream);

    if (matCount == 0)
        int k = 0;  // why is there a 0 at frame 178?

    if (mats.size() != matCount)
        mats.resize(matCount);

    for (int i = 0; i < matCount; i++)
        readMat(mats[i]);

    return true;
}

void StreamReader::readMat(cv::Mat& mat)
{
    int width = 0, height = 0, flags = 0, size = 0;
    fread(&width, sizeof(int), 1, m_stream);
    fread(&height, sizeof(int), 1, m_stream);
    fread(&flags, sizeof(int), 1, m_stream);
    fread(&size, sizeof(int), 1, m_stream);

    int matSize = mat.elemSize() * mat.cols * mat.rows;
    if (mat.cols != width || mat.rows != height || mat.flags != flags || matSize != size)
        mat = cv::Mat(height, width, CV_MAT_TYPE(flags));

    fread(mat.data, sizeof(uchar), size, m_stream);
}
}
