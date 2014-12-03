#include "utils.h"
#include "boundingbox3d.h"
#include <iostream>

namespace pose
{
void Utils::getColoredLabelMap(const cv::Mat& labelMap, cv::Mat& coloredLabelMap)
{
    if (coloredLabelMap.empty() || coloredLabelMap.rows != labelMap.rows ||
            coloredLabelMap.cols != labelMap.cols || coloredLabelMap.type() != CV_8UC3) {
        coloredLabelMap = cv::Mat(labelMap.rows, labelMap.cols, CV_8UC3);
    }
    coloredLabelMap.setTo(0);

    for (int i = 0; i < labelMap.cols; i++) {
        for (int j = 0; j < labelMap.rows; j++) {
            const unsigned int& label = labelMap.at<unsigned int>(cv::Point(i, j));

            if (label > 0) {
                srand(label);
                cv::Vec3b col((uchar)((rand() / (float)RAND_MAX) * 255),
                              (uchar)((rand() / (float)RAND_MAX) * 255),
                              (uchar)((rand() / (float)RAND_MAX) * 255));
                coloredLabelMap.at<cv::Vec3b>(cv::Point(i, j)) = col;
            }
        }
    }
}

cv::Mat Utils::getColoredLabelMap(const cv::Mat& labelMap)
{
    cv::Mat coloredLabelMap(labelMap.rows, labelMap.cols, CV_8UC3);
    coloredLabelMap.setTo(0);

    getColoredLabelMap(labelMap, coloredLabelMap);

    return coloredLabelMap;
}

bool Utils::loadCvMat(const char* filename, cv::Mat& image)
{
    if (!filename || strlen(filename) == 0)
        return false;

    FILE* file = NULL;

#ifdef _WIN32
    errno_t err = fopen_s(&file, filename, "rb");
    if(!file || err) {
        std::cerr << "could not open file: " << filename << std::endl;
        return false;
    }
#elif __APPLE__ & __MACH__
    file = fopen(filename, "rb");
    if(!file || ferror(file)) {
        cerr << "could not open file: " << filename << endl;
        return false;
    }
#endif

    // read header
    int flags = 0;
    int headerSize = 0;
    cv::Size imgSize(0, 0);
    int dims = 0;
    cv::Size origSize(0, 0);
    cv::Point startPoint(0, 0);
    int size = 0;

    char buffer[4];
    fread(&buffer, sizeof(char), 3, file);
    buffer[3] = '\0';

    if (strcmp(buffer, "CVM") != 0) {
        std::cerr << "file does not have cvm format: " << filename << std::endl;
        fclose(file);
        return 0;
    }

    // read header
    fread(&headerSize, sizeof(int), 1, file);
    fread(&flags, sizeof(int), 1, file);
    fread(&imgSize.width, sizeof(int), 1, file);
    fread(&imgSize.height, sizeof(int), 1, file);
    fread(&dims, sizeof(int), 1, file);
    fread(&origSize.width, sizeof(int), 1, file);
    fread(&origSize.height, sizeof(int), 1, file);
    fread(&startPoint.x, sizeof(int), 1, file);
    fread(&startPoint.y, sizeof(int), 1, file);
    fread(&size, sizeof(int), 1, file);

    image = cv::Mat(origSize.height, origSize.width, CV_MAT_TYPE(flags));

    // set file pointer
    fseek(file, headerSize, SEEK_SET);

    // read actual data
    fread(image.data, sizeof(unsigned char), size, file);

    // NOTE: Can be speeded up: don't create new image with the specified roi
    // like here, but only set image.dataStart and image.dataEnd accordingly
    // to point to the image region specified by the roi. [9/15/2011 Norman]
    image = image(cv::Rect(startPoint.x, startPoint.y, imgSize.width, imgSize.height));

    fclose(file);

    return true;
}

bool Utils::saveCvMat(const char* filename, const cv::Mat& image)
{
    if (!filename || strlen(filename) == 0)
        return false;

    FILE* file = NULL;

#ifdef _WIN32
    errno_t err = fopen_s(&file, filename, "wb");
    if(!file || err) {
        std::cerr << "could not create file: " << filename << std::endl;
        return false;
    }
#elif __APPLE__ & __MACH__
    file = fopen(filename, "wb");
    if(!file || ferror(file)) {
        cerr << "could not create file: " << filename << endl;
        return false;
    }
#endif

    // process roi information if available
    cv::Size origSize = cv::Size(0, 0);
    cv::Point startPoint = cv::Point(0, 0);

    image.locateROI(origSize, startPoint);
    int size = image.elemSize() * origSize.width * origSize.height;

    // header size
    const int headerSize = 3 * sizeof(char) +
        10 * sizeof(int);

    // write identification string
    fwrite("CVM", sizeof(char), 3, file);

    // write header
    fwrite(&headerSize, sizeof(int), 1, file);
    fwrite(&image.flags, sizeof(int), 1, file);
    fwrite(&image.cols, sizeof(int), 1, file);
    fwrite(&image.rows, sizeof(int), 1, file);
    fwrite(&image.dims, sizeof(int), 1, file);
    fwrite(&origSize.width, sizeof(int), 1, file);
    fwrite(&origSize.height, sizeof(int), 1, file);
    fwrite(&startPoint.x, sizeof(int), 1, file);
    fwrite(&startPoint.y, sizeof(int), 1, file);
    fwrite(&size, sizeof(int), 1, file);

    // write data
    fwrite(image.data, sizeof(uchar), size, file);

    fclose(file);

    return true;
}

float distancePrio1(const BoundingBox3D& box1, const BoundingBox3D& box2)
{
    float distLeft = box1.getAnchorDistance(BoundingBox3D::AT_LEFT, box2);
    float distTop = box1.getAnchorDistance(BoundingBox3D::AT_TOP, box2);
    float distRight = box1.getAnchorDistance(BoundingBox3D::AT_RIGHT, box2);
    float distBottom = box1.getAnchorDistance(BoundingBox3D::AT_BOTTOM, box2);

    return sqrtf(distLeft * distLeft + distTop * distTop + distRight * distRight + distBottom * distBottom);
    //return (distLeft + distTop + distRight + distBottom) / 4.0f;
    //return distLeft + distTop + distRight + distBottom;
}

float distancePrio2(const BoundingBox3D& box1, const BoundingBox3D& box2)
{
    float distLeft = box1.getAnchorDistance(BoundingBox3D::AT_LEFT, box2);
    float distTop = box1.getAnchorDistance(BoundingBox3D::AT_TOP, box2);
    float distRight = box1.getAnchorDistance(BoundingBox3D::AT_RIGHT, box2);
    float distBottom = box1.getAnchorDistance(BoundingBox3D::AT_BOTTOM, box2);

    float d1 = sqrtf(distLeft * distLeft + distTop * distTop + distRight * distRight);
    float d2 = sqrtf(distLeft * distLeft + distRight * distRight + distBottom * distBottom);
    float d3 = sqrtf(distTop * distTop + distRight * distRight + distBottom * distBottom);

    return std::min(d1, std::min(d2, d3));
}

float distancePrio3(const BoundingBox3D& box1, const BoundingBox3D& box2)
{
    float distLeft = box1.getAnchorDistance(BoundingBox3D::AT_LEFT, box2);
    float distTop = box1.getAnchorDistance(BoundingBox3D::AT_TOP, box2);
    float distRight = box1.getAnchorDistance(BoundingBox3D::AT_RIGHT, box2);
    float distBottom = box1.getAnchorDistance(BoundingBox3D::AT_BOTTOM, box2);

    float d1 = sqrtf(distLeft * distLeft + distTop * distTop);
    float d2 = sqrtf(distLeft * distLeft + distRight * distRight);
    float d3 = sqrtf(distLeft * distLeft + distBottom * distBottom);
    float d4 = sqrtf(distTop * distTop + distRight * distRight);
    float d5 = sqrtf(distTop * distTop + distBottom * distBottom);
    float d6 = sqrtf(distRight * distRight + distBottom * distBottom);

    return std::min(d1, std::min(d2, std::min(d3, std::min(d4, std::min(d5, d6)))));
}

float distancePrio4(const BoundingBox3D& box1, const BoundingBox3D& box2)
{
    float distLeft = box1.getAnchorDistance(BoundingBox3D::AT_LEFT, box2);
    float distTop = box1.getAnchorDistance(BoundingBox3D::AT_TOP, box2);
    float distRight = box1.getAnchorDistance(BoundingBox3D::AT_RIGHT, box2);
    float distBottom = box1.getAnchorDistance(BoundingBox3D::AT_BOTTOM, box2);

    return std::min(distLeft, std::min(distTop, std::min(distRight, distBottom)));
}

float Utils::distance(const BoundingBox3D& box1, const BoundingBox3D& box2, float searchRadius)
{
    /* priorisierung:
     * 1.)  AT_LEFT + AT_TOP + AT_RIGHT + AT_BOTTOM
     *
     * 2.)  AT_LEFT + AT_TOP + AT_RIGHT
     *      AT_LEFT + AT_RIGHT + AT_BOTTOM
     *      AT_TOP + AT_RIGHT + AT_BOTTOM
     *
     * 3.)  AT_LEFT + AT_TOP
     *      AT_LEFT + AT_RIGHT
     *      AT_LEFT + AT_BOTTOM
     *      AT_TOP + AT_RIGHT
     *      AT_TOP + AT_BOTTOM
     *      AT_RIGHT + AT_BOTTOM
     *
     * 4.)  AT_LEFT
     *      AT_TOP
     *      AT_RIGHT
     *      AT_BOTTOM
     */

    float dist1 = distancePrio1(box1, box2);
    /*if (dist1 < searchRadius)
        return dist1;*/

    float dist2 = distancePrio2(box1, box2);
    /*if (dist2 < searchRadius)
        return dist2;*/

    float dist3 = distancePrio3(box1, box2);
    /*if (dist3 < searchRadius)
        return dist3;*/

    //float dist4 = distancePrio4(box1, box2);
    return std::min(dist1, std::min(dist2, dist3));
}
}
