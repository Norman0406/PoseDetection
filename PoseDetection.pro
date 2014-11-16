#-------------------------------------------------
#
# Project created by QtCreator 2014-09-25T22:06:29
#
#-------------------------------------------------

TARGET = PoseDetection
TEMPLATE = app

SOURCES += src/main.cpp \
    src/input/depthcamera.cpp \
    src/input/depthcamerakinectsdk.cpp \
    src/input/depthcamerakinectsdk2.cpp \
    src/segmentation/connectedcomponentlabeling.cpp \
    src/segmentation/tracking.cpp \
    src/segmentation/staticmap.cpp \
    src/utils/utils.cpp \
    src/utils/boundingbox2d.cpp \
    src/utils/boundingbox3d.cpp

HEADERS += src/input/depthcamera.h \
    src/input/depthcamerakinectsdk.h \
    src/input/depthcamerakinectsdk2.h \
    src/segmentation/connectedcomponentlabeling.h \
    src/segmentation/tracking.h \
    src/segmentation/staticmap.h \
    src/utils/utils.h \
    src/utils/boundingbox2d.h \
    src/utils/boundingbox3d.h

win32 {
    DEFINES += _CRT_SECURE_NO_WARNINGS

    INCLUDEPATH += $$(_PRO_FILE_PWD_)src \
        $$(OPENCV_DIR_2_4_9)/build/include \
        $$(PCL_DIR_1_7_2)/include \
        $$(EIGEN_DIR_3) \
        $$(BOOST_DIR_1_56) \
        $$(FLANN_DIR)/include \
        $$"C:\Program Files\Microsoft SDKs\Kinect\v1.8\inc" \
        $$"C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\inc"

    LIBS += "C:\Program Files\Microsoft SDKs\Kinect\v1.8\lib\x86\Kinect10.lib" \
        "C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\lib\x86\Kinect20.lib"

    CONFIG(debug, debug|release) {
        BUILDDIR = debug
        QMAKE_CXXFLAGS_DEBUG += /openmp

        LIBS += -L$$(BOOST_DIR_1_56)\stage\lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_core249d.lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_imgproc249d.lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_highgui249d.lib \
            $$(PCL_DIR_1_7_2)\lib\Debug\pcl_common_debug.lib \
            $$(PCL_DIR_1_7_2)\lib\Debug\pcl_features_debug.lib \
            $$(PCL_DIR_1_7_2)\lib\Debug\pcl_search_debug.lib \
            $$(PCL_DIR_1_7_2)\lib\Debug\pcl_segmentation_debug.lib

        OTHER_FILES += $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_core249d.dll \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_imgproc249d.dll \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_highgui249d.dll \
            $$(PCL_DIR_1_7_2)\bin\Debug\pcl_common_debug.dll \
            $$(PCL_DIR_1_7_2)\bin\Debug\pcl_features_debug.dll \
            $$(PCL_DIR_1_7_2)\bin\Debug\pcl_filters_debug.dll \
            $$(PCL_DIR_1_7_2)\bin\Debug\pcl_kdtree_debug.dll \
            $$(PCL_DIR_1_7_2)\bin\Debug\pcl_octree_debug.dll \
            $$(PCL_DIR_1_7_2)\bin\Debug\pcl_sample_consensus_debug.dll \
            $$(PCL_DIR_1_7_2)\bin\Debug\pcl_search_debug.dll \
            $$(PCL_DIR_1_7_2)\bin\Debug\pcl_segmentation_debug.dll
    }
    CONFIG(release, debug|release) {
        BUILDDIR = release
        QMAKE_CXXFLAGS_RELEASE += /openmp

        LIBS += -L$$(BOOST_DIR_1_56)\stage\lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_core249.lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_imgproc249.lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_highgui249.lib \
            $$(PCL_DIR_1_7_2)\lib\Release\pcl_common_release.lib \
            $$(PCL_DIR_1_7_2)\lib\Release\pcl_features_release.lib \
            $$(PCL_DIR_1_7_2)\lib\Release\pcl_search_release.lib \
            $$(PCL_DIR_1_7_2)\lib\Release\pcl_segmentation_release.lib

        OTHER_FILES += $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_core249.dll \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_imgproc249.dll \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_highgui249.dll \
            $$(PCL_DIR_1_7_2)\bin\Release\pcl_common_release.dll \
            $$(PCL_DIR_1_7_2)\bin\Release\pcl_features_release.dll \
            $$(PCL_DIR_1_7_2)\bin\Release\pcl_filters_release.dll \
            $$(PCL_DIR_1_7_2)\bin\Release\pcl_kdtree_release.dll \
            $$(PCL_DIR_1_7_2)\bin\Release\pcl_octree_release.dll \
            $$(PCL_DIR_1_7_2)\bin\Release\pcl_sample_consensus_release.dll \
            $$(PCL_DIR_1_7_2)\bin\Release\pcl_search_release.dll \
            $$(PCL_DIR_1_7_2)\bin\Release\pcl_segmentation_release.dll
    }

    # copy binary files to output directory
    for (file, OTHER_FILES) {
        copyFiles.commands += $(COPY) $$file \"$$OUT_PWD\"\\$$BUILDDIR $$escape_expand(\n\t)
    }
    QMAKE_EXTRA_TARGETS += copyFiles
    POST_TARGETDEPS += copyFiles
}
