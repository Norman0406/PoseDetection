TARGET = pose
TEMPLATE = lib
CONFIG += create_prl

SOURCES += src/pose.cc \
    src/algorithm.cpp \
    src/input/input.cpp \
    src/segmentation/connectedcomponentlabeling.cpp \
    src/segmentation/tracking.cpp \
    src/segmentation/staticmap.cpp \
    src/tracking/bone.cpp \
    src/tracking/joint.cpp \
    src/tracking/fitting.cpp \
    src/tracking/fittingmethod.cpp \
    src/tracking/fittingmethodpso.cpp \
    src/tracking/skeleton.cpp \
    src/tracking/skeletonupperbody.cpp \
    src/utils/utils.cpp \
    src/utils/boundingbox2d.cpp \
    src/utils/boundingbox3d.cpp \
    src/utils/connectedcomponent.cpp \
    src/utils/exception.cpp \
    src/utils/numberedfilewriter.cpp \
    src/utils/numberedfilereader.cpp \
    src/utils/module.cpp \
    src/utils/timer.cpp \
    src/utils/streamreader.cpp \
    src/utils/streamwriter.cpp

HEADERS += include/pose.h \
    src/internal.h \
    src/algorithm.h \
    src/input/input.h \
    src/segmentation/connectedcomponentlabeling.h \
    src/segmentation/tracking.h \
    src/segmentation/staticmap.h \
    src/tracking/bone.h \
    src/tracking/joint.h \
    src/tracking/fitting.h \
    src/tracking/fittingmethod.h \
    src/tracking/fittingmethodpso.h \
    src/tracking/skeleton.h \
    src/tracking/skeletonupperbody.h \
    src/utils/utils.h \
    src/utils/boundingbox2d.h \
    src/utils/boundingbox3d.h \
    src/utils/connectedcomponent.h \
    src/utils/exception.h \
    src/utils/numberedfilewriter.h \
    src/utils/numberedfilereader.h \
    src/utils/module.h \
    src/utils/timer.h \
    src/utils/streamreader.h \
    src/utils/streamwriter.h

INCLUDEPATH += $${_PRO_FILE_PWD_}/src \
    $${_PRO_FILE_PWD_}/include

win32 {
    DEFINES += _CRT_SECURE_NO_WARNINGS \
        DEBUG_IMAGES

    INCLUDEPATH += $$(OPENCV_DIR_2_4_9)/build/include \
        $$(EIGEN_DIR_3) \
        $$(BOOST_DIR_1_58) \
        $$(FLANN_DIR_1_8_4)/include

    CONFIG(debug, debug|release) {
        BUILDDIR = debug
        QMAKE_CXXFLAGS_DEBUG += /openmp

        LIBS += -L$$(BOOST_DIR_1_58)\stage\lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_core249d.lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_imgproc249d.lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_highgui249d.lib \
            $$(FLANN_DIR_1_8_4)\lib\Debug\flann_cpp_s.lib

        OTHER_FILES += $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_core249d.dll \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_imgproc249d.dll \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_highgui249d.dll
    }
    CONFIG(release, debug|release) {
        BUILDDIR = release
        QMAKE_CXXFLAGS_RELEASE += /openmp

        LIBS += -L$$(BOOST_DIR_1_58)\stage\lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_core249.lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_imgproc249.lib \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\lib\opencv_highgui249.lib \
            $$(FLANN_DIR_1_8_4)\lib\Release\flann_cpp_s.lib

        OTHER_FILES += $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_core249.dll \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_imgproc249.dll \
            $$(OPENCV_DIR_2_4_9)\build\x86\vc12\bin\opencv_highgui249.dll
    }

    # copy binary files to output directory
    for (file, OTHER_FILES) {
        copyFiles.commands += $(COPY) $$file \"$$OUT_PWD\"\\$$BUILDDIR $$escape_expand(\n\t)
    }
    QMAKE_EXTRA_TARGETS += copyFiles
    POST_TARGETDEPS += copyFiles
}
