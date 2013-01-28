LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OPENCV_CAMERA_MODULES:=off
include /home/jlue/OpenCV-2.3.1/share/OpenCV/OpenCV.mk

LOCAL_MODULE    := NativeLib #nativeLib #NativeLib
LOCAL_SRC_FILES := NativeLib.cpp MarkerFinder.cpp mCV.cpp
LOCAL_LDLIBS +=  -llog -ldl

include $(BUILD_SHARED_LIBRARY)
