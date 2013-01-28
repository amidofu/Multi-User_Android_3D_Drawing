LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OPENCV_CAMERA_MODULES:=off
include /home/jlue/OpenCV-2.3.1/share/OpenCV/OpenCV.mk

#OSG_ANDROID_DIR	:= /home/jlue/OpenSceneGraph-3.0.1/osginstall
#LIBDIR 			:= $(OSG_ANDROID_DIR)/obj/local/armeabi

#ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
#	LOCAL_ARM_NEON 	:= true
#	LIBDIR 			:= $(OSG_ANDROID_DIR)/obj/local/armeabi-v7a
#endif

### Add all source file names to be included in lib separated by a whitespace

#LOCAL_C_INCLUDES += $(OSG_ANDROID_DIR)/include
#LOCAL_CFLAGS    :=  -fno-short-enums
#LOCAL_CPPFLAGS  := -DOSG_LIBRARY_STATIC 

LOCAL_MODULE    := NativeLib 
LOCAL_SRC_FILES := MarkerFinder.cpp NativeLib.cpp mCV.cpp functions.cpp CVAuxFunc.cpp 
LOCAL_LDLIBS +=  -llog -ldl #-losgdb_osg

include $(BUILD_SHARED_LIBRARY)
