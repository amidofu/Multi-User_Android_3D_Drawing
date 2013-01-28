LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OPENCV_CAMERA_MODULES:=off
include /home/jlue/OpenCV-2.3.1/share/OpenCV/OpenCV.mk

LOCAL_MODULE    := NativeLib
### Main Install dir
OSG_ANDROID_DIR	:= /home/jlue/OpenSceneGraph-3.0.1/osginstall
LIBDIR 			:= $(OSG_ANDROID_DIR)/obj/local/armeabi

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
	LOCAL_ARM_NEON 	:= true
	LIBDIR 			:= $(OSG_ANDROID_DIR)/obj/local/armeabi-v7a
endif

### Add all source file names to be included in lib separated by a whitespace

LOCAL_C_INCLUDES += $(OSG_ANDROID_DIR)/include
LOCAL_CFLAGS    :=  -fno-short-enums
LOCAL_CPPFLAGS  := -DOSG_LIBRARY_STATIC 

LOCAL_LDLIBS    := -llog -lGLESv1_CM -ldl -lz -lgnustl_static#-llog -lGLESv1_CM -ldl -lz
LOCAL_SRC_FILES := osgMain.cpp  CamInfo.cpp MyFirstPersonCamManipulator.cpp PolytopePickHandler.cpp AuxFunc.cpp scene.cpp transformedboundingbox.cpp findchildrenboundboxnodevisitor.cpp getworldcoordofnodevisitor.cpp \
UpdateOtherPlayerCamCallback.cpp clientMain.cpp NativeLib.cpp MarkerFinder.cpp mCV.cpp MarkerStorage.cpp KeyFrame.cpp MarkerlessFunc.cpp
LOCAL_LDFLAGS   := -L $(LIBDIR) \
-losgdb_dds \
-losgdb_openflight \
-losgdb_tga \
-losgdb_rgb \
-losgdb_osgterrain \
-losgdb_osg \
-losgdb_ive \
-losgdb_deprecated_osgviewer \
-losgdb_deprecated_osgvolume \
-losgdb_deprecated_osgtext \
-losgdb_deprecated_osgterrain \
-losgdb_deprecated_osgsim \
-losgdb_deprecated_osgshadow \
-losgdb_deprecated_osgparticle \
-losgdb_deprecated_osgfx \
-losgdb_deprecated_osganimation \
-losgdb_deprecated_osg \
-losgdb_serializers_osgvolume \
-losgdb_serializers_osgtext \
-losgdb_serializers_osgterrain \
-losgdb_serializers_osgsim \
-losgdb_serializers_osgshadow \
-losgdb_serializers_osgparticle \
-losgdb_serializers_osgmanipulator \
-losgdb_serializers_osgfx \
-losgdb_serializers_osganimation \
-losgdb_serializers_osg \
-losgViewer \
-losgVolume \
-losgTerrain \
-losgText \
-losgShadow \
-losgSim \
-losgParticle \
-losgManipulator \
-losgGA \
-losgFX \
-losgDB \
-losgAnimation \
-losgUtil \
-losg \
-lOpenThreads

include $(BUILD_SHARED_LIBRARY)