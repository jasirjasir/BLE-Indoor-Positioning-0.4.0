LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OPENCV_INSTALL_MODULES:=on
OPENCV_CAMERA_MODULES:=on
OPENCV_LIB_TYPE := STATIC

include $(LOCAL_PATH)/OpenCV.mk

LOCAL_MODULE := localize-lib,trilateration-lib,pathplan-lib
LOCAL_SRC_FILES := E:/Android_Studio_Projects/BLE-Indoor-Positioning-0.4.0/app/src/main/cpp/localise.cpp,E:/Android_Studio_Projects/BLE-Indoor-Positioning-0.4.0/app/src/main/cpp/trilateration.cpp,E:/Android_Studio_Projects/BLE-Indoor-Positioning-0.4.0/app/src/main/cpp/beacon.cpp,E:/Android_Studio_Projects/BLE-Indoor-Positioning-0.4.0/app/src/main/cpp/test_trilateration.cpp,E:/Android_Studio_Projects/Camera2_ZBar-continous_scanning_camera2/app/src/main/cpp/pathPlanning.cpp
LOCAL_C_INCLUDES := E:/Android_Studio_Projects/Util/OpenCV-android-sdk/sdk/native/jni/include
#LOCAL_LDLIBS += -lm -llog
LOCAL_LDLIBS := -L$(SYSROOT)/usr/lib -llog

include $(BUILD_SHARED_LIBRARY)
