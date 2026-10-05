iPhoneLOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE    := aimlock
LOCAL_SRC_FILES := aimlock_s7.cpp
LOCAL_LDLIBS    := -llog
LOCAL_CFLAGS    := -O2 -fvisibility=hidden
include $(BUILD_SHARED_LIBRARY)
