LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE    := aimlock_head
LOCAL_SRC_FILES := aimlock_head.cpp
LOCAL_LDLIBS    := -llog
LOCAL_CFLAGS    := -O2 -std=c++17 -fPIC -ffast-math
include $(BUILD_SHARED_LIBRARY)
