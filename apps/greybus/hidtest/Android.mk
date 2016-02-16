LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_C_INCLUDES := \
	vendor/ara-fw-test/apps/lib/include \
	external/udev/include
LOCAL_SRC_FILES := \
	hid-task.c \
	hidtest.c
LOCAL_MODULE := hidtest
LOCAL_STATIC_LIBRARIES := libfwtest
LOCAL_SHARED_LIBRARIES := libudev
LOCAL_MODULE_TAGS := optional

include $(BUILD_EXECUTABLE)

