LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_C_INCLUDES := vendor/ara-fw-test/apps/lib/include
LOCAL_SRC_FILES := \
	commsteps.c \
	gpiotest.c
LOCAL_MODULE := gpiotest
LOCAL_STATIC_LIBRARIES := libfwtest
LOCAL_MODULE_TAGS := optional

include $(BUILD_EXECUTABLE)

