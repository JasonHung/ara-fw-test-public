LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

fwtest_files := \
	dumpargs.c \
	fwtools.c \
	log.c

LOCAL_SRC_FILES := $(fwtest_files)
LOCAL_C_INCLUDES := vendor/ara-fw-test/apps/lib/include
LOCAL_MODULE := libfwtest
LOCAL_MODULE_TAGS := optional

include $(BUILD_STATIC_LIBRARY)