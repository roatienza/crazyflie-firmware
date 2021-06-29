# enable app support
APP=1
APP_STACKSIZE=300

VPATH += src/
PROJ_OBJ += fly.o
PROJ_OBJ += policy.o

CRAZYFLIE_BASE=../../..
include $(CRAZYFLIE_BASE)/Makefile

#LDFLAGS += -lonnx
INCLUDES += -I$(CRAZYFLIE_BASE)/vendor/libonnx
