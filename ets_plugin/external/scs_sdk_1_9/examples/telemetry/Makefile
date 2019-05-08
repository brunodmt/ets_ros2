ROS_DISTRO=crystal
ROS_PATH=/opt/ros/$(ROS_DISTRO)

ROS_HEADERS=\
	$(ROS_PATH)/include/*.hpp \
	$(ROS_PATH)/include/rclcpp/*.hpp

ROS_INCLUDES=\
	-I$(ROS_PATH)/include

SDK_HEADERS=\
	../../include/*.h \
	../../include/common/*.h \
	../../include/amtrucks/*.h \
	../../include/eurotrucks2/*.h

SDK_INCLUDES=\
	-I../../include \
	-I../../include/common/ \
	-I../../include/amtrucks/ \
	-I../../include/eurotrucks2

UNAME:= $(shell uname -s)

ifeq ($(UNAME),Darwin)
LIB_NAME_OPTION=-install_name
else
LIB_NAME_OPTION=-soname
endif

telemetry.so:  *.cpp $(SDK_HEADERS)
	gcc -v -o $@ -fPIC -Wall --shared -Wl,-z,defs,-rpath=$(ROS_PATH)/lib,$(LIB_NAME_OPTION),$@ $(SDK_INCLUDES) $(ROS_INCLUDES) *.cpp

.PHONY: clean
clean:
	@rm -f -- *.so
