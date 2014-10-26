#######################################################################
# Instuctions on including driver dependencies
#    Serial may be left out (the lines below may be commented out)
#    if the API was compiled without support for serial
#######################################################################
PROGRAM = torqeuControl
ifeq (${ARCH},windows)
	#######################################################################
	# Windows
	INCLUDES = -I"$(POCO_BASE)/Foundation/include" -I"../../API/"
	LIBDIR += -L"$(POCO_BASE)/lib/MinGW/ia32" -L"../../API/lib"
	CFLAGS = -mthreads -O4
	CFLAGS -= -g
	LDLIBS = -lBHand -lwinmm
	LDFLAGS = -s -Wl,--allow-multiple-definition
	CLEANCMD = del ${PROGRAM}.exe *.o
	DEFS =
	#######################################################################
	# Windows CAN Driver Dependency (must be included)
	PCAN_DIR = ../../ThirdParty/PCAN
	INCLUDES += -I"$(PCAN_DIR)/include"
	LDLIBS += -lPcan_usb
	#######################################################################
	# Last
	LDLIBS += -lPocoFoundation
else 
	#######################################################################
	# Leave these commented to use installed headers and BHand library
	#INCLUDES += -I../../API
	#LIBDIR += -L../../API/lib
	#######################################################################
	# Linux (Required)
	BHAND_INSTALL_DIR = /usr/local/Barrett/BHand
	#BHAND_DIR = /home/yoshiki/ROS/mybhand/src/serial_bhand
	INCLUDES += -I${BHAND_INSTALL_DIR}/thirdparty/include -I${BHAND_INSTALL_DIR}/API
	LIBDIR += -L${BHAND_INSTALL_DIR}/API/lib
	LDLIBS = -lBHand
	CLEANCMD = rm -f ${PROGRAM} *.o
	DEFS = -DLINUX 
	#######################################################################
	# Linux CAN Driver Dependency (must be included)
	INCLUDES += -I../../thirdparty/peak-linux-driver-6.20/driver
	INCLUDES += -I../../thirdparty/peak-linux-driver-6.20/lib
	LDLIBS += -lpcan
	#######################################################################
	# Last
	LDLIBS += -lPocoFoundationBarrett
endif 

default: ${PROGRAM}.o
	g++ ${CFLAGS} ${PROGRAM}.o ${INCLUDES} ${LDFLAGS} ${LIBDIR} ${LDLIBS} -o ${PROGRAM}


${PROGRAM}.o: ${PROGRAM}.cpp
	g++ ${CFLAGS} ${DEFS} ${INCLUDES} -c ${PROGRAM}.cpp


.PHONY: clean 

clean: 
	${CLEANCMD} 

