LBITS := $(shell getconf LONG_BIT)
MARCH ?= $(LBITS)
PREFIX ?= /usr/local
INSTALL_LIB_DIR ?= $(PREFIX)/lib
CFLAGS = -Wall -Wstrict-aliasing=0 -O0 -g -I ../hashlink/src -D LIBHL_EXPORTS -I ./libs/JoltPhysics
CPPFLAGS = -std=c++17
# Linux
CFLAGS += -m$(MARCH) -fPIC -pthread -fno-omit-frame-pointer

ifdef OSX_SDK
ISYSROOT = $(shell xcrun --sdk macosx$(OSX_SDK) --show-sdk-path)
CFLAGS += -isysroot $(ISYSROOT)
endif

JOLT_LIB = libs/JoltPhysics/Build/Linux_Debug/libJolt.a

jolt.hdll: ${JOLT_LIB} jolt.o JobSystemImpl.o
	${CXX} ${CPPFLAGS} ${CFLAGS} -shared -o jolt.hdll jolt.o ${JOLT_LIB} -lhl

jolt-test: ${JOLT_LIB} jolt.o
	${CXX} ${CPPFLAGS} ${CFLAGS} -o jolt-test jolt.o ${JOLT_LIB} -lhl

%.o: %.c
	${CC} ${CFLAGS} -o $@ -c $<

%.o: %.cpp
	${CXX} ${CPPFLAGS} ${CFLAGS} -o $@ -c $<

build: jolt.hdll

install build:
	rm -f $(INSTALL_LIB_DIR)/jolt.hdll
	cp jolt.hdll $(INSTALL_LIB_DIR)

clean: 
	rm -f jolt.o
	rm -f jolt.hdll