
BUILD_DIR := build

.DEFAULT_GOAL := ${BUILD_DIR}/BalancingController.elf

STM32_KIT=$(wildcard stm_lib/src/*.c) $(wildcard syscalls/*.c) $(wildcard cmsis_boot/*.c) $(wildcard cmsis_boot/*/*.c)
PROTO_LIB=$(wildcard ../nanopb-0.3.9.2-windows-x86/*.c)
PROTO_CODE=drv/comms/protocol.pb.c

HDRS := $(wildcard *.h) $(wildcard *.hpp) $(wildcard */*.h) $(wildcard */*.hpp) $(wildcard */*/*.h) $(wildcard */*/*.hpp) $(wildcard */*/*/*.h) $(wildcard */*/*/*.hpp)
SRCS := $(wildcard *.cpp) $(wildcard io/*.cpp) $(wildcard imu/*.cpp) $(wildcard guards/*.cpp) $(wildcard drv/vesc/*.cpp) $(wildcard drv/settings/*.cpp) $(wildcard drv/mpu6050/*.cpp) $(wildcard drv/led/*.cpp) ${STM32_KIT} ${PROTO_CODE} ${PROTO_LIB}
INC:=..\nanopb-0.3.9.2-windows-x86 drv cmsis_boot drv\vesc drv\comms stm_lib stm_lib\inc cmsis .
INC_PARAMS=$(INC:%=-I%)

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-gcc

ARCH = -mcpu=cortex-m3 -mthumb
CFLAGS = ${ARCH} -Wall -ffunction-sections -g -O2 -flto -fno-builtin -c -DSTM32F103CB -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -D__ASSEMBLY__ -DSUPPORT_CPLUSPLUS # -fstack-usage
CPPFLAGS = $(CFLAGS) -std=gnu++11

OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)

# Build step for C source
${BUILD_DIR}/%.c.o : %.c ${HDRS}
	mkdir -p $(dir $@)
	${CC} $(CFLAGS) ${INC_PARAMS} -c $< -o $@

# Build step for C++ source
$(BUILD_DIR)/%.cpp.o: %.cpp ${HDRS}
	mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(INC_PARAMS) -c $< -o $@

# The final build step.
${BUILD_DIR}/BalancingController.elf: $(OBJS) link.ld
	$(CC) ${ARCH} -g -flto -Wl,-Map=${BUILD_DIR}/BalancingController.map -O2 -Wl,--gc-sections -Wl,--entry=main -Wl,-T./link.ld -g -o $@ $(OBJS) -lm -lgcc -lc -lstdc++


# TODO1: Port generate bat.
# TODO2: Add program step
# TODO3: Figure out CooCox references.


.PHONY: clean
clean:
	rm -r $(BUILD_DIR)