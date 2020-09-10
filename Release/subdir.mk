################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../EcatDataSocket.cpp \
../RTClient.cpp 

OBJS += \
./EcatDataSocket.o \
./RTClient.o 

CPP_DEPS += \
./EcatDataSocket.d \
./RTClient.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	i686-unknown-linux-gnu-g++ -std=c++0x -D__XENO__ -D_USE_LIB_ -I"C:\Program Files (x86)\neuromeka\NRMKPlatformPC2\bin\i686\xenomai\include" -I"C:\Program Files (x86)\neuromeka\NRMKPlatformPC2\bin\i686\ethercat\include" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\helper\include" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\helper\include\hw" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\core\3rdparty\Poco\include" -I"c:\cygwin\usr\include\eigen3" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


