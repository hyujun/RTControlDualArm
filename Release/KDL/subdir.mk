################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../KDL/LieDynamics.cpp \
../KDL/LieOperator.cpp \
../KDL/PoEKinematics.cpp \
../KDL/PropertyDefinition.cpp \
../KDL/SerialManipulator.cpp 

OBJS += \
./KDL/LieDynamics.o \
./KDL/LieOperator.o \
./KDL/PoEKinematics.o \
./KDL/PropertyDefinition.o \
./KDL/SerialManipulator.o 

CPP_DEPS += \
./KDL/LieDynamics.d \
./KDL/LieOperator.d \
./KDL/PoEKinematics.d \
./KDL/PropertyDefinition.d \
./KDL/SerialManipulator.d 


# Each subdirectory must supply rules for building sources it contributes
KDL/%.o: ../KDL/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	i686-unknown-linux-gnu-g++ -std=c++0x -D__XENO__ -D_USE_LIB_ -I"C:\Program Files (x86)\neuromeka\NRMKPlatformPC2\bin\i686\xenomai\include" -I"C:\Program Files (x86)\neuromeka\NRMKPlatformPC2\bin\i686\ethercat\include" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\helper\include" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\helper\include\hw" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\core\3rdparty\Poco\include" -I"c:\cygwin\usr\include\eigen3" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


