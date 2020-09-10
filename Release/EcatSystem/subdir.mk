################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../EcatSystem/Ecat_Master.cpp 

C_SRCS += \
../EcatSystem/PDOConfig.c 

OBJS += \
./EcatSystem/Ecat_Master.o \
./EcatSystem/PDOConfig.o 

CPP_DEPS += \
./EcatSystem/Ecat_Master.d 

C_DEPS += \
./EcatSystem/PDOConfig.d 


# Each subdirectory must supply rules for building sources it contributes
EcatSystem/%.o: ../EcatSystem/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C++ Compiler'
	i686-unknown-linux-gnu-g++ -std=c++0x -D__XENO__ -D_USE_LIB_ -I"C:\Program Files (x86)\neuromeka\NRMKPlatformPC2\bin\i686\xenomai\include" -I"C:\Program Files (x86)\neuromeka\NRMKPlatformPC2\bin\i686\ethercat\include" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\helper\include" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\helper\include\hw" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\core\3rdparty\Poco\include" -I"c:\cygwin\usr\include\eigen3" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

EcatSystem/%.o: ../EcatSystem/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C Compiler'
	i686-unknown-linux-gnu-gcc -std=c11 -D__XENO__ -D_USE_LIB_ -I"C:\Program Files (x86)\neuromeka\NRMKPlatformPC2\bin\i686\xenomai\include" -I"C:\Program Files (x86)\neuromeka\NRMKPlatformPC2\bin\i686\ethercat\include" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\helper\include" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\helper\include\hw" -I"C:\Program Files (x86)\neuromeka\NRMKFoundation\core\3rdparty\Poco\include" -I"c:\cygwin\usr\include\eigen3" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


