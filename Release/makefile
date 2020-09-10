################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include KDL/subdir.mk
-include EcatSystem/subdir.mk
-include Control/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: RTControlDualArm

# Tool invocations
RTControlDualArm: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cygwin C++ Linker'
	i686-unknown-linux-gnu-g++ -L"C:\Program Files (x86)\neuromeka\NRMKPlatformPC2\bin\i686\xenomai\lib" -L"C:\Program Files (x86)\neuromeka\NRMKPlatformPC2\bin\i686\ethercat\lib" -L"C:\Program Files (x86)\neuromeka\NRMKFoundation\lib" -L"C:\Program Files (x86)\neuromeka\NRMKFoundation\core\3rdparty\Poco\lib\i686" -o "RTControlDualArm" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(OBJS)$(C_UPPER_DEPS)$(CXX_DEPS)$(INO_DEPS)$(PDE_DEPS)$(CPP_DEPS)$(C_DEPS) RTControlDualArm
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
