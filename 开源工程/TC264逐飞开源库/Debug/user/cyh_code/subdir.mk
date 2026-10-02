################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../user/cyh_code/Rot.c \
../user/cyh_code/cyh_anniu.c \
../user/cyh_code/cyh_camera.c \
../user/cyh_code/cyh_duoji.c \
../user/cyh_code/cyh_encoder.c \
../user/cyh_code/cyh_headfile.c \
../user/cyh_code/cyh_motor.c \
../user/cyh_code/cyh_yuansu.c 

COMPILED_SRCS += \
./user/cyh_code/Rot.src \
./user/cyh_code/cyh_anniu.src \
./user/cyh_code/cyh_camera.src \
./user/cyh_code/cyh_duoji.src \
./user/cyh_code/cyh_encoder.src \
./user/cyh_code/cyh_headfile.src \
./user/cyh_code/cyh_motor.src \
./user/cyh_code/cyh_yuansu.src 

C_DEPS += \
./user/cyh_code/Rot.d \
./user/cyh_code/cyh_anniu.d \
./user/cyh_code/cyh_camera.d \
./user/cyh_code/cyh_duoji.d \
./user/cyh_code/cyh_encoder.d \
./user/cyh_code/cyh_headfile.d \
./user/cyh_code/cyh_motor.d \
./user/cyh_code/cyh_yuansu.d 

OBJS += \
./user/cyh_code/Rot.o \
./user/cyh_code/cyh_anniu.o \
./user/cyh_code/cyh_camera.o \
./user/cyh_code/cyh_duoji.o \
./user/cyh_code/cyh_encoder.o \
./user/cyh_code/cyh_headfile.o \
./user/cyh_code/cyh_motor.o \
./user/cyh_code/cyh_yuansu.o 


# Each subdirectory must supply rules for building sources it contributes
user/cyh_code/%.src: ../user/cyh_code/%.c user/cyh_code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/varyapps/sxt/newPCBC8/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/user\/cyh_code\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

user/cyh_code/%.o: ./user/cyh_code/%.src user/cyh_code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-user-2f-cyh_code

clean-user-2f-cyh_code:
	-$(RM) ./user/cyh_code/Rot.d ./user/cyh_code/Rot.o ./user/cyh_code/Rot.src ./user/cyh_code/cyh_anniu.d ./user/cyh_code/cyh_anniu.o ./user/cyh_code/cyh_anniu.src ./user/cyh_code/cyh_camera.d ./user/cyh_code/cyh_camera.o ./user/cyh_code/cyh_camera.src ./user/cyh_code/cyh_duoji.d ./user/cyh_code/cyh_duoji.o ./user/cyh_code/cyh_duoji.src ./user/cyh_code/cyh_encoder.d ./user/cyh_code/cyh_encoder.o ./user/cyh_code/cyh_encoder.src ./user/cyh_code/cyh_headfile.d ./user/cyh_code/cyh_headfile.o ./user/cyh_code/cyh_headfile.src ./user/cyh_code/cyh_motor.d ./user/cyh_code/cyh_motor.o ./user/cyh_code/cyh_motor.src ./user/cyh_code/cyh_yuansu.d ./user/cyh_code/cyh_yuansu.o ./user/cyh_code/cyh_yuansu.src

.PHONY: clean-user-2f-cyh_code

