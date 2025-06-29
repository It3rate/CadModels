# ST Visual Debugger Generated MAKE File, based on stm8testprog.stp

ifeq ($(CFG), )
CFG=Debug
$(warning ***No configuration specified. Defaulting to $(CFG)***)
endif

ToolsetRoot=C:\COSMIC\FSE_Compilers\CXSTM8
ToolsetBin=C:\COSMIC\FSE_Compilers\CXSTM8
ToolsetInc=C:\COSMIC\FSE_Compilers\CXSTM8\Hstm8
ToolsetLib=C:\COSMIC\FSE_Compilers\CXSTM8\Lib
ToolsetIncOpts=-iC:\COSMIC\FSE_Compilers\CXSTM8\Hstm8 
ToolsetLibOpts=-lC:\COSMIC\FSE_Compilers\CXSTM8\Lib 
ObjectExt=o
OutputExt=elf
InputName=$(basename $(notdir $<))


# 
# Debug
# 
ifeq "$(CFG)" "Debug"


OutputPath=Debug
ProjectSFile=stm8testprog
TargetSName=$(ProjectSFile)
TargetFName=$(ProjectSFile).elf
IntermPath=$(dir $@)
CFLAGS_PRJ=$(ToolsetBin)\cxstm8  -icustom +mods0 +debug -pxp -no -pp -v -ilibs\inc -iinc -ec $(ToolsetIncOpts) -cl$(IntermPath:%\=%) -co$(IntermPath:%\=%) $<
ASMFLAGS_PRJ=$(ToolsetBin)\castm8  -xx -l $(ToolsetIncOpts) -o$(IntermPath)$(InputName).$(ObjectExt) $<

all : $(OutputPath) $(ProjectSFile).elf

$(OutputPath) : 
	if not exist $(OutputPath)/ mkdir $(OutputPath)

Debug\bme280.$(ObjectExt) : custom\bme280.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h custom\bme280.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_adc1.$(ObjectExt) : libs\src\stm8s_adc1.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_adc1.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_awu.$(ObjectExt) : libs\src\stm8s_awu.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_awu.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_beep.$(ObjectExt) : libs\src\stm8s_beep.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_beep.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_clk.$(ObjectExt) : libs\src\stm8s_clk.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_clk.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_exti.$(ObjectExt) : libs\src\stm8s_exti.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_exti.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_flash.$(ObjectExt) : libs\src\stm8s_flash.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_flash.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_gpio.$(ObjectExt) : libs\src\stm8s_gpio.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_gpio.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_i2c.$(ObjectExt) : libs\src\stm8s_i2c.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_i2c.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_itc.$(ObjectExt) : libs\src\stm8s_itc.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_itc.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_iwdg.$(ObjectExt) : libs\src\stm8s_iwdg.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_iwdg.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_rst.$(ObjectExt) : libs\src\stm8s_rst.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_rst.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_spi.$(ObjectExt) : libs\src\stm8s_spi.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_spi.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_tim1.$(ObjectExt) : libs\src\stm8s_tim1.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_tim1.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_tim2.$(ObjectExt) : libs\src\stm8s_tim2.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_tim2.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_uart1.$(ObjectExt) : libs\src\stm8s_uart1.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_uart1.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8s_wwdg.$(ObjectExt) : libs\src\stm8s_wwdg.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_wwdg.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\main.$(ObjectExt) : main.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h custom\bme280.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Debug\stm8_interrupt_vector.$(ObjectExt) : stm8_interrupt_vector.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

$(ProjectSFile).elf :  $(OutputPath)\bme280.o $(OutputPath)\stm8s_adc1.o $(OutputPath)\stm8s_awu.o $(OutputPath)\stm8s_beep.o $(OutputPath)\stm8s_clk.o $(OutputPath)\stm8s_exti.o $(OutputPath)\stm8s_flash.o $(OutputPath)\stm8s_gpio.o $(OutputPath)\stm8s_i2c.o $(OutputPath)\stm8s_itc.o $(OutputPath)\stm8s_iwdg.o $(OutputPath)\stm8s_rst.o $(OutputPath)\stm8s_spi.o $(OutputPath)\stm8s_tim1.o $(OutputPath)\stm8s_tim2.o $(OutputPath)\stm8s_uart1.o $(OutputPath)\stm8s_wwdg.o $(OutputPath)\main.o $(OutputPath)\stm8_interrupt_vector.o $(OutputPath)\stm8testprog.lkf
	$(ToolsetBin)\clnk  $(ToolsetLibOpts) -o $(OutputPath)\$(TargetSName).sm8 -m$(OutputPath)\$(TargetSName).map $(OutputPath)\$(TargetSName).lkf 
	$(ToolsetBin)\cvdwarf  $(OutputPath)\$(TargetSName).sm8

	$(ToolsetBin)\chex  -o $(OutputPath)\$(TargetSName).s19 $(OutputPath)\$(TargetSName).sm8
clean : 
	-@erase $(OutputPath)\bme280.o
	-@erase $(OutputPath)\stm8s_adc1.o
	-@erase $(OutputPath)\stm8s_awu.o
	-@erase $(OutputPath)\stm8s_beep.o
	-@erase $(OutputPath)\stm8s_clk.o
	-@erase $(OutputPath)\stm8s_exti.o
	-@erase $(OutputPath)\stm8s_flash.o
	-@erase $(OutputPath)\stm8s_gpio.o
	-@erase $(OutputPath)\stm8s_i2c.o
	-@erase $(OutputPath)\stm8s_itc.o
	-@erase $(OutputPath)\stm8s_iwdg.o
	-@erase $(OutputPath)\stm8s_rst.o
	-@erase $(OutputPath)\stm8s_spi.o
	-@erase $(OutputPath)\stm8s_tim1.o
	-@erase $(OutputPath)\stm8s_tim2.o
	-@erase $(OutputPath)\stm8s_uart1.o
	-@erase $(OutputPath)\stm8s_wwdg.o
	-@erase $(OutputPath)\main.o
	-@erase $(OutputPath)\stm8_interrupt_vector.o
	-@erase $(OutputPath)\stm8testprog.elf
	-@erase $(OutputPath)\stm8testprog.elf
	-@erase $(OutputPath)\stm8testprog.map
	-@erase $(OutputPath)\bme280.ls
	-@erase $(OutputPath)\stm8s_adc1.ls
	-@erase $(OutputPath)\stm8s_awu.ls
	-@erase $(OutputPath)\stm8s_beep.ls
	-@erase $(OutputPath)\stm8s_clk.ls
	-@erase $(OutputPath)\stm8s_exti.ls
	-@erase $(OutputPath)\stm8s_flash.ls
	-@erase $(OutputPath)\stm8s_gpio.ls
	-@erase $(OutputPath)\stm8s_i2c.ls
	-@erase $(OutputPath)\stm8s_itc.ls
	-@erase $(OutputPath)\stm8s_iwdg.ls
	-@erase $(OutputPath)\stm8s_rst.ls
	-@erase $(OutputPath)\stm8s_spi.ls
	-@erase $(OutputPath)\stm8s_tim1.ls
	-@erase $(OutputPath)\stm8s_tim2.ls
	-@erase $(OutputPath)\stm8s_uart1.ls
	-@erase $(OutputPath)\stm8s_wwdg.ls
	-@erase $(OutputPath)\main.ls
	-@erase $(OutputPath)\stm8_interrupt_vector.ls
endif

# 
# Release
# 
ifeq "$(CFG)" "Release"


OutputPath=Release
ProjectSFile=stm8testprog
TargetSName=stm8testprog
TargetFName=stm8testprog.elf
IntermPath=$(dir $@)
CFLAGS_PRJ=$(ToolsetBin)\cxstm8  -icustom -ilibs\inc -iinc +mods0 -pp $(ToolsetIncOpts) -cl$(IntermPath:%\=%) -co$(IntermPath:%\=%) $< 
ASMFLAGS_PRJ=$(ToolsetBin)\castm8  $(ToolsetIncOpts) -o$(IntermPath)$(InputName).$(ObjectExt) $<

all : $(OutputPath) stm8testprog.elf

$(OutputPath) : 
	if not exist $(OutputPath)/ mkdir $(OutputPath)

Release\bme280.$(ObjectExt) : custom\bme280.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h custom\bme280.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_adc1.$(ObjectExt) : libs\src\stm8s_adc1.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_adc1.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_awu.$(ObjectExt) : libs\src\stm8s_awu.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_awu.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_beep.$(ObjectExt) : libs\src\stm8s_beep.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_beep.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_clk.$(ObjectExt) : libs\src\stm8s_clk.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_clk.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_exti.$(ObjectExt) : libs\src\stm8s_exti.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_exti.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_flash.$(ObjectExt) : libs\src\stm8s_flash.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_flash.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_gpio.$(ObjectExt) : libs\src\stm8s_gpio.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_gpio.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_i2c.$(ObjectExt) : libs\src\stm8s_i2c.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_i2c.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_itc.$(ObjectExt) : libs\src\stm8s_itc.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_itc.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_iwdg.$(ObjectExt) : libs\src\stm8s_iwdg.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_iwdg.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_rst.$(ObjectExt) : libs\src\stm8s_rst.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_rst.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_spi.$(ObjectExt) : libs\src\stm8s_spi.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_spi.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_tim1.$(ObjectExt) : libs\src\stm8s_tim1.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_tim1.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_tim2.$(ObjectExt) : libs\src\stm8s_tim2.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_tim2.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_uart1.$(ObjectExt) : libs\src\stm8s_uart1.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_uart1.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_wwdg.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8s_wwdg.$(ObjectExt) : libs\src\stm8s_wwdg.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s_wwdg.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\main.$(ObjectExt) : main.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h libs\inc\stm8s.h libs\inc\stm8s_conf.h libs\inc\stm8s_adc1.h libs\inc\stm8s_awu.h libs\inc\stm8s_beep.h libs\inc\stm8s_clk.h libs\inc\stm8s_exti.h libs\inc\stm8s_flash.h libs\inc\stm8s_gpio.h libs\inc\stm8s_i2c.h libs\inc\stm8s_itc.h libs\inc\stm8s_iwdg.h libs\inc\stm8s_rst.h libs\inc\stm8s_spi.h libs\inc\stm8s_tim1.h libs\inc\stm8s_tim2.h libs\inc\stm8s_uart1.h libs\inc\stm8s_wwdg.h custom\bme280.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

Release\stm8_interrupt_vector.$(ObjectExt) : stm8_interrupt_vector.c c:\cosmic\fse_compilers\cxstm8\hstm8\mods0.h 
	@if not exist $(dir $@)  mkdir $(dir $@)
	$(CFLAGS_PRJ)

stm8testprog.elf :  $(OutputPath)\bme280.o $(OutputPath)\stm8s_adc1.o $(OutputPath)\stm8s_awu.o $(OutputPath)\stm8s_beep.o $(OutputPath)\stm8s_clk.o $(OutputPath)\stm8s_exti.o $(OutputPath)\stm8s_flash.o $(OutputPath)\stm8s_gpio.o $(OutputPath)\stm8s_i2c.o $(OutputPath)\stm8s_itc.o $(OutputPath)\stm8s_iwdg.o $(OutputPath)\stm8s_rst.o $(OutputPath)\stm8s_spi.o $(OutputPath)\stm8s_tim1.o $(OutputPath)\stm8s_tim2.o $(OutputPath)\stm8s_uart1.o $(OutputPath)\stm8s_wwdg.o $(OutputPath)\main.o $(OutputPath)\stm8_interrupt_vector.o $(OutputPath)\stm8testprog.lkf
	$(ToolsetBin)\clnk  $(ToolsetLibOpts) -o $(OutputPath)\$(TargetSName).sm8 $(OutputPath)\$(TargetSName).lkf 
	$(ToolsetBin)\cvdwarf  $(OutputPath)\$(TargetSName).sm8 

	$(ToolsetBin)\chex  -o $(OutputPath)\$(TargetSName).s19 $(OutputPath)\$(TargetSName).sm8
clean : 
	-@erase $(OutputPath)\bme280.o
	-@erase $(OutputPath)\stm8s_adc1.o
	-@erase $(OutputPath)\stm8s_awu.o
	-@erase $(OutputPath)\stm8s_beep.o
	-@erase $(OutputPath)\stm8s_clk.o
	-@erase $(OutputPath)\stm8s_exti.o
	-@erase $(OutputPath)\stm8s_flash.o
	-@erase $(OutputPath)\stm8s_gpio.o
	-@erase $(OutputPath)\stm8s_i2c.o
	-@erase $(OutputPath)\stm8s_itc.o
	-@erase $(OutputPath)\stm8s_iwdg.o
	-@erase $(OutputPath)\stm8s_rst.o
	-@erase $(OutputPath)\stm8s_spi.o
	-@erase $(OutputPath)\stm8s_tim1.o
	-@erase $(OutputPath)\stm8s_tim2.o
	-@erase $(OutputPath)\stm8s_uart1.o
	-@erase $(OutputPath)\stm8s_wwdg.o
	-@erase $(OutputPath)\main.o
	-@erase $(OutputPath)\stm8_interrupt_vector.o
	-@erase $(OutputPath)\stm8testprog.elf
	-@erase $(OutputPath)\stm8testprog.map
	-@erase $(OutputPath)\stm8testprog.st7
	-@erase $(OutputPath)\stm8testprog.s19
	-@erase $(OutputPath)\bme280.ls
	-@erase $(OutputPath)\stm8s_adc1.ls
	-@erase $(OutputPath)\stm8s_awu.ls
	-@erase $(OutputPath)\stm8s_beep.ls
	-@erase $(OutputPath)\stm8s_clk.ls
	-@erase $(OutputPath)\stm8s_exti.ls
	-@erase $(OutputPath)\stm8s_flash.ls
	-@erase $(OutputPath)\stm8s_gpio.ls
	-@erase $(OutputPath)\stm8s_i2c.ls
	-@erase $(OutputPath)\stm8s_itc.ls
	-@erase $(OutputPath)\stm8s_iwdg.ls
	-@erase $(OutputPath)\stm8s_rst.ls
	-@erase $(OutputPath)\stm8s_spi.ls
	-@erase $(OutputPath)\stm8s_tim1.ls
	-@erase $(OutputPath)\stm8s_tim2.ls
	-@erase $(OutputPath)\stm8s_uart1.ls
	-@erase $(OutputPath)\stm8s_wwdg.ls
	-@erase $(OutputPath)\main.ls
	-@erase $(OutputPath)\stm8_interrupt_vector.ls
endif
