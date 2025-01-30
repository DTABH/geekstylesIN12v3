I used the original source code 
formated it, added On/Off time during the night
and made changes to RGB-configuration

Starting with no experience with ST-programming
I used ST Visual Developper with free Cosmic Compiler in Toolchain
and got besides other beginner problems these:

1. due to source code size limitation of Cosmic Compiler
in the C-Source includes stm8s_gpio.c stm8s_tim1.c stm8s_flash.c
unused functions must be deleted or commented out to reduce the size 
else Linker error "segment .text size overflow (nnnn)" occurs 

2. Uncomment the line  #define USE_FULL_ASSERT    (1) 
in stm8s_conf.h else error "symbol _assert_failed not defined" will raise 

3.
Set "C-Compiler" setting "Optimization" in ST-Programmer to "Minimize code size"
If not set and the code is to big, you get address warnings loading the bin-file to st-programmer.
example. "FILE : line 188: Address 0x97A9 is out of range and is ignored!"
You still can load the program to the chip but i got strange behaviours of the clock.
I got this problem adding display On/Off during night time to the code.

also as mentioned in the original code dont' forget 
"IMPORTANT: SET OPTIONBYTE AFR0 to Port C5, C6 & C7 to Alternative Function with the ST Visual Programmer
or Leds won't work properly"


parameter for Off and On during the Night are set via RBG Setting
As before 1,2,3 are for the Led RGB 
Then new 4 (hour) ,5 (minute) are for Off-time
and 6 (hour) ,7 (minute) for On-time
