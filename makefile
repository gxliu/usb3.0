FX3FWROOT="C:\Program Files\Cypress\EZ-USB FX3 SDK\1.2\"
FX3PFWROOT="C:\Program Files\Cypress\EZ-USB FX3 SDK\1.2\u3p_firmware"

all:compile

include $(FX3FWROOT)/common/fx3_build_config.mak

MODULE = frmwexam

SOURCE += $(MODULE).c
SOURCE += dscr.c

C_OBJECT=$(SOURCE:%.c=./%.o)
A_OBJECT=$(SOURCE_ASM:%.S=./%.o)

EXES = $(MODULE).$(EXEEXT)

$(MODULE).$(EXEEXT): $(A_OBJECT) $(C_OBJECT)
	$(LINK)

$(C_OBJECT) : %.o : %.c frmwexam.h
	$(COMPILE)

$(A_OBJECT) : %.o : %.S
	$(ASSEMBLE)

clean:
	rm -f ./$(MODULE).$(EXEEXT)
	rm -f ./$(MODULE).map
	rm -f ./*.o

compile: $(C_OBJECT) $(A_OBJECT) $(EXES)

#[]#
