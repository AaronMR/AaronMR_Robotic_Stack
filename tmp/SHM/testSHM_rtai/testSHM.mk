
MAKECMD         = make
HOST=UNIX
BUILD=yes
SYS_TARGET_FILE = rtai.tlc

MODEL           = testSHM
MODULES         = rtGetInf.c rtGetNaN.c rt_logging.c rt_matrx.c rt_printf.c testSHM_data.c 
MAKEFILE        = testSHM.mk
MATLAB_ROOT     = /usr/local/matlab
S_FUNCTIONS     = sSHM.c sfun_rtai_scope.c
S_FUNCTIONS_LIB = 
SOLVER          = 
NUMST           = 1
TID01EQ         = 0
NCSTATES        = 0
COMPUTER        = GLNX86
BUILDARGS       =  GENERATE_REPORT=0 MODELLIB=testSHMlib.lib RELATIVE_PATH_TO_ANCHOR=.. MODELREF_TARGET_TYPE=NONE
MULTITASKING    = 0

LINUX_HOME = /usr/src/linux
RTAIDIR    = /usr/realtime
RTW_R      = $(MATLAB_ROOT)/rtw/c
RTAI_RTW   = $(RTW_R)/rtai
COMEDI_HOME = /usr/local/src/comedi

#--------------------------- Tool Specifications -------------------------------

include $(MATLAB_ROOT)/rtw/c/tools/unixtools.mk

#------------------------------ Include Path -----------------------------------

MATLAB_INCLUDES = \
	-I$(MATLAB_ROOT)/simulink/include \
        -I$(MATLAB_ROOT)/extern/include \
	-I$(MATLAB_ROOT)/rtw/c/src \
	-I$(MATLAB_ROOT)/rtw/c/libsrc \
	-I$(MATLAB_ROOT)/rtw/c/rtai/devices
 
USER_INCLUDES = -I$(LINUX_HOME)/include -I$(RTAIDIR)/include -I$(RTAI_RTW)/include -I$(COMEDI_HOME)/include

# Additional file include paths
ADD_INCLUDES = \
       -ID:\SVN\misc\programming\SHM\testSHM_rtai \
       -ID:\SVN\misc\programming\SHM \


INCLUDES = -I. -I.. $(MATLAB_INCLUDES) $(ADD_INCLUDES) $(USER_INCLUDES) \
	$(INSTRUMENT_INCLUDES)

#-------------------------------- C Flags --------------------------------------

# Optimization Options
OPT_OPTS = -O2 

# General User Options
OPTS =
DEB_OPT = -DDBGPRT

ANSI_OPTS = 

# Compiler options, etc:
CC_OPTS = -Wall $(DEB_OPT) $(OPT_OPTS) $(OPTS) $(ANSI_OPTS) $(EXT_CC_OPTS) \
	$(LOG_OPTS)

CPP_REQ_DEFINES = -DMODEL=$(MODEL) -DRT -DNUMST=$(NUMST) \
		  -DTID01EQ=$(TID01EQ) -DNCSTATES=$(NCSTATES) -DUNIX \
		  -DMT=$(MULTITASKING) 

CFLAGS = -D_GNU_SOURCE -DRT -DUSE_RTMODEL $(CC_OPTS) $(CPP_REQ_DEFINES) $(INCLUDES) $(NOVERSION) -ffast-math -c

RT_MAIN_DEFINES = 
LDFLAGS =

#-------------------------- Additional Libraries ------------------------------

SYSLIBS = $(EXT_LIB) -lpthread -lm -lpcan

LIBS = $(RTAIDIR)/lib/liblxrt.a
 
LIBS += $(S_FUNCTIONS_LIB) $(INSTRUMENT_LIBS)

#----------------------------- Source Files ------------------------------------

REQ_SRCS  = $(MODEL).c $(MODULES) rtmain.c rt_sim.c rt_nonfinite.c $(EXT_SRC)
USER_SRCS =
USER_OBJS = 
LOCAL_USER_OBJS = $(notdir $(USER_OBJS))

SRCS      = $(REQ_SRCS) $(S_FUNCTIONS) $(SOLVER)
OBJS      = $(SRCS:.c=.o) $(USER_OBJS)
LINK_OBJS = $(SRCS:.c=.o) $(LOCAL_USER_OBJS)

PROGRAM = ../$(MODEL)

#--------------------------------- Rules ---------------------------------------

$(PROGRAM) : $(OBJS) $(LIBS)
	gcc  $(LDFLAGS) -o $@ $(LINK_OBJS) $(LIBS) $(SYSLIBS)
	@echo "### Created executable: $(PROGRAM) ###"

%.o : %.c
	gcc -c  $(CFLAGS) $<

%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	gcc -c $(CFLAGS) $<

rtmain.o : $(MATLAB_ROOT)/rtw/c/rtai/rtmain.c $(MODEL).c 
	gcc -c $(CFLAGS) $(RT_MAIN_DEFINES) $(MATLAB_ROOT)/rtw/c/rtai/rtmain.c

%.o : $(MATLAB_ROOT)/rtw/c/rtai/%.c
	gcc -c $(CFLAGS) $(RT_MAIN_DEFINES) $<

sfun_comedi%.o : $(MATLAB_ROOT)/rtw/c/rtai/devices/sfun_comedi%.c
	gcc -c $(CFLAGS) -I$(COMEDI_HOME)/include $<

%.o : $(MATLAB_ROOT)/rtw/c/rtai/devices/%.c
	gcc -c $(CFLAGS) $<

%.o : $(MATLAB_ROOT)/rtw/c/rtai/lib/%.c
	gcc -c $(CFLAGS) $<

%.o : $(MATLAB_ROOT)/rtw/c/libsrc/%.c
	gcc -c $(CFLAGS) $<

%.o : $(MATLAB_ROOT)/simulink/src/%.c
	gcc -c $(CFLAGS) $<



#------------------------------- Libraries -------------------------------------





#----------------------------- Dependencies ------------------------------------

$(OBJS) : $(MAKEFILE) rtw_proj.tmw

#--------- Miscellaneous rules to purge and clean ------------------------------

purge : clean
	@echo "### Deleting the generated source code for $(MODEL)"
	@\rm -f $(MODEL).c $(MODEL).h $(MODEL).prm $(MODEL).reg $(MODEL).rtw \
	        $(MODULES) rtw_proj.tmw $(MAKEFILE)

clean :
	@echo "### Deleting the objects and $(PROGRAM)"
	@\rm -f $(MODULES_rtwlib) $(MODULES_dsp_rt)
	@\rm -f $(LINK_OBJS) $(PROGRAM)

# EOF: testSHM.mk
