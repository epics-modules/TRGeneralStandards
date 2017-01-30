TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

USR_INCLUDES += -I$(GENSTDS_DRIVER)

#==================================================
# build a support library

LIBRARY_IOC += trGenStds

DBD += trGenStds.dbd

trGenStds_SRCS += TRGeneralStandards.cpp
trGenStds_SRCS += LinuxGenStdsHAL.cpp

trGenStds_LIBS += trCore ADBase asyn $(EPICS_BASE_IOC_LIBS)

#=============================
# Build the IOC application

PROD_IOC = genStdsTestIoc
# genStdsTestIoc.dbd will be created and installed
DBD += genStdsTestIoc.dbd

# genStdsTestIoc.dbd will be made up from these files:
genStdsTestIoc_DBD += base.dbd

# Include dbd files from all support applications:
genStdsTestIoc_DBD += asyn.dbd ADSupport.dbd NDPluginStdArrays.dbd NDPluginAttribute.dbd trGenStds.dbd

# Add all the support libraries needed by this IOC
genStdsTestIoc_LIBS += trGenStds trCore NDPlugin ADBase asyn

# genStdsTestIoc_registerRecordDeviceDriver.cpp derives from genStdsTestIoc.dbd
genStdsTestIoc_SRCS += genStdsTestIoc_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
genStdsTestIoc_SRCS_DEFAULT += genStdsTestIocMain.cpp
genStdsTestIoc_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#genStdsTestIoc_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
genStdsTestIoc_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

