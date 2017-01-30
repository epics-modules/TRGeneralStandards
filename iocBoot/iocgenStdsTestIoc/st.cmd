#!../../bin/linux-x86_64/genStdsTestIoc

< envPaths

cd ${TOP}

## Basic configuration
# device node
epicsEnvSet("GENSTDS_DEVICE", "/dev/16ai64ssa.0")
# prefix of all records
epicsEnvSet("PREFIX", "GENSTDS")
# Size (NELM) of waveform records.
epicsEnvSet("WAVEFORM_SIZE", "2048")
# Increase CA buffer sizes, needed for larger waveforms.
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "65535")
# Device name (used in port identifiers and :name record).
epicsEnvSet("DEVICE_NAME", "genstds0")

## Thread configuration
# Priority of read thread (EPICS units 0-99)
epicsEnvSet("READ_THREAD_PRIORITY_EPICS", "50")
# Stack size of read thread (default medium).
epicsEnvSet("READ_THREAD_STACK_SIZE", "0")
# Thread priority of interrupt waiter threads (pthread units).
epicsEnvSet("WAITER_THREAD_PRIORITY_PTHREAD", "78")

## AreaDetector configuration
# Max buffers for the channel port
epicsEnvSet("MAX_AD_BUFFERS", "256")
# Max memory for the channels port
epicsEnvSet("MAX_AD_MEMORY", "0")

## Stdarrays plugins configuration.
# Blocking-callbacks for stdarrays ports.
epicsEnvSet("STDARRAYS_BLOCKING_CALLBACKS", "1")
# Queue size for stdarrays plugins (relevant only if blocking-callbacks is 0).
epicsEnvSet("STDARRAYS_QUEUE_SIZE", "3")
# Max-memory for stdarrays plugins.
epicsEnvSet("STDARRAYS_MAX_MEMORY", "-1")
# Thread priority for stdarrays plugins.
epicsEnvSet("STDARRAYS_PRIORITY", "0")
# Thread stack size for stdarrays plugins.
epicsEnvSet("STDARRAYS_STACK_SIZE", "16384")

## Scan configuration.
# SCAN for refreshing device states
epicsEnvSet("REFRESH_STATES_SCAN", "1 second")
# SCAN for slow snapshot records
epicsEnvSet("REFRESH_SNAPSHOT_SCAN", "1 second")


## Register all support components
dbLoadDatabase "dbd/genStdsTestIoc.dbd"
genStdsTestIoc_registerRecordDeviceDriver pdbbase

# Initialize the main port.
genStdsInitDevice("$(DEVICE_NAME)", "$(GENSTDS_DEVICE)", "$(READ_THREAD_PRIORITY_EPICS)", "$(READ_THREAD_STACK_SIZE)", "$(WAITER_THREAD_PRIORITY_PTHREAD)", "$(MAX_AD_BUFFERS)", "$(MAX_AD_MEMORY)")

# Initialize the channel ports (generated using gen_channels.py).
< iocBoot/iocgenStdsTestIoc/genStdsInitChannels.cmd

# Uncomment to initialize NDAttrPlugin to test sample rate.
#NDAttrConfigure("$(DEVICE_NAME)_sample_rate_attr", 3, 1, "$(DEVICE_NAME)_channels", 0, 10, -1, 0, 16384)

# Load main records.
dbLoadRecords("$(TRANSRECORDER_CORE)/db/TRBase.db", "PREFIX=$(PREFIX), PORT=$(DEVICE_NAME), DEVICE_NAME=$(DEVICE_NAME), SIZE=$(WAVEFORM_SIZE), PRESAMPLES=#")
dbLoadRecords("db/TRGeneralStandards.db", "PREFIX=$(PREFIX), PORT=$(DEVICE_NAME), REFRESH_STATES_SCAN=$(REFRESH_STATES_SCAN)")
dbLoadRecords("$(TRANSRECORDER_CORE)/db/TRGenericRequest.db", "PREFIX=$(PREFIX), PORT=$(DEVICE_NAME), REQUEST=INITIALIZE")

# Load channel-specific records (generated using gen_channels.py).
< iocBoot/iocgenStdsTestIoc/genStdsLoadChannelsDb.cmd

# Uncomment to load records for NDAttrPlugin waveforms.
#dbLoadRecords("$(TRANSRECORDER_CORE)/db/TRSampleRateAttrTest.db", "PREFIX=$(PREFIX), ATTR_PORT=$(DEVICE_NAME)_sample_rate_attr")

# Initialize IOC.
cd ${TOP}/iocBoot/${IOC}
iocInit
