/* This file is part of the General Standards Digitizer Driver.
 * It is subject to the license terms in the LICENSE.txt file found in the
 * top-level directory of this distribution and at
 * https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
 * of the General Standards Digitizer Driver, including this file, may be copied,
 * modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <limits.h>

#include <string>
#include <cstring>

#include <errlog.h>

#include <16ai64ssa.h>

#include "GenStdsHAL.h"

class LinuxGenStdsHAL: public GenStdsHAL
{
    static int const InterruptThreadSchedPolicy = SCHED_FIFO;
    
public:
    LinuxGenStdsHAL(char const *device_node, int waiter_priority);
    ~LinuxGenStdsHAL();
    
    GenStatus Open();
    void Close();
    
    GenStatus SetInputMode(InputMode arg);
    GetResult<InputMode> GetInputMode();   
    
    GenStatus SetVoltageRange(VoltageRange arg);
    GetResult<VoltageRange> GetVoltageRange();
    
    GenStatus SetChannelActiveRange(ChannelActiveRange arg);
    GetResult<ChannelActiveRange> GetChannelActiveRange();
    
    GenStatus SetChannel(ChannelType channel_type, int32_t channel_number);
    GetResult<int32_t> GetChannel(ChannelType channel_type);
    
    GenStatus SetRateGenEnabled(RateGen gen, OnOffState state);
    GetResult<OnOffState> GetRateGenEnabled(RateGen gen);
    
    GenStatus SetRateGenDivisor(RateGen gen, int32_t value);
    GetResult<int32_t> GetRateGenDivisor(RateGen gen);
    
    GenStatus SetRateGenBSource(RateBSource source);
    GetResult<RateBSource> GetRateGenBSource();
    
    GenStatus SetSampleClockSource(SampleClockSource source);
    GetResult<SampleClockSource> GetSampleClockSource();
    
    GenStatus SetExternalSync(OnOffState arg);
    GetResult<OnOffState> GetExternalSync();
    
    GenStatus SetAuxLineMode(AuxLine line, AuxLineMode arg);
    GetResult<AuxLineMode> GetAuxLineMode(AuxLine line);
    
    GenStatus SetAuxInvertInMode(OnOffState arg);
    GetResult<OnOffState> GetAuxInvertInMode();
    
    GenStatus SetAuxInvertOutMode(OnOffState arg);
    GetResult<OnOffState> GetAuxInvertOutMode();
    
    GenStatus SetAuxNoiseSuppression(OnOffState arg);
    GetResult<OnOffState> GetAuxNoiseSuppression();
 
    GenStatus SetBurstTriggerSource(BurstTriggerSource arg);
    GetResult<BurstTriggerSource> GetBurstTriggerSource();
    
    GenStatus SetBurstSize(int32_t size);
    GetResult<int32_t> GetBurstSize();
    
    GenStatus SetThresholdLevel(int32_t level);
    GetResult<int32_t> GetThresholdLevel();

    GenStatus SendSoftwareTrigger();
    GenStatus ClearBufferOverflowFlag();
    GenStatus ClearFifoBuffer();
    GenStatus Initialize();
    GenStatus Autocalibrate();
    
    GetResult<int32_t> GetFirmwareRevision();
    GetResult<int32_t> GetMaximumInputChannels();
    GetResult<int32_t> GetMaxSampleRate();
    GetResult<int32_t> GetBufferSize();
    GetResult<int32_t> GetDeviceType();
    GetResult<int32_t> GetMasterClockFrequency();
    GetResult<int32_t> GetRateGenDivisorLimit(RateGenDivLimit limit);
    
    GetResult<BufferOverflow> GetBufferOverflow();
    GetResult<BurstStatus> GetBurstStatus();
    GetResult<int32_t> GetNumberOfBufferSamples();
    
    GenStatus SetIoMode(IoMode io_mode);
    GenStatus SetReadTimeout(uint32_t timeout_sec);
    GenStatus SetReadOverflowCheck(OnOffState enabled);
    GetResult<size_t> ReadFIFOBuffer(void *buffer, size_t num_bytes);
    GetResult<bool> AbortRead();
    
    GenStatus ConfigureInterrupt(Interrupt intr);
    GetResult<WaitResult> WaitForInterrupt(Interrupt intr, WaitCondition &cond);
    void CancelWait(Interrupt intr);
    void ResetWait(Interrupt intr);
    
private:
    // Helper functions for various ioctl requests.
    GenStatus IoctlWriteOption(char const *name, uint32_t request, int32_t value); 
    GetResult<int32_t> IoctlReadOption(char const *name, uint32_t request); 
    GetResult<int32_t> IoctlQuery(char const *name, int32_t param);
    
    // Class encapsulating waiting for a specific interrupt.
    class InterruptWaiter {
        static size_t const WaiterThreadStackSize = PTHREAD_STACK_MIN;
    public:
        InterruptWaiter(uint32_t gsc_intr, int priority);
        ~InterruptWaiter();
        
        bool start(int fd);
        void stop();
        
        GetResult<WaitResult> WaitForInterrupt (WaitCondition &cond);
        void CancelWait();
        void ResetWait();

    private:
        static void * thread_trampoline(void *obj);
        void thread_func();
    private:
        uint32_t m_gsc_intr;
        int m_wait_sched__priority;
        sem_t m_stopped_sem;
        sem_t m_event_sem;
        int m_waiter_fd;
        int volatile m_stop;
        int volatile m_cancel_wait;
        pthread_t m_thread;
        void* m_stack_buffer;
    };
    
private:
    // Device node path.
    std::string m_device_node;
    
    // Interrupt waiters.
    InterruptWaiter m_waiter_burst_start;
    InterruptWaiter m_waiter_buf_thr_l2h;
    
    // Device file descriptor (-1 if not opened).
    int m_fd;

    // Cached information.
    int32_t m_buffer_size;
};

/**
 * Post to the semaphore but only if it's currently zero.
 * This way we can use semaphores like epicsEvent.
 * Note, if signal_sem is used for multiple threads we can potentially
 * ncrement the semaphore to higher than one, but that is not an issue.
 */
inline static void signal_sem(sem_t *sem)
{
    int value;
    sem_getvalue(sem, &value);
    if (value == 0) {
        sem_post(sem);
    }
}

// Factory function.
GenStdsHAL * GetGenStdsHAL(char const *device_node, int waiter_priority)
{
    return new LinuxGenStdsHAL(device_node, waiter_priority);
}

// These macros automatically pass the request name using token stringification.
#define IOCTL_WRITE_OPTION(request, arg) IoctlWriteOption(#request, (request), (arg))
#define IOCTL_READ_OPTION(request) IoctlReadOption(#request, (request))
#define IOCTL_QUERY(param) IoctlQuery(#param, (param))

// This macro is used to reduce boilerplate with propagating errors.
#define PROPAGATE_READ_ERROR(read_result) \
if ((read_result).status != success) return (read_result).status;

LinuxGenStdsHAL::LinuxGenStdsHAL(char const *device_node, int waiter_priority)
: m_device_node(device_node),
  m_waiter_burst_start(AI64SSA_WAIT_GSC_BURST_START, waiter_priority),
  m_waiter_buf_thr_l2h(AI64SSA_WAIT_GSC_IN_BUF_THR_L2H, waiter_priority),
  m_fd(-1)
{
}

LinuxGenStdsHAL::~LinuxGenStdsHAL()
{
    Close();
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::Open()
{
    do {
        // Check for already opened.
        if (m_fd >= 0) {
            errlogSevPrintf(errlogMajor, "Device already opened\n");
            goto fail0;
        }
        
        // Open the device node.
        m_fd = open(m_device_node.c_str(), O_RDWR);
        if (m_fd < 0) {
            int err = errno;
            errlogSevPrintf(errlogMajor, "Failed to open %s, errno=%d\n", m_device_node.c_str(), err);
            goto fail0;
        }
        
        // Get the buffer size.
        GetResult<int32_t> buf_result = IOCTL_QUERY(AI64SSA_QUERY_FIFO_SIZE);
        if (buf_result.status != success) {
            goto fail1;
        }
        m_buffer_size = buf_result.value;
        
        // Start interrupt waiters.
        if (!m_waiter_burst_start.start(m_fd)) {
            goto fail1;
        }
        if (!m_waiter_buf_thr_l2h.start(m_fd)) {
            goto fail2;
        }
    } while (false);
    
    return success;
    
fail2:
    m_waiter_burst_start.stop();
fail1:
    close(m_fd);
    m_fd = -1;
fail0:
    return failed;
}

void LinuxGenStdsHAL::Close()
{
    if (m_fd >= 0) {
        // Stop interrupt waiters.
        m_waiter_buf_thr_l2h.stop();
        m_waiter_burst_start.stop();
        
        // Close the device file descriptor.
        if (close(m_fd) < 0) {
            int err = errno;
            errlogSevPrintf(errlogMajor, "Failed to close device, errno=%d\n", err);
        }
        
        m_fd = -1;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::IoctlWriteOption(char const *name, uint32_t request, int32_t value)
{
    if (ioctl(m_fd, request, &value) < 0) {
        int err = errno;
        errlogSevPrintf(errlogMajor, "Failed to write %s, errno=%d\n", name, err);
        return failed;
    }
    
    return success;
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::IoctlReadOption(char const *name, uint32_t request)
{
    int32_t value = -1;
    if (ioctl(m_fd, request, &value) < 0) {
        int err = errno;
        errlogSevPrintf(errlogMajor, "Failed to read %s, errno=%d\n", name, err);
        return failed;
    }
    
    return value;
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::IoctlQuery(char const *name, int32_t param)
{
    int32_t arg = param;
    if (ioctl(m_fd, AI64SSA_IOCTL_QUERY, &arg) < 0) {
        int err = errno;
        errlogSevPrintf(errlogMajor, "Failed to query %s, errno=%d\n", name, err);
        return failed;
    }

    return arg;
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetInputMode(GenStdsHAL::InputMode arg)
{
    int32_t cntl;
    switch (arg) {
        case SingleEnded:        cntl = AI64SSA_AI_MODE_SINGLE;  break;
        case PseudoDifferential: cntl = AI64SSA_AI_MODE_PS_DIFF; break;
        case Differential:       cntl = AI64SSA_AI_MODE_DIFF;    break;
        case ZeroTest:           cntl = AI64SSA_AI_MODE_ZERO;    break;
        case ReferenceTest:      cntl = AI64SSA_AI_MODE_VREF;    break;
        default:
            errlogSevPrintf(errlogMajor, "SetInputMode: invalid argument.\n");
            return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_AI_MODE, cntl);
}

GenStdsHAL::GetResult<GenStdsHAL::InputMode> LinuxGenStdsHAL::GetInputMode()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_AI_MODE);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_AI_MODE_SINGLE:  return SingleEnded;
        case AI64SSA_AI_MODE_PS_DIFF: return PseudoDifferential;
        case AI64SSA_AI_MODE_DIFF:    return Differential;
        case AI64SSA_AI_MODE_ZERO:    return ZeroTest;
        case AI64SSA_AI_MODE_VREF:    return ReferenceTest;
        default:
            errlogSevPrintf(errlogMajor, "GetInputMode: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetVoltageRange(GenStdsHAL::VoltageRange arg)
{
    int32_t cntl;
    switch (arg) {
        case R_Bi_2_5: cntl = AI64SSA_AI_RANGE_2_5V;  break;
        case R_Uni_5:  cntl = AI64SSA_AI_RANGE_0_5V;  break;
        case R_Bi_5:   cntl = AI64SSA_AI_RANGE_5V;    break;
        case R_Uni_10: cntl = AI64SSA_AI_RANGE_0_10V; break;
        case R_Bi_10:  cntl = AI64SSA_AI_RANGE_10V;   break;
        default:
            errlogSevPrintf(errlogMajor, "SetVoltageRange: invalid argument.\n");
            return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_AI_RANGE, cntl);
}

GenStdsHAL::GetResult<GenStdsHAL::VoltageRange> LinuxGenStdsHAL::GetVoltageRange()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_AI_RANGE);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_AI_RANGE_2_5V:  return R_Bi_2_5;
        case AI64SSA_AI_RANGE_0_5V:  return R_Uni_5;
        case AI64SSA_AI_RANGE_5V:    return R_Bi_5;
        case AI64SSA_AI_RANGE_0_10V: return R_Uni_10;
        case AI64SSA_AI_RANGE_10V:   return R_Bi_10;
        default:
            errlogSevPrintf(errlogMajor, "GetVoltageRange: invalid result.\n");
            return failed;
    }
}
    
GenStdsHAL::GenStatus LinuxGenStdsHAL::SetChannelActiveRange(GenStdsHAL::ChannelActiveRange arg)
{
    int32_t cntl;
    switch (arg) {
        case ChSingle: cntl = AI64SSA_CHAN_ACTIVE_SINGLE; break;
        case Ch2:      cntl = AI64SSA_CHAN_ACTIVE_0_1;    break;
        case Ch4:      cntl = AI64SSA_CHAN_ACTIVE_0_3;    break;
        case Ch8:      cntl = AI64SSA_CHAN_ACTIVE_0_7;    break;
        case Ch16:     cntl = AI64SSA_CHAN_ACTIVE_0_15;   break;
        case Ch32:     cntl = AI64SSA_CHAN_ACTIVE_0_31;   break;
        case Ch64:     cntl = AI64SSA_CHAN_ACTIVE_0_63;   break;
        case ChRange:  cntl = AI64SSA_CHAN_ACTIVE_RANGE;  break;
        default :
            errlogSevPrintf(errlogMajor, "SetChannelActiveRange: invalid argument.\n");
            return failed;
    }

    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_CHAN_ACTIVE, cntl);
}

GenStdsHAL::GetResult<GenStdsHAL::ChannelActiveRange> LinuxGenStdsHAL::GetChannelActiveRange()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_CHAN_ACTIVE);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_CHAN_ACTIVE_SINGLE:  return ChSingle;
        case AI64SSA_CHAN_ACTIVE_0_1:     return Ch2;
        case AI64SSA_CHAN_ACTIVE_0_3:     return Ch4;
        case AI64SSA_CHAN_ACTIVE_0_7:     return Ch8;
        case AI64SSA_CHAN_ACTIVE_0_15:    return Ch16;
        case AI64SSA_CHAN_ACTIVE_0_31:    return Ch32;
        case AI64SSA_CHAN_ACTIVE_0_63:    return Ch64;
        case AI64SSA_CHAN_ACTIVE_RANGE:   return ChRange;
        default:
            errlogSevPrintf(errlogMajor, "GetChannelActiveRange: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetChannel(GenStdsHAL::ChannelType channel_type, int32_t channel_number)
{
    GetResult<int32_t> channel_max_res = IOCTL_QUERY(AI64SSA_QUERY_CHANNEL_MAX);
    PROPAGATE_READ_ERROR(channel_max_res)
    
    if (channel_number < 0 || channel_number >= channel_max_res.value) {
        errlogSevPrintf(errlogMajor, "SetChannel: channel number out of range.\n");
        return failed; 
    }
    
    switch (channel_type) {
        case ChTypeFirst:  return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_CHAN_FIRST,  channel_number);
        case ChTypeLast:   return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_CHAN_LAST ,  channel_number);
        case ChTypeSingle: return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_CHAN_SINGLE, channel_number);
        default:
            errlogSevPrintf(errlogMajor, "SetChannel: invalid channel type.\n");
            return failed;
    }
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetChannel(GenStdsHAL::ChannelType channel_type)
{
    switch (channel_type) {
        case ChTypeFirst:  return IOCTL_READ_OPTION(AI64SSA_IOCTL_CHAN_FIRST);
        case ChTypeLast:   return IOCTL_READ_OPTION(AI64SSA_IOCTL_CHAN_LAST);
        case ChTypeSingle: return IOCTL_READ_OPTION(AI64SSA_IOCTL_CHAN_SINGLE);
        default:
            errlogSevPrintf(errlogMajor, "GetChannel: invalid channel type.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetRateGenEnabled(GenStdsHAL::RateGen gen, GenStdsHAL::OnOffState state)
{
    int32_t cntl;
    switch (state) {
        case Off: cntl = AI64SSA_GEN_ENABLE_NO;  break;
        case On:  cntl = AI64SSA_GEN_ENABLE_YES; break;
        default:
            errlogSevPrintf(errlogMajor, "SetRateGenEnabled: invalid state argument.\n");
            return failed;
    }
    
    switch (gen) {
        case GeneratorA: return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_RAG_ENABLE, cntl);
        case GeneratorB: return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_RBG_ENABLE, cntl);
        default:
            errlogSevPrintf(errlogMajor, "SetRateGenEnabled: invalid rate generator.\n");
            return failed;
    }
}

GenStdsHAL::GetResult<GenStdsHAL::OnOffState> LinuxGenStdsHAL::GetRateGenEnabled(GenStdsHAL::RateGen gen)
{
    GetResult<int32_t> read_res;
    switch (gen) {
        case GeneratorA: read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_RAG_ENABLE); break;
        case GeneratorB: read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_RBG_ENABLE); break;
        default:
            errlogSevPrintf(errlogMajor, "GetRateGenEnabled: invalid rate generator.\n");
            return failed;
    }
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_GEN_ENABLE_NO:  return Off;
        case AI64SSA_GEN_ENABLE_YES: return On;
        default:
            errlogSevPrintf(errlogMajor, "GetRateGenEnabled: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetRateGenDivisor(GenStdsHAL::RateGen gen, int32_t value)
{
    GetResult<int32_t> min_rate_res = IOCTL_QUERY(AI64SSA_QUERY_NRATE_MIN);
    PROPAGATE_READ_ERROR(min_rate_res)
    GetResult<int32_t> max_rate_res = IOCTL_QUERY(AI64SSA_QUERY_NRATE_MAX);
    PROPAGATE_READ_ERROR(max_rate_res)
    
    if (value < min_rate_res.value || value > max_rate_res.value) {
        errlogSevPrintf(errlogMajor, "SetRateGenDivisor: divisor %d not in range [%d, %d].\n",
            (int)value, (int)min_rate_res.value, (int)max_rate_res.value);
        return failed;
    }
    
    switch (gen) {
        case GeneratorA: return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_RAG_NRATE, value);
        case GeneratorB: return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_RBG_NRATE, value);
        default:
            errlogSevPrintf(errlogMajor, "SetRateGenDivisor: invalid rate generator.\n");
            return failed;
    }
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetRateGenDivisor(GenStdsHAL::RateGen gen)
{
    switch (gen) {
        case GeneratorA: return IOCTL_READ_OPTION(AI64SSA_IOCTL_RAG_NRATE);
        case GeneratorB: return IOCTL_READ_OPTION(AI64SSA_IOCTL_RBG_NRATE);
        default:
            errlogSevPrintf(errlogMajor, "GetRateGenDivisor: invalid rate generator.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetRateGenBSource(GenStdsHAL::RateBSource source)
{
    int32_t cntl;
    switch (source) {
        case RateBSrcMaster: cntl = AI64SSA_RBG_CLK_SRC_MASTER; break;
        case RateBSrcRateA:  cntl = AI64SSA_RBG_CLK_SRC_RAG;    break;
        default:
            errlogSevPrintf(errlogMajor, "SetRateGenBSource: invalid source.\n");
            return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_RBG_CLK_SRC, cntl);
}

GenStdsHAL::GetResult<GenStdsHAL::RateBSource> LinuxGenStdsHAL::GetRateGenBSource()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_RBG_CLK_SRC);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_RBG_CLK_SRC_MASTER: return RateBSrcMaster;
        case AI64SSA_RBG_CLK_SRC_RAG:    return RateBSrcRateA;
        default:
            errlogSevPrintf(errlogMajor, "GetRateGenBSource: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetSampleClockSource(GenStdsHAL::SampleClockSource source)
{
    int32_t cntl;
    switch (source) {
        case ClkSrcRateA:    cntl = AI64SSA_SAMP_CLK_SRC_RAG; break;
        case ClkSrcRateB:    cntl = AI64SSA_SAMP_CLK_SRC_RBG; break;
        case ClkSrcExternal: cntl = AI64SSA_SAMP_CLK_SRC_EXT; break;
        case ClkSrcSoftware: cntl = AI64SSA_SAMP_CLK_SRC_BCR; break;
        default:
            errlogSevPrintf(errlogMajor, "SetSampleClockSource: invalid source.\n");
            return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_SAMP_CLK_SRC, cntl);
}

GenStdsHAL::GetResult<GenStdsHAL::SampleClockSource> LinuxGenStdsHAL::GetSampleClockSource()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_SAMP_CLK_SRC);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_SAMP_CLK_SRC_RAG: return ClkSrcRateA;
        case AI64SSA_SAMP_CLK_SRC_RBG: return ClkSrcRateB;
        case AI64SSA_SAMP_CLK_SRC_EXT: return ClkSrcExternal;
        case AI64SSA_SAMP_CLK_SRC_BCR: return ClkSrcSoftware;
        default:
            errlogSevPrintf(errlogMajor, "GetSampleClockSource: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetExternalSync(GenStdsHAL::OnOffState arg)
{
    int32_t cntl;
    switch (arg) {
        case Off: cntl = AI64SSA_EXT_SYNC_ENABLE_NO;  break;
        case On:  cntl = AI64SSA_EXT_SYNC_ENABLE_YES; break;
        default:
            errlogSevPrintf(errlogMajor, "SetExternalSync: invalid argument.\n");
            return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_EXT_SYNC_ENABLE, cntl);
}

GenStdsHAL::GetResult<GenStdsHAL::OnOffState> LinuxGenStdsHAL::GetExternalSync()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_EXT_SYNC_ENABLE);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_EXT_SYNC_ENABLE_NO:  return Off;
        case AI64SSA_EXT_SYNC_ENABLE_YES: return On;
        default:
            errlogSevPrintf(errlogMajor, "GetExternalSync: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetAuxLineMode(GenStdsHAL::AuxLine line, GenStdsHAL::AuxLineMode mode)
{
    int32_t cntl;
    switch (mode) {
        case AuxModeDisable: cntl = AI64SSA_AUX_MODE_DISABLE; break;
        case AuxModeInput:   cntl = AI64SSA_AUX_MODE_INPUT;   break;
        case AuxModeOutput:  cntl = AI64SSA_AUX_MODE_OUTPUT;  break;
        default:
            errlogSevPrintf(errlogMajor, "SetAuxLineMode: invalid mode.\n");
            return failed;
    }
    
    switch (line) {
        case Aux0: return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_AUX_0_MODE, cntl);
        case Aux1: return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_AUX_1_MODE, cntl);
        case Aux2: return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_AUX_2_MODE, cntl);
        case Aux3: return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_AUX_3_MODE, cntl);
        default:
            errlogSevPrintf(errlogMajor, "SetAuxLineMode: invalid line.\n");
            return failed;
    }
}

GenStdsHAL::GetResult<GenStdsHAL::AuxLineMode> LinuxGenStdsHAL::GetAuxLineMode(GenStdsHAL::AuxLine line)
{
    GetResult<int32_t> read_res;
    switch (line) {
        case Aux0: read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_AUX_0_MODE); break;
        case Aux1: read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_AUX_1_MODE); break;
        case Aux2: read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_AUX_2_MODE); break;
        case Aux3: read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_AUX_3_MODE); break;
        default:
            errlogSevPrintf(errlogMajor, "GetAuxLineMode: invalid line.\n");
            return failed;
    }
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_AUX_MODE_DISABLE: return AuxModeDisable;
        case AI64SSA_AUX_MODE_INPUT:   return AuxModeInput;
        case AI64SSA_AUX_MODE_OUTPUT:  return AuxModeOutput;
        default:
            errlogSevPrintf(errlogMajor, "GetAuxLineMode: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetAuxInvertInMode(GenStdsHAL::OnOffState arg)
{
    int32_t cntl;
    switch (arg) {
        case Off: cntl = AI64SSA_AUX_IN_POL_LO_2_HI; break;
        case On:  cntl = AI64SSA_AUX_IN_POL_HI_2_LO; break;
        default:
            errlogSevPrintf(errlogMajor, "SetAuxInvertInMode: invalid argument.\n");
            return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_AUX_IN_POL, cntl);
}

GenStdsHAL::GetResult<GenStdsHAL::OnOffState> LinuxGenStdsHAL::GetAuxInvertInMode()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_AUX_IN_POL);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_AUX_IN_POL_LO_2_HI: return Off;
        case AI64SSA_AUX_IN_POL_HI_2_LO: return On;
        default:
            errlogSevPrintf(errlogMajor, "GetAuxInvertInMode: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetAuxInvertOutMode(GenStdsHAL::OnOffState arg)
{
    int32_t cntl;
    switch (arg) {
        case Off: cntl = AI64SSA_AUX_OUT_POL_HI_PULSE;  break;
        case On:  cntl = AI64SSA_AUX_OUT_POL_LOW_PULSE; break;
        default:
            errlogSevPrintf(errlogMajor, "SetAuxInvertOutMode: invalid argument.\n");
            return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_AUX_OUT_POL, cntl);
}

GenStdsHAL::GetResult<GenStdsHAL::OnOffState> LinuxGenStdsHAL::GetAuxInvertOutMode()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_AUX_OUT_POL);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_AUX_OUT_POL_HI_PULSE:  return Off;
        case AI64SSA_AUX_OUT_POL_LOW_PULSE: return On;
        default:
            errlogSevPrintf(errlogMajor, "GetAuxInvertOutMode: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetAuxNoiseSuppression(GenStdsHAL::OnOffState arg)
{
    int32_t cntl;
    switch (arg) {
        case Off: cntl = AI64SSA_AUX_NOISE_LOW;  break;
        case On:  cntl = AI64SSA_AUX_NOISE_HIGH; break;
        default:
            errlogSevPrintf(errlogMajor, "SetAuxNoiseSuppression: invalid argument.\n");
            return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_AUX_NOISE, cntl);
}

GenStdsHAL::GetResult<GenStdsHAL::OnOffState> LinuxGenStdsHAL::GetAuxNoiseSuppression()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_AUX_NOISE);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_AUX_NOISE_LOW:  return Off;
        case AI64SSA_AUX_NOISE_HIGH: return On;
        default:
            errlogSevPrintf(errlogMajor, "GetAuxNoiseSuppression: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetBurstTriggerSource(GenStdsHAL::BurstTriggerSource arg)
{
    int32_t cntl;
    switch (arg) {
        case DisableBurst: cntl = AI64SSA_BURST_SRC_DISABLE; break;
        case RateBGen:     cntl = AI64SSA_BURST_SRC_RBG;     break;
        case ExtInputSync: cntl = AI64SSA_BURST_SRC_EXT;     break;
        case BcrOutSync:   cntl = AI64SSA_BURST_SRC_BCR;     break;
        default:
            errlogSevPrintf(errlogMajor, "SetBurstTriggerSource: wrong argument.\n");
            return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_BURST_SRC, cntl);
}

GenStdsHAL::GetResult<GenStdsHAL::BurstTriggerSource> LinuxGenStdsHAL::GetBurstTriggerSource()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_BURST_SRC);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_BURST_SRC_DISABLE:  return DisableBurst;
        case AI64SSA_BURST_SRC_RBG:      return RateBGen;
        case AI64SSA_BURST_SRC_EXT:      return ExtInputSync;
        case AI64SSA_BURST_SRC_BCR:      return BcrOutSync;
        default:
            errlogSevPrintf(errlogMajor, "GetBurstTriggerSource: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetBurstSize(int32_t size)
{
    if (size < 0) {
        errlogSevPrintf(errlogMajor, "SetBurstSize: argument too small.\n");
        return failed;
    }
    if (size > 0xFFFFF) {
        errlogSevPrintf(errlogMajor, "SetBurstSize: argument too large.\n");
        return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_BURST_SIZE, size);
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetBurstSize()
{
    return IOCTL_READ_OPTION(AI64SSA_IOCTL_BURST_SIZE);
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetThresholdLevel(int32_t level)
{
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_AI_BUF_THR_LVL, level);
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetThresholdLevel()
{
    return IOCTL_READ_OPTION(AI64SSA_IOCTL_AI_BUF_THR_LVL);
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SendSoftwareTrigger()
{
    // TODO: The driver uses a different bit for some devices. Bit 12 is in the
    // device manual but the driver uses a different bit for some devices.
    
    gsc_reg_t reg;
    reg.reg = AI64SSA_GSC_BCTLR;
    reg.value = (uint32_t)1 << 12;
    reg.mask = reg.value;
    if (ioctl(m_fd, AI64SSA_IOCTL_REG_MOD, &reg) < 0) {
        int err = errno;
        errlogSevPrintf(errlogMajor, "SendSoftwareTrigger: AI64SSA_IOCTL_REG_MOD failed, errno=%d\n", err);
        return failed;
    }
    
    return success;
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::ClearBufferOverflowFlag()
{
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_AI_BUF_OVERFLOW, AI64SSA_AI_BUF_OVERFLOW_CLEAR);
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::Initialize()
{
    if (ioctl(m_fd, AI64SSA_IOCTL_INITIALIZE) < 0) { 
        int err = errno;
        errlogSevPrintf(errlogMajor, "Initialize: AI64SSA_IOCTL_INITIALIZE failed, errno=%d\n", err);
        return failed;
    }
    
    return success;
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::Autocalibrate()
{
    if (ioctl(m_fd, AI64SSA_IOCTL_AUTO_CALIBRATE) < 0) { 
        int err = errno;
        errlogSevPrintf(errlogMajor, "Autocalibrate: AI64SSA_IOCTL_AUTO_CALIBRATE failed, errno=%d\n", err);
        return failed;
    }
    
    return success;
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::ClearFifoBuffer()
{
    if (ioctl(m_fd, AI64SSA_IOCTL_AI_BUF_CLEAR) < 0) { 
        int err = errno;
        errlogSevPrintf(errlogMajor, "ClearFifoBuffer: AI64SSA_IOCTL_AI_BUF_CLEAR failed, errno=%d\n", err);
        return failed;
    }
    
    return success;
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetFirmwareRevision()
{
    gsc_reg_t reg = {};
    reg.reg = AI64SSA_GSC_BCFGR;
    if (ioctl(m_fd, AI64SSA_IOCTL_REG_READ, &reg) < 0) {
        int err = errno;
        errlogSevPrintf(errlogMajor, "GetFirmwareRevision: AI64SSA_IOCTL_REG_READ failed, errno=%d\n", err);
        return failed;
    }
    
    return (int32_t)(reg.value & 0xFFF);
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetMaximumInputChannels()
{
    return IOCTL_QUERY(AI64SSA_QUERY_CHANNEL_MAX);
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetMaxSampleRate()
{
    return IOCTL_QUERY(AI64SSA_QUERY_FSAMP_MAX);
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetBufferSize()
{
    return m_buffer_size;
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetDeviceType()
{
    GetResult<int> read_res = IOCTL_QUERY(AI64SSA_QUERY_DEVICE_TYPE);
    PROPAGATE_READ_ERROR(read_res);
    
    switch (read_res.value) {
        case GSC_DEV_TYPE_16AI64SSA: return (int32_t)1;
        case GSC_DEV_TYPE_16AI64SSC: return (int32_t)2;
        default:
            errlogSevPrintf(errlogMajor, "GetDeviceType: unknown device.\n");
            return failed;
    }
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetMasterClockFrequency()
{
    return IOCTL_QUERY(AI64SSA_QUERY_MASTER_CLOCK);
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetRateGenDivisorLimit(GenStdsHAL::RateGenDivLimit limit)
{
    switch (limit) {
        case Low: return IOCTL_QUERY(AI64SSA_QUERY_NRATE_MIN);
        case High: return IOCTL_QUERY(AI64SSA_QUERY_NRATE_MAX);
        default:
            errlogSevPrintf(errlogMajor, "GetRateGenDivisorLimit: wrong argument.\n");
            return failed;
    }
}

GenStdsHAL::GetResult<GenStdsHAL::BufferOverflow> LinuxGenStdsHAL::GetBufferOverflow()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_AI_BUF_OVERFLOW);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_AI_BUF_OVERFLOW_NO:  return NoOverflow;
        case AI64SSA_AI_BUF_OVERFLOW_YES: return Overflow;
        default:
            errlogSevPrintf(errlogMajor, "GetBufferOverflow: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GetResult<GenStdsHAL::BurstStatus> LinuxGenStdsHAL::GetBurstStatus()
{
    GetResult<int32_t> read_res = IOCTL_READ_OPTION(AI64SSA_IOCTL_BURST_STATUS);
    PROPAGATE_READ_ERROR(read_res)
    
    switch (read_res.value) {
        case AI64SSA_BURST_STATUS_IDLE:   return Idle;
        case AI64SSA_BURST_STATUS_ACTIVE: return Active;
        default:
            errlogSevPrintf(errlogMajor, "GetBurstStatus: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GetResult<int32_t> LinuxGenStdsHAL::GetNumberOfBufferSamples()
{
    return IOCTL_READ_OPTION(AI64SSA_IOCTL_AI_BUF_LEVEL);
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetIoMode(GenStdsHAL::IoMode io_mode)
{
    int32_t ctrl;
    switch (io_mode) {
        case IoModeDMA:   ctrl = GSC_IO_MODE_DMA;   break;
        case IoModeDMDMA: ctrl = GSC_IO_MODE_DMDMA; break;
        case IoModePIO:   ctrl = GSC_IO_MODE_PIO;   break;
        default:
            errlogSevPrintf(errlogMajor, "SetIoMode: invalid argument\n");
            return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_RX_IO_MODE, ctrl);
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetReadTimeout(uint32_t timeout_sec)
{
    int32_t const max_timeout = 3600;
    int32_t value;
    if (timeout_sec == ReadTimeoutInfinite) {
        value = AI64SSA_IO_TIMEOUT_INFINITE;
    } else {
        value = (timeout_sec > max_timeout) ? max_timeout : timeout_sec;
    }
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_RX_IO_TIMEOUT, value);
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::SetReadOverflowCheck(OnOffState enabled)
{
    int32_t ctrl;
    switch (enabled) {
        case Off: ctrl = AI64SSA_IO_OVERFLOW_IGNORE; break;
        case On:  ctrl = AI64SSA_IO_OVERFLOW_CHECK;  break;
        default:
            errlogSevPrintf(errlogMajor, "SetReadOverflowCheck: invalid argument\n");
            return failed;
    }
    
    return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_RX_IO_OVERFLOW, ctrl);
}

GenStdsHAL::GetResult<size_t> LinuxGenStdsHAL::ReadFIFOBuffer(void *buffer, size_t num_bytes)
{
    ssize_t status = read(m_fd, buffer, num_bytes);
    if (status < 0) {
        int err = errno;
        if (err == ETIMEDOUT) {
            return timeout;
        }
        errlogSevPrintf(errlogMajor, "ReadFIFOBuffer: read failed, errno=%d\n", err);
        return failed;
    }
    
    if ((size_t)status > num_bytes) {
        errlogSevPrintf(errlogMajor, "ReadFIFOBuffer: read too much (%u out of %u)\n",
                        (unsigned int)status, (unsigned int)num_bytes);
        return failed;
    }
    
    return (size_t)status;
}

GenStdsHAL::GetResult<bool> LinuxGenStdsHAL::AbortRead()
{
    int32_t result = -1;
    if (ioctl(m_fd, AI64SSA_IOCTL_RX_IO_ABORT, &result) < 0) { 
        int err = errno;
        errlogSevPrintf(errlogMajor, "AbortRead: AI64SSA_IOCTL_RX_IO_ABORT failed, errno=%d\n", err);
        return failed;
    }
    
    switch (result) {
        case AI64SSA_IO_ABORT_NO:  return false;
        case AI64SSA_IO_ABORT_YES: return true;
        default:
            errlogSevPrintf(errlogMajor, "AbortRead: invalid result.\n");
            return failed;
    }
}

GenStdsHAL::GenStatus LinuxGenStdsHAL::ConfigureInterrupt(GenStdsHAL::Interrupt intr)
{
    switch (intr) {
        case Irq0None:       return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_IRQ0_SEL, AI64SSA_IRQ0_INIT_DONE);
        case Irq0BurstStart: return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_IRQ0_SEL, AI64SSA_IRQ0_BURST_START);
        case Irq1None:       return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_IRQ1_SEL, AI64SSA_IRQ1_NONE);
        case Irq1BufThrL2H:  return IOCTL_WRITE_OPTION(AI64SSA_IOCTL_IRQ1_SEL, AI64SSA_IRQ1_IN_BUF_THR_L2H);
        default:
            errlogSevPrintf(errlogMajor, "ConfigureInterrupt: invalid interrupt\n");
            return failed;
    }
}

GenStdsHAL::GetResult<GenStdsHAL::WaitResult>
LinuxGenStdsHAL::WaitForInterrupt(GenStdsHAL::Interrupt intr, GenStdsHAL::WaitCondition &cond)
{
    switch (intr) {
        case Irq0BurstStart: return m_waiter_burst_start.WaitForInterrupt(cond);
        case Irq1BufThrL2H:  return m_waiter_buf_thr_l2h.WaitForInterrupt(cond);
        default:
            errlogSevPrintf(errlogMajor, "WaitForInterrupt: invalid interrupt\n");
            return failed;
    }
}

void LinuxGenStdsHAL::CancelWait(GenStdsHAL::Interrupt intr)
{
    switch (intr) {
        case Irq0BurstStart:  return m_waiter_burst_start.CancelWait();
        case Irq1BufThrL2H:   return m_waiter_buf_thr_l2h.CancelWait();
        default:
            errlogSevPrintf(errlogMajor, "CancelWait: invalid interrupt\n");
    }
}

void LinuxGenStdsHAL::ResetWait(GenStdsHAL::Interrupt intr)
{
    switch (intr) {
        case Irq0BurstStart: return m_waiter_burst_start.ResetWait();
        case Irq1BufThrL2H:  return m_waiter_buf_thr_l2h.ResetWait();
        default:
            errlogSevPrintf(errlogMajor, "ResetWait: invalid interrupt\n");
    }
}

LinuxGenStdsHAL::InterruptWaiter::InterruptWaiter(uint32_t gsc_intr, int priority)
: m_gsc_intr(gsc_intr), m_wait_sched__priority(priority)
{
    sem_init(&m_stopped_sem, 0, 0);
    sem_init(&m_event_sem, 0, 0);
}

LinuxGenStdsHAL::InterruptWaiter::~InterruptWaiter()
{
    sem_destroy(&m_event_sem);
    sem_destroy(&m_stopped_sem);
}

bool LinuxGenStdsHAL::InterruptWaiter::start(int fd)
{
    int thr_res;
    int min_priority, max_priority;
    void *m_stack_buffer;
    pthread_attr_t attr;
    struct sched_param sparam = {};
    
    // Ensure that the semaphores are at zero value.
    while (sem_trywait(&m_stopped_sem) == 0);
    while (sem_trywait(&m_event_sem) == 0);
    
    // Initialize some variables.
    m_waiter_fd = fd;
    m_stop = 0;
    m_cancel_wait = 0;
    
    // Stack prefaulting
    m_stack_buffer = mmap(NULL, WaiterThreadStackSize, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
    if (m_stack_buffer == MAP_FAILED) {
        errlogSevPrintf(errlogMajor, "mmap failed, errno=%d\n", errno);
        goto fail0;
    }
    memset(m_stack_buffer, 0, WaiterThreadStackSize);
    
    // Initialize the thread attributes.
    if ((thr_res = pthread_attr_init(&attr)) != 0) {
        errlogSevPrintf(errlogMajor, "pthread_attr_init failed, err=%d\n", thr_res);
        goto fail1;
    }
    if ((thr_res = pthread_attr_setstack(&attr, m_stack_buffer, WaiterThreadStackSize)) != 0) {
        errlogSevPrintf(errlogMajor, "pthread setstack failed, err=%d\n", thr_res);
        goto fail2;
    }
    if ((thr_res = pthread_attr_setschedpolicy(&attr, InterruptThreadSchedPolicy)) != 0) {
        errlogSevPrintf(errlogMajor, "pthread_attr_setschedpolicy failed, err=%d\n", thr_res);
        goto fail2;
    }
    min_priority = sched_get_priority_min(InterruptThreadSchedPolicy);
    max_priority = sched_get_priority_max(InterruptThreadSchedPolicy);
    if (m_wait_sched__priority < sched_get_priority_min(InterruptThreadSchedPolicy) || m_wait_sched__priority > max_priority) {
        errlogSevPrintf(errlogMajor, "pthread priority not in range: [%i, %i]\n", min_priority, max_priority);
        goto fail2;
    }
    sparam.sched_priority = m_wait_sched__priority;
    if ((thr_res = pthread_attr_setschedparam(&attr, &sparam)) != 0) {
        errlogSevPrintf(errlogMajor, "pthread_attr_setschedparam failed, err=%d\n", thr_res);
        goto fail2;
    }
    if ((thr_res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) != 0) {
        errlogSevPrintf(errlogMajor, "pthread attr_set_inherit_sched failed, err=%d\n", thr_res);
        goto fail2;
    }
    
    // Start the thread which waits for interrupts.
    thr_res = pthread_create(&m_thread, &attr, thread_trampoline, this);
    
    if (thr_res != 0) {
        errlogSevPrintf(errlogMajor, "pthread_create failed, err=%d\n", thr_res);
        goto fail2;
    }
    pthread_attr_destroy(&attr);
    
    return true;
    
fail2:
    pthread_attr_destroy(&attr);
fail1:
    munmap(m_stack_buffer, WaiterThreadStackSize);
fail0:
    return false;
}

void LinuxGenStdsHAL::InterruptWaiter::stop()
{
    // Set the stop flag for the thread.
    m_stop = 1;
    
    while (true) {
        // Perform a cancel-wait request to the driver.
        gsc_wait_t wait = {};
        wait.gsc = m_gsc_intr;
        if (ioctl(m_waiter_fd, AI64SSA_IOCTL_WAIT_CANCEL, &wait) < 0) { 
            int err = errno;
            errlogSevPrintf(errlogMajor, "AI64SSA_IOCTL_WAIT_CANCEL failed, errno=%d\n", err);
        }
        
        // Wait for the thread to report its termination.
        // We need a timeout and to retry because the thread may not have been
        // waiting in the wait ioctl when we performed the cancel request.
        struct timespec timeout;
        timeout.tv_sec = 0;
        timeout.tv_nsec = 1000000; // 1 ms
        if (sem_timedwait(&m_stopped_sem, &timeout) == 0) {
            break;
        }
    }
    
    // Join the thread.
    int thr_res = pthread_join(m_thread, NULL);
    if (thr_res != 0) {
        errlogSevPrintf(errlogMajor, "pthread_join failed, err=%d\n", thr_res);
    }
    
    // Free the stack.
    munmap(m_stack_buffer, WaiterThreadStackSize);
}

GenStdsHAL::GetResult<GenStdsHAL::WaitResult>
LinuxGenStdsHAL::InterruptWaiter::WaitForInterrupt(GenStdsHAL::WaitCondition &cond)
{
    // Clear the semaphore so that we don't unnecessarily
    // check the condition twice when we start. We are only
    // interested in interrupts ariving after this.
    sem_trywait(&m_event_sem);
    
    while (true) {
        // Check the cancel request flag.
        if (m_cancel_wait) {
            return WaitCancelled;
        }

        // Check the wait condition.
        if (cond.isMet()) {
            return WaitDone;
        }
        
        // Wait for an interrupt before checking again.
        // This will pick up any interrupt that we have
        // not picked up since the previous wait.
        sem_wait(&m_event_sem);
    }
}

void LinuxGenStdsHAL::InterruptWaiter::CancelWait()
{
    // Set the cancel request flag.
    m_cancel_wait = 1;
    
    // Poke the semaphore so that WaitForInterrupt
    // picks up our request immediately.
    signal_sem(&m_event_sem);
}

void LinuxGenStdsHAL::InterruptWaiter::ResetWait()
{
    // Reset the cancel request flag.
    m_cancel_wait = 0;
}

void * LinuxGenStdsHAL::InterruptWaiter::thread_trampoline(void *obj)
{
    ((InterruptWaiter *)obj)->thread_func();
    return NULL;
}

void LinuxGenStdsHAL::InterruptWaiter::thread_func()
{
    // Load some things into local variables for minor performance benefit.
    int fd = m_waiter_fd;
    uint32_t gsc_intr = m_gsc_intr;
    
    while (!m_stop) {
        // Perform the wait-event ioctl.
        gsc_wait_t wait = {};
        wait.gsc = gsc_intr;
        wait.timeout_ms = GSC_WAIT_TIMEOUT_MAX;       

        if (ioctl(fd, AI64SSA_IOCTL_WAIT_EVENT, &wait) < 0) {
            int err = errno;
            errlogSevPrintf(errlogMajor, "AI64SSA_IOCTL_WAIT_EVENT failed, errno=%d\n", err);
            // Error not expected, bail out to avoid hogging the CPU.
            break;
        }

        // Fast path is that the wait is done.
        if (__builtin_expect(wait.flags != GSC_WAIT_FLAG_DONE, 0)) {
            if (wait.flags == GSC_WAIT_FLAG_TIMEOUT) {
                // On timeout just wait again.
                continue;
            }
            else if (wait.flags == GSC_WAIT_FLAG_CANCEL) {
                // On cancelled, return from the thread.
                break;
            }
            else {
                errlogSevPrintf(errlogMajor, "invalid result from AI64SSA_IOCTL_WAIT_EVENT\n");
                // Not expected, bail out to avoid hogging the CPU.
                break;
            }
        }
        
        // Report the interrupt via the semaphore.
        signal_sem(&m_event_sem);
    }
    
    // Post to the stopped semaphore to let stop() know that
    // we're terminating and may be joined.
    sem_post(&m_stopped_sem);
}
