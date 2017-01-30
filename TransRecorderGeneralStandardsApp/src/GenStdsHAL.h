/* This file is part of the General Standards Digitizer Driver.
 * It is subject to the license terms in the LICENSE.txt file found in the
 * top-level directory of this distribution and at
 * https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
 * of the General Standards Digitizer Driver, including this file, may be copied,
 * modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#ifndef GENSTDS_HAL_H
#define GENSTDS_HAL_H

#include <stdint.h>
#include <stddef.h>

class GenStdsHAL
{
public:
    // TYPE DEFINITIONS
    
    // Represents success or failure of a request.
    enum GenStatus {failed=0, success=1, timeout=2};
    
    // Represents the result of a get request.
    template <class ResultValueType>
    struct GetResult {
        // Dummy constructor, does not initialize anything.
        inline GetResult()
        {
        }
        
        // Construct using the given status and zero value.
        inline GetResult(GenStatus status_arg)
        : status(status_arg), value((ResultValueType)0)
        {
        }
        
        // Construct using success status and the given value.
        inline GetResult(ResultValueType value_arg)
        : status(success), value(value_arg)
        {
        }
        
        GenStatus status;
        ResultValueType value;
    };
    
    // Enums for various things (configuration, states, operation results).
    enum OnOffState {Off, On};
    enum InputMode {SingleEnded, PseudoDifferential, Differential, ZeroTest, ReferenceTest};
    enum VoltageRange {R_Bi_2_5, R_Uni_5, R_Bi_5, R_Uni_10, R_Bi_10};
    enum ChannelActiveRange {ChSingle, Ch2, Ch4, Ch8, Ch16, Ch32, Ch64, ChRange};
    enum ChannelType {ChTypeFirst, ChTypeLast, ChTypeSingle};
    enum RateGen {GeneratorA, GeneratorB};
    enum RateBSource {RateBSrcMaster, RateBSrcRateA};
    enum SampleClockSource {ClkSrcRateA, ClkSrcRateB, ClkSrcExternal, ClkSrcSoftware};
    enum AuxLine {Aux0, Aux1, Aux2, Aux3};
    enum AuxLineMode {AuxModeDisable, AuxModeInput, AuxModeOutput};
    enum BurstTriggerSource {DisableBurst, RateBGen, ExtInputSync, BcrOutSync};
    enum RateGenDivLimit {Low, High};
    enum BufferOverflow {NoOverflow, Overflow};
    enum BurstStatus {Idle, Active};
    enum IoMode {IoModeDMA, IoModeDMDMA, IoModePIO};
    enum Interrupt {Irq0None, Irq0BurstStart, Irq1None, Irq1BufThrL2H};
    enum WaitResult {WaitDone, WaitCancelled};
    
    // Special values.
    static uint32_t const ReadTimeoutInfinite = (uint32_t)-1;
    
    // Functor type for wait-start callbacks.
    class WaitCondition {
    public:
        virtual bool isMet () = 0;
    };
    
    // INITIALIZATION AND CLEANUP
    
    // Destructor allowing polymorphic delete.
    virtual ~GenStdsHAL() = 0;
    
    // Open device (fails if opened already).
    virtual GenStatus Open() = 0;
    
    // Close device (may be closed already).
    virtual void Close() = 0;
    
    
    // DEVICE CONFIGURATION
    
    virtual GenStatus SetInputMode(InputMode arg) = 0;
    virtual GetResult<InputMode> GetInputMode() = 0;
    
    virtual GenStatus SetVoltageRange(VoltageRange arg) = 0;
    virtual GetResult<VoltageRange> GetVoltageRange() = 0;
    
    virtual GenStatus SetChannelActiveRange(ChannelActiveRange arg) = 0;
    virtual GetResult<ChannelActiveRange> GetChannelActiveRange() = 0;
    
    virtual GenStatus SetChannel(ChannelType channel_type, int32_t channel_number) = 0;
    virtual GetResult<int32_t> GetChannel(ChannelType channel_type) = 0;
    
    virtual GenStatus SetRateGenEnabled(RateGen gen, OnOffState state) = 0;
    virtual GetResult<OnOffState> GetRateGenEnabled(RateGen gen) = 0;
    
    virtual GenStatus SetRateGenDivisor(RateGen gen, int32_t value) = 0;
    virtual GetResult<int32_t> GetRateGenDivisor(RateGen gen) = 0;
    
    virtual GenStatus SetRateGenBSource(RateBSource source) = 0;
    virtual GetResult<RateBSource> GetRateGenBSource() = 0;
  
    virtual GenStatus SetSampleClockSource(SampleClockSource source) = 0; 
    virtual GetResult<SampleClockSource> GetSampleClockSource() = 0;
    
    virtual GenStatus SetExternalSync(OnOffState arg) = 0;
    virtual GetResult<OnOffState> GetExternalSync() = 0;
   
    virtual GenStatus SetAuxLineMode(AuxLine line, AuxLineMode mode) = 0;
    virtual GetResult<AuxLineMode> GetAuxLineMode(AuxLine line) = 0;        

    virtual GenStatus SetAuxInvertInMode(OnOffState arg) = 0;
    virtual GetResult<OnOffState> GetAuxInvertInMode() = 0;
    
    virtual GenStatus SetAuxInvertOutMode(OnOffState arg) = 0;
    virtual GetResult<OnOffState> GetAuxInvertOutMode() = 0;
    
    virtual GenStatus SetAuxNoiseSuppression(OnOffState arg) = 0;
    virtual GetResult<OnOffState> GetAuxNoiseSuppression() = 0;    
    
    virtual GenStatus SetBurstTriggerSource(BurstTriggerSource arg) = 0;
    virtual GetResult<BurstTriggerSource> GetBurstTriggerSource() = 0;

    virtual GenStatus SetBurstSize(int32_t size) = 0;
    virtual GetResult<int32_t> GetBurstSize() = 0;
    
    virtual GenStatus SetThresholdLevel(int32_t level) = 0;
    virtual GetResult<int32_t> GetThresholdLevel() = 0;
    
    // DEVICE REQUESTS
    
    virtual GenStatus SendSoftwareTrigger() = 0;
    
    virtual GenStatus ClearBufferOverflowFlag() = 0;
    
    // NOTE: blocking
    virtual GenStatus ClearFifoBuffer() = 0;
    
    // NOTE: blocking
    virtual GenStatus Initialize() = 0;
    
    // NOTE: blocking
    virtual GenStatus Autocalibrate() = 0;
    
    
    // DEVICE INFORMATION
    
    virtual GetResult<int32_t> GetFirmwareRevision() = 0;
    virtual GetResult<int32_t> GetMaximumInputChannels() = 0;
    virtual GetResult<int32_t> GetMaxSampleRate() = 0;
    virtual GetResult<int32_t> GetBufferSize() = 0;
    virtual GetResult<int32_t> GetDeviceType() = 0;
    virtual GetResult<int32_t> GetMasterClockFrequency() = 0;
    virtual GetResult<int32_t> GetRateGenDivisorLimit(RateGenDivLimit limit) = 0;
    
    // DEVICE STATES
    
    virtual GetResult<BufferOverflow> GetBufferOverflow() = 0;
    virtual GetResult<BurstStatus> GetBurstStatus() = 0;
    virtual GetResult<int32_t> GetNumberOfBufferSamples() = 0;
    
    // DATA ACQUISITION
    
    virtual GenStatus SetIoMode(IoMode io_mode) = 0;
    virtual GenStatus SetReadTimeout(uint32_t timeout_sec) = 0;
    virtual GenStatus SetReadOverflowCheck(OnOffState enabled) = 0;
    virtual GetResult<size_t> ReadFIFOBuffer(void *buffer, size_t num_bytes) = 0;
    virtual GetResult<bool> AbortRead() = 0;
    
    
    // WAITING FOR INTERRUPTS
    
    virtual GenStatus ConfigureInterrupt(Interrupt intr) = 0;
    virtual GetResult<WaitResult> WaitForInterrupt(Interrupt intr, WaitCondition &cond) = 0;
    virtual void CancelWait(Interrupt intr) = 0;
    virtual void ResetWait(Interrupt intr) = 0;
};

inline GenStdsHAL::~GenStdsHAL()
{
}

#endif
