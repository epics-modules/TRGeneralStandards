/* This file is part of the General Standards Digitizer Driver.
 * It is subject to the license terms in the LICENSE.txt file found in the
 * top-level directory of this distribution and at
 * https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
 * of the General Standards Digitizer Driver, including this file, may be copied,
 * modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#include <limits.h>

#include <cmath>
#include <string>
#include <algorithm>
#include <new>

#include <epicsExport.h>
#include <epicsGuard.h>
#include <epicsThread.h>
#include <errlog.h>
#include <iocsh.h>

#include "TRGeneralStandards.h"
#include "TRChannelDataSubmit.h"

// Factory function for the HAL comes from the HAL implementation.
extern GenStdsHAL * GetGenStdsHAL(char const *device_node, int waiter_priority);


// Specific conditions for accepting the relevant hardware interrupts

bool TRGeneralStandards::BufferThresholdCondition::isMet()
{
    GenStdsHAL::GetResult<int32_t> hal_res = m_cond_hal->GetNumberOfBufferSamples();
    if (hal_res.status != GenStdsHAL::success) {
        return false;
    }
    return (hal_res.value > m_threshold);
}

bool TRGeneralStandards::BurstStartCondition::isMet()
{
    GenStdsHAL::GetResult<GenStdsHAL::BurstStatus> hal_stat = m_cond_hal->GetBurstStatus();
    GenStdsHAL::GetResult<int32_t> hal_num = m_cond_hal->GetNumberOfBufferSamples();
    if (hal_stat.status != GenStdsHAL::success || hal_num.status != GenStdsHAL::success) {
        return false;
    }
    return (hal_stat.value == GenStdsHAL::Active || hal_num.value > 0);
}


// Methods of the main class

TRGeneralStandards::TRGeneralStandards(
    char const *port_name, char const *device_node,
    int read_thread_prio_epics, int read_thread_stack_size,
    int wait_thread_prio_pthread, int max_ad_buffers, size_t max_ad_memory)
:
    TRBaseDriver(TRBaseConfig()
        .set(&TRBaseConfig::port_name, std::string(port_name))
        .set(&TRBaseConfig::num_channels, maxChannelNum)
        .set(&TRBaseConfig::num_asyn_params, (int) REG_PARAM_NUMBER)
        .set(&TRBaseConfig::num_config_params, CONF_PARAM_NUMBER)
        .set(&TRBaseConfig::read_thread_prio, read_thread_prio_epics)
        .set(&TRBaseConfig::read_thread_stack_size, read_thread_stack_size)
        .set(&TRBaseConfig::max_ad_buffers, max_ad_buffers)
        .set(&TRBaseConfig::max_ad_memory, max_ad_memory)
    ),
    m_worker((std::string("TRwork:") + port_name)),
    m_interrupt_read_req(&m_worker, this, InterruptReadHR),
    m_interrupting_read(false),
    m_clock_calc_req(&m_worker, this, ClockCalcHR),
    m_clock_calc_state(ClockCalcIdle),
    m_previous_requested_rate(NAN),
    m_read_loop_is_waiting_for_worker(false),
    m_initialize_req(&m_worker, this, InitializeHR),
    m_init_req_running(false),
    m_opened(false),
    m_accepting_sw_triggers(false)
{
    // Initialization of of variables contained in member structs.
    m_bp.burst_id = 0;

    // Configuration params.
    // The default values of parameters with a discrete set of possible values
    // are set to the largest value of their corresponding enum in GenStdsHAL.h
    // plus one which represents a "not-available" value. Epics records
    // exposing armed values are encouraged to follow this convention. The
    // effective parts of parameters which represent actual numeric values are
    // externally visible as doubles and their default values are NaN.
    initConfigParam(m_param_burst_trigger_source,   "BURST_TRIGGER_SOURCE", (int) TriggerNotAvailable);     
    initConfigParam(m_param_input_mode,             "INPUT_MODE",               GenStdsHAL::ReferenceTest + 1);
    initConfigParam(m_param_voltage_range,          "VOLTAGE_RANGE",            GenStdsHAL::R_Bi_10 + 1);
    initConfigParam(m_param_channel_active_range,   "CHANNEL_ACTIVE_RANGE",     GenStdsHAL::ChRange + 1);
    initConfigParam(m_param_channel_range_first,    "CHANNEL_RANGE_FIRST",      (double)NAN);
    initConfigParam(m_param_channel_range_last,     "CHANNEL_RANGE_LAST",       (double)NAN);
    initConfigParam(m_param_channel_single,         "CHANNEL_SINGLE",           (double)NAN);
    initConfigParam(m_param_read_burst_start_ts_en, "READ_BURST_START_TS_EN",   GenStdsHAL::On + 1);
    initConfigParam(m_param_read_dmdma_enabled,     "READ_DMDMA_ENABLED",       GenStdsHAL::On + 1);

    // Internal params.
    // These cannot be set by the user but only by the driver after a
    // successful ClockCalcRequest.
    initInternalParam(m_param_div_a_value,          "RATE_A_DIVISION_VALUE",    0);
    initInternalParam(m_param_div_b_value,          "RATE_B_DIVISION_VALUE",    0);
    
    // NOTE: All initConfigParam/initInternalParam must be before all createParam
    // so that the parameter index comparison in writeInt32/readInt32 works as expected.

    // Read-only or write-only params
    // Not writing to the parameters has to be enforced in the epics code.
    createParam("SEND_SW_TRIGGER",                  asynParamInt32,     &m_reg_params[SEND_SW_TRIGGER]);            // w/o
    createParam("REFRESH_STATUS_INFO",              asynParamInt32,     &m_reg_params[REFRESH_STATUS_INFO]);        // w/o
    
    // Device information
    createParam("FIRMWARE_REVISION",                asynParamInt32,     &m_reg_params[FIRMWARE_REVISION]);          // r/o
    createParam("MAX_INPUT_CHANNELS",               asynParamInt32,     &m_reg_params[MAX_INPUT_CHANNELS]);         // r/o
    createParam("MASTER_CLOCK_FREQUENCY",           asynParamInt32,     &m_reg_params[MASTER_CLOCK_FREQUENCY]);     // r/o
    createParam("MAX_SAMPLE_RATE",                  asynParamInt32,     &m_reg_params[MAX_SAMPLE_RATE]);            // r/o
    createParam("BUFFER_SIZE",                      asynParamInt32,     &m_reg_params[BUFFER_SIZE]);                // r/o
    createParam("DEVICE_TYPE",                      asynParamInt32,     &m_reg_params[DEVICE_TYPE]);                // r/o
    createParam("MIN_DIVISOR",                      asynParamInt32,     &m_reg_params[MIN_DIVISOR]);                // r/o
    createParam("MAX_DIVISOR",                      asynParamInt32,     &m_reg_params[MAX_DIVISOR]);                // r/o

    // Device states
    createParam("BUFFER_OVERFLOW_STATUS",           asynParamInt32,     &m_reg_params[BUFFER_OVERFLOW_STATUS]);     // r/o
    createParam("BURST_STATUS",                     asynParamInt32,     &m_reg_params[BURST_STATUS]);               // r/o
    createParam("BUFFER_SAMPLES_NUMBER",            asynParamInt32,     &m_reg_params[BUFFER_SAMPLES_NUMBER]);      // r/o

    // Parameters that may be sent directly to hardware at any time
    createParam("AUX_LINE_0_MODE",                  asynParamInt32,     &m_reg_params[AUX_LINE_0_MODE]);
    createParam("AUX_LINE_1_MODE",                  asynParamInt32,     &m_reg_params[AUX_LINE_1_MODE]);
    createParam("AUX_LINE_2_MODE",                  asynParamInt32,     &m_reg_params[AUX_LINE_2_MODE]);
    createParam("AUX_LINE_3_MODE",                  asynParamInt32,     &m_reg_params[AUX_LINE_3_MODE]);
    createParam("AUX_INVERT_IN",                    asynParamInt32,     &m_reg_params[AUX_INVERT_IN]);
    createParam("AUX_INVERT_OUT",                   asynParamInt32,     &m_reg_params[AUX_INVERT_OUT]);
    createParam("AUX_NOISE_SUPPRESSION",            asynParamInt32,     &m_reg_params[AUX_NOISE_SUPPRESSION]);
    createParam("ZERO_VOLTAGE_OFFSET",              asynParamFloat64,   &m_reg_params[ZERO_VOLTAGE_OFFSET]);
    createParam("GET_RAW_DATA",                     asynParamInt32,     &m_reg_params[GET_RAW_DATA]);

    // Readbacks
    createParam("INPUT_MODE_READBACK",              asynParamInt32,     &m_reg_params[INPUT_MODE_READBACK]);
    createParam("VOLTAGE_RANGE_READBACK",           asynParamInt32,     &m_reg_params[VOLTAGE_RANGE_READBACK]);
    createParam("CHANNEL_ACTIVE_RANGE_READBACK",    asynParamInt32,     &m_reg_params[CHANNEL_ACTIVE_RANGE_READBACK]);
    createParam("CHANNEL_RANGE_FIRST_READBACK",     asynParamInt32,     &m_reg_params[CHANNEL_RANGE_FIRST_READBACK]);
    createParam("CHANNEL_RANGE_LAST_READBACK",      asynParamInt32,     &m_reg_params[CHANNEL_RANGE_LAST_READBACK]);
    createParam("CHANNEL_SINGLE_READBACK",          asynParamInt32,     &m_reg_params[CHANNEL_SINGLE_READBACK]);
    createParam("CLOCK_SOURCE_READBACK",            asynParamInt32,     &m_reg_params[CLOCK_SOURCE_READBACK]);

    // Other params
    createParam("INITIALIZE",                       asynParamInt32,     &m_reg_params[INITIALIZE]);
    createParam("EXTERNAL_CLOCK_RATE",              asynParamFloat64,   &m_reg_params[EXTERNAL_CLOCK_RATE]);

    // Initial parameter values.
    setIntegerParam(m_reg_params[INITIALIZE], HelperRequestFailed);

    // Start worker thread
    m_worker.start();

    // Create the GenStds HAL instance
    m_hal = GetGenStdsHAL(device_node, wait_thread_prio_pthread);
}

bool TRGeneralStandards::Open()
{
    assert(!m_opened);
    
    if (!m_hal->Open()) {
        return false;
    }

    // Initialize device info.
    // If getting any of it fails the HAL prints an error message and we
    // exit.
    GenericGetResult result;
    bool success = true;

    epicsGuard<asynPortDriver> guard(*this);

    result = m_hal->GetFirmwareRevision();    
    success = success && (result.status == GenStdsHAL::success);
    setIntegerParam(m_reg_params[FIRMWARE_REVISION], result.value);

    result = m_hal->GetMaximumInputChannels();    
    success = success && (result.status == GenStdsHAL::success);
    setIntegerParam(m_reg_params[MAX_INPUT_CHANNELS], result.value);
    m_hw.max_channel_num = result.value;

    result = m_hal->GetMasterClockFrequency();    
    success = success && (result.status == GenStdsHAL::success);
    setIntegerParam(m_reg_params[MASTER_CLOCK_FREQUENCY], result.value);
    m_hw.master_freq = result.value;

    result = m_hal->GetMaxSampleRate();    
    success = success && (result.status == GenStdsHAL::success);
    setIntegerParam(m_reg_params[MAX_SAMPLE_RATE], result.value);
    m_hw.max_sample_rate = result.value;

    result = m_hal->GetBufferSize();    
    success = success && (result.status == GenStdsHAL::success);
    setIntegerParam(m_reg_params[BUFFER_SIZE], result.value);
    m_hw.buffer_size = result.value;

    result = m_hal->GetDeviceType();    
    success = success && (result.status == GenStdsHAL::success);
    setIntegerParam(m_reg_params[DEVICE_TYPE], result.value);

    result = m_hal->GetRateGenDivisorLimit(GenStdsHAL::Low);    
    success = success && (result.status == GenStdsHAL::success);
    setIntegerParam(m_reg_params[MIN_DIVISOR], result.value);
    m_hw.min_div = result.value;

    result = m_hal->GetRateGenDivisorLimit(GenStdsHAL::High);    
    success = success && (result.status == GenStdsHAL::success);
    setIntegerParam(m_reg_params[MAX_DIVISOR], result.value);
    m_hw.max_div = result.value;

    callParamCallbacks();

    if (!success) {
        return false;
    }

    // Sanity check device information.
    if (!(m_hw.master_freq > 1000000.0 && m_hw.min_div > 0 && m_hw.max_div > 0 &&
          m_hw.min_div <= 65535 && m_hw.max_div <= 65535 && m_hw.min_div < m_hw.max_div))
    {
        errlogSevPrintf(errlogMajor, "genstds device initialization error: insane device information.\n");
        return false;
    }

    m_opened = true;
    
    return true;
}

asynStatus TRGeneralStandards::writeInt32(asynUser *pasynUser, int32_t value)
{
    int reason = pasynUser->reason;
    GenStdsHAL::GenStatus result;

    // If the parameter is not ours or a config parameter, delegate to base class
    if (reason < m_reg_params[0]) {
        return TRBaseDriver::writeInt32(pasynUser, value);
    }
    
    // Check that the device has been opened.
    if (!m_opened) {
        return asynError;
    }
    
    // Handle based on which parameter it is.
    if (reason == m_reg_params[AUX_LINE_0_MODE]) {
        result = m_hal->SetAuxLineMode(GenStdsHAL::Aux0, (GenStdsHAL::AuxLineMode)value); 
    }
    else if (reason == m_reg_params[AUX_LINE_1_MODE]) {
        result = m_hal->SetAuxLineMode(GenStdsHAL::Aux1, (GenStdsHAL::AuxLineMode)value); 
    }
    else if (reason == m_reg_params[AUX_LINE_2_MODE]) {
        result = m_hal->SetAuxLineMode(GenStdsHAL::Aux2, (GenStdsHAL::AuxLineMode)value); 
    }
    else if (reason == m_reg_params[AUX_LINE_3_MODE]) {
        result = m_hal->SetAuxLineMode(GenStdsHAL::Aux3, (GenStdsHAL::AuxLineMode)value); 
    }
    else if (reason == m_reg_params[AUX_INVERT_IN]) {
        result = m_hal->SetAuxInvertInMode((GenStdsHAL::OnOffState)value);
    }
    else if (reason == m_reg_params[AUX_INVERT_OUT]) {
        result = m_hal->SetAuxInvertOutMode((GenStdsHAL::OnOffState)value);
    }
    else if (reason == m_reg_params[AUX_NOISE_SUPPRESSION]) {
        result = m_hal->SetAuxNoiseSuppression((GenStdsHAL::OnOffState)value);
    }
    else if (reason == m_reg_params[SEND_SW_TRIGGER]) {
        result = handleSendSoftwareTrigger();
    }
    else if (reason == m_reg_params[REFRESH_STATUS_INFO]) {
        return handleRefreshStatusInfo();
    }
    else if (reason == m_reg_params[INITIALIZE]) {
        return handleInitializeRequest();
    }
    else {
        // Simple parameter, delegate to asynPortDriver.
        return asynPortDriver::writeInt32(pasynUser, value);
    }
    
    return (result == GenStdsHAL::success) ? asynSuccess : asynError;
}

asynStatus TRGeneralStandards::readInt32(asynUser *pasynUser, int32_t *value)
{
    int reason = pasynUser->reason;
    GenericGetResult result;

    // If the parameter is not ours or a config parameter, delegate to base class
    if (reason < m_reg_params[0]) {
        return TRBaseDriver::readInt32(pasynUser, value);
    }

    // Check that the device has been opened.
    if (!m_opened) {
        return asynError;
    }
    
    // Handle based on which parameter it is.
    if (reason == m_reg_params[AUX_LINE_0_MODE]) {
        result = m_hal->GetAuxLineMode(GenStdsHAL::Aux0);
    }
    else if (reason == m_reg_params[AUX_LINE_1_MODE]) {
        result = m_hal->GetAuxLineMode(GenStdsHAL::Aux1); 
    }
    else if (reason == m_reg_params[AUX_LINE_2_MODE]) {
        result = m_hal->GetAuxLineMode(GenStdsHAL::Aux2); 
    }
    else if (reason == m_reg_params[AUX_LINE_3_MODE]) {
        result = m_hal->GetAuxLineMode(GenStdsHAL::Aux3);
    }
    else if (reason == m_reg_params[AUX_INVERT_IN]) {
        result = m_hal->GetAuxInvertInMode();
    }
    else if (reason == m_reg_params[AUX_INVERT_OUT]) {
        result = m_hal->GetAuxInvertOutMode();
    }
    else if (reason == m_reg_params[AUX_NOISE_SUPPRESSION]) {
        result = m_hal->GetAuxNoiseSuppression();
    }
    else if (reason == m_reg_params[INPUT_MODE_READBACK]) {
        result = m_hal->GetInputMode();
    }
    else if (reason == m_reg_params[VOLTAGE_RANGE_READBACK]) {
        result = m_hal->GetVoltageRange();
    }
    else if (reason == m_reg_params[CHANNEL_ACTIVE_RANGE_READBACK]) {
        result = m_hal->GetChannelActiveRange();
    }
    else if (reason == m_reg_params[CHANNEL_RANGE_FIRST_READBACK]) {
        result = m_hal->GetChannel(GenStdsHAL::ChTypeFirst);
    }
    else if (reason == m_reg_params[CHANNEL_RANGE_LAST_READBACK]) {
        result = m_hal->GetChannel(GenStdsHAL::ChTypeLast);
    }
    else if (reason == m_reg_params[CHANNEL_SINGLE_READBACK]) {
        result = m_hal->GetChannel(GenStdsHAL::ChTypeSingle);
    }
    else if (reason == m_reg_params[CLOCK_SOURCE_READBACK]) {
        result = m_hal->GetSampleClockSource();
    }
    else {
        // Simple parameter, delegate to asynPortDriver.
        return asynPortDriver::readInt32(pasynUser, value);
    }
    
    // On success, return the value and update the value in the parameter cache.
    asynStatus status;
    if (result.status == GenStdsHAL::success) {
        status = asynSuccess;
        *value = result.value;
        setIntegerParam(reason, result.value);
    } else {
        status = asynError;
    }
    
    // Set the parameter status based on the outcome of the read.
    setParamStatus(reason, status);
    
    // Do the parameter callbacks so that any I/O Intr records are updated.
    callParamCallbacks();
    
    return status;
}

asynStatus TRGeneralStandards::handleInitializeRequest()
{
    if (isDisarmed()) {
        if (!m_init_req_running) {
            m_init_req_running = true;
            setIntegerParam(m_reg_params[INITIALIZE], HelperRequestRunning);
            callParamCallbacks();
            m_initialize_req.start();
        }
    } else {
        // Set to a dummy value first and then to failed status to ensure
        // asyn triggers an I/O Intr.
        setIntegerParam(m_reg_params[INITIALIZE], HelperRequestRunning);
        setIntegerParam(m_reg_params[INITIALIZE], HelperRequestFailed);
        callParamCallbacks();
    }
    
    return asynSuccess;
}

asynStatus TRGeneralStandards::handleRefreshStatusInfo()
{
    GenericGetResult result;
    bool success = true;

    result = m_hal->GetBufferOverflow();
    success = success && (result.status == GenStdsHAL::success);
    setIntegerParam(m_reg_params[BUFFER_OVERFLOW_STATUS], result.value);

    result = m_hal->GetBurstStatus(); 
    success = success && (result.status == GenStdsHAL::success);
    setIntegerParam(m_reg_params[BURST_STATUS], result.value);

    result = m_hal->GetNumberOfBufferSamples(); 
    success = success && (result.status == GenStdsHAL::success);
    setIntegerParam(m_reg_params[BUFFER_SAMPLES_NUMBER], result.value);

    callParamCallbacks();

    return success ? asynSuccess : asynError;
}

GenStdsHAL::GenStatus TRGeneralStandards::handleSendSoftwareTrigger()
{
    // Ignore software triggers while we're not live so that configuration code
    // can use BurstTriggerSource=SoftwareTriger configuration as a bursting-enabled
    // setting which does not start any bursts.
    return m_accepting_sw_triggers ? m_hal->SendSoftwareTrigger() : GenStdsHAL::success;
}

void TRGeneralStandards::requestedSampleRateChanged()
{
    // Get the requested sample rate from the framework.
    double desiredSampleRate = getRequestedSampleRate();

    // EDM tends to send out a lot of requests so only react to those where the
    // requested rate has actually changed.
    if (desiredSampleRate == m_previous_requested_rate) {
        return;
    }
    m_previous_requested_rate = desiredSampleRate;

    // Schedule the clock calculation or, if running, set the dirty flag so
    // that it restarts once finished.
    switch (m_clock_calc_state){
        case ClockCalcIdle: {
            m_clock_calc_state = ClockCalcQueued;
            m_clock_calc_req.start();
        } break;
        case ClockCalcRunning: {
            m_clock_calc_state = ClockCalcDirty;
        } break;
        case ClockCalcQueued:
        case ClockCalcDirty: {
            // do nothing
        } break;
        default:
            assert(false);
    }
}

void TRGeneralStandards::runWorkerThreadTask(int hrt)
{
    if (hrt == ClockCalcHR) {
        calculateClockFromWorker();
    }
    else if (hrt == InterruptReadHR) {
        interruptReadingFromWorker();
    }
    else if (hrt == InitializeHR) {
        initializeFromWorker();
    }
    else {
        assert(false);
    }
}

void TRGeneralStandards::initializeFromWorker()
{
    assert(m_init_req_running);
    
    GenStdsHAL::GenStatus result = m_hal->Initialize();

    epicsGuard<asynPortDriver> guard(*this);

    m_init_req_running = false;
    
    // Report the result using the parameter.
    HelperRequestState status = (result == GenStdsHAL::success) ? HelperRequestSucceeded
                    : HelperRequestFailed;
    setIntegerParam(m_reg_params[INITIALIZE], status);
    callParamCallbacks();

    // Make sure arming waits until this request is done.
    if (m_read_loop_is_waiting_for_worker) {
        m_worker_done.signal();
    }
}

void TRGeneralStandards::calculateClockFromWorker()
{
    double requested_sample_rate;
    
    {
        epicsGuard<asynPortDriver> guard(*this);

        assert(m_clock_calc_state == ClockCalcQueued);
        m_clock_calc_state = ClockCalcRunning;
        requested_sample_rate = getRequestedSampleRate();
    }
    
    // Calculation results.
    int div_a = 0;
    int div_b = 0;
    double achievable_rate = NAN;
    
    // Check the desired rate is sensible so that the helper thread doesn't
    // have to.
    // Note that 0 is a legal value that results in the lowest achievable rate
    // being used. -1 signifies that an external clock will be used. In that
    // case the user interface should allow for setting an estimate of the
    // external rate, to be written to the EXTERNAL_CLOCK_RATE param.
    if (requested_sample_rate < 0 && requested_sample_rate != -1.){
        errlogSevPrintf(errlogMajor, "Invalid sample rate requested.");
        
        // Allow checkSettings to see there was an error here.
        achievable_rate = -INFINITY;
    } else {
        // If using internal clock (master frequency + dividers), run the lengthy
        // calculation to obtain optimal divider values.
        bool external_clock = (requested_sample_rate == -1.);
        if (!external_clock) {
            calculateRateGenDivisors(requested_sample_rate, &div_a, &div_b);
            achievable_rate = calculateAchievableSampleRate(div_a, div_b);
        }
    }

    epicsGuard<asynPortDriver> guard(*this);

    updateClockCalcParams(div_a, div_b, achievable_rate);
    
    if (m_clock_calc_state == ClockCalcDirty) {
        // Different sample rate requested during calculation, restart.
        m_clock_calc_state = ClockCalcQueued;
        m_clock_calc_req.start();
    } else {
        assert(m_clock_calc_state == ClockCalcRunning);
        m_clock_calc_state = ClockCalcIdle;

        // Make sure arming waits until this request is done
        if (m_read_loop_is_waiting_for_worker) {
            m_worker_done.signal();
        }
    }
}

void TRGeneralStandards::updateClockCalcParams(int div_a, int div_b, double achievable_rate)
{
    // Should be called locked.
    
    m_param_div_a_value.setDesired(div_a);
    m_param_div_b_value.setDesired(div_b);
    setAchievableSampleRate(achievable_rate);
    
    // setAchievableSampleRate called callParamCallbacks.
}

// Calculate best divisor aproximations.
void TRGeneralStandards::calculateRateGenDivisors(double rate, int* best_div_a, int* best_div_b)
{
    int local_best_div_a;
    int local_best_div_b;
    double min_div_dbl = m_hw.min_div;
    double max_div_dbl = m_hw.max_div;
    
    // If the desired rate is >= the maximum achievable rate (using just one
    // divider at its minimum divisor value), then return that.
    if (rate >= m_hw.master_freq/min_div_dbl) {
        local_best_div_a = m_hw.min_div;
        local_best_div_b = 0;
    }
    // If the desired rate is <= the minimum achievablerate (using both dividers
    // at their maximum divisor values), then return that.
    else if (rate <= m_hw.master_freq/(max_div_dbl*max_div_dbl)) {
        local_best_div_a = m_hw.max_div;
        local_best_div_b = m_hw.max_div;
    }
    // If the rate is >= least the minimum rate achievable with just one divider,
    // calculate the best divisor.
    else if (rate >= m_hw.master_freq/max_div_dbl) {
        // Calculate the real-valued divisor that would give exactly the disired rate.
        double calc_div = m_hw.master_freq/rate;

        // In reality we need to pick one of the two closest integer divisors.
        int32_t div_low = (int32_t)calc_div;
        int32_t div_high = div_low + 1;
        
        // Calculate the frequency errors for each of these and choose the
        // divisor that gives the least error.
        double err_low = std::abs(m_hw.master_freq/div_low - rate);
        double err_high = std::abs(m_hw.master_freq/div_high - rate);
        int32_t div_a = (err_low <= err_high) ? div_low : div_high;
        
        // This divisor must be in range due to the range of rate here, even
        // roundoff errors will not be a problem since we chose the best divisor.
        assert(div_a >= m_hw.min_div && div_a <= m_hw.max_div);
        
        local_best_div_a = div_a;
        local_best_div_b = 0;
    }
    // Otherwise we need to use both dividers.
    else {
        // We will try different A dividers and calculate the best B
        // divider for each, tracking the best set of dividers and the
        // frequency error.
        double best_error = INFINITY;
        
        // Zeroing these is redundant but silences a compiler warning.
        local_best_div_a = 0;
        local_best_div_b = 0;
        
        for (int32_t div_a = m_hw.min_div; div_a <= m_hw.max_div; div_a++) {
            // Calculate the real-valued B divisor that would give exactly the disired rate.
            double calc_div = m_hw.master_freq/(div_a*rate);
            
            // Calculate the two closest integer divisors. Here we must be
            // careful that they are not out of range.
            int32_t div_low = std::max(m_hw.min_div, std::min(m_hw.max_div, (int32_t)calc_div));
            int32_t div_high = std::min(m_hw.max_div, div_low + 1);
            
            // Calculate the frequency errors.
            double err_low = std::abs(m_hw.master_freq/((double)div_a*div_low) - rate);
            double err_high = std::abs(m_hw.master_freq/((double)div_a*div_high) - rate);
            
            // Choose the divisor that gives the least error.
            int32_t div_b;
            double err;
            if (err_low <= err_high) {
                div_b = div_low;
                err = err_low;
            } else {
                div_b = div_high;
                err = err_high;
            }
            
            // Update best.
            if (err < best_error) {
                best_error = err;
                local_best_div_a = div_a;
                local_best_div_b = div_b;
            }
        }
    }
    
    *best_div_a = local_best_div_a;
    *best_div_b = local_best_div_b;
}

double TRGeneralStandards::calculateAchievableSampleRate(int div_a, int div_b)
{
    return (double)m_hw.master_freq / ((double)div_a * std::max(1, div_b));
}

bool TRGeneralStandards::waitForPreconditions ()
{
    // Wait for frequency divider calculation and initialization requests
    // In this particular implementation, calling waitForPreconditions can't ever fail
    while (m_clock_calc_state != ClockCalcIdle || m_init_req_running) {
        m_read_loop_is_waiting_for_worker = true;
        unlock();
        m_worker_done.wait();
        lock();
    }
    m_read_loop_is_waiting_for_worker = false;
    
    return true;
}

bool TRGeneralStandards::checkSettings(TRArmInfo &arm_info)
{
    // Check if there was an error in calculateClockFromWorker.
    if (getAchievableSampleRateSnapshot() == -INFINITY) {
        errlogSevPrintf(errlogMajor, "genstds checkSettings error: there was an error in clock calculation");
        return false;
    }
    
    // Set the appropriate display sample rate with respect to clock source.
    m_using_external_clock = (getRequestedSampleRateSnapshot() == -1.);

    // Determine the sample rate for display.
    if (m_using_external_clock) {
        getDoubleParam(m_reg_params[EXTERNAL_CLOCK_RATE], &m_private_rate_for_display);
    } else {
        m_private_rate_for_display = getAchievableSampleRateSnapshot();
    }

    // Check passed parameter validity.
    if (!checkChannelSettings() || !checkClockSettings() || !checkSampleSizeSettings()) {
        // Error message printed by lower-level function.
        return false;
    }

    // Communicate display rate to the framework.
    arm_info.rate_for_display = m_private_rate_for_display;

    return true;
}

bool TRGeneralStandards::startAcquisition(bool had_overflow)
{
    if (!had_overflow) { // Executing as part of an arm request
        // Set voltage range for autocalibration
        int voltageRange = m_param_voltage_range.getSnapshot();
        if (m_hal->SetVoltageRange((GenStdsHAL::VoltageRange) voltageRange) != GenStdsHAL::success) {
            errlogSevPrintf(errlogMajor, "genstds startAcquisition error: "
                    "SetVoltageRange(%d) failed.", voltageRange);
            return false;
        }

        // Set A divisor value for autocalibration
        if (!setDivisorAForAutocalibrate()) {
            return false;
        }

        // Launch autocalibration
        if (m_hal->Autocalibrate() != GenStdsHAL::success) {
            errlogSevPrintf(errlogMajor, "genstds startAcquisition error: Autocalibrate failed.");
            return false;
        }

        // Calculate many variables, allocate data buffer.
        if (!calcBurstParams()){
            return false;
        }
        
        // Commit most of the configuration to hardware.
        if (!commitConfigParams()){
            return false;
        }
    }

    // The following executes always, both when arming and recovering from
    // overflow

    if (!commitBurstParams()) {
        return false;
    } 

    // From now on we accept software triggers.
    {
        epicsGuard<asynPortDriver> guard(*this);
        m_accepting_sw_triggers = true;
    }

    return true;
}

bool TRGeneralStandards::checkChannelSettings()
{
    int inputMode          = m_param_input_mode.getSnapshot();
    int channelActiveRange = m_param_channel_active_range.getSnapshot();
    int channelSingle      = m_param_channel_single.getSnapshot();
    int channelRangeFirst  = m_param_channel_range_first.getSnapshot();
    int channelRangeLast   = m_param_channel_range_last.getSnapshot();

    if (inputMode < GenStdsHAL::SingleEnded       || inputMode > GenStdsHAL::ReferenceTest ||
        channelActiveRange < GenStdsHAL::ChSingle || channelActiveRange > GenStdsHAL::ChRange)
    {
        errlogSevPrintf(errlogMajor, "genstds checkChannelSettings error: InputMode %d and/or "
                "ChannelActiveRange %d is invalid.", inputMode, channelActiveRange);
        return false;
    }

    switch (channelActiveRange) {
        case GenStdsHAL::ChSingle: {
            m_param_channel_range_first.setIrrelevant();
            m_param_channel_range_last.setIrrelevant();

            if (!(channelSingle >= 0 && channelSingle < m_hw.max_channel_num)) {
                errlogSevPrintf(errlogMajor, "genstds checkChannelSettings error: "
                    "Single channel %d is out of valid range.", channelSingle);
                return false;
            }
            if (inputMode == GenStdsHAL::PseudoDifferential && channelSingle == 0) {
                errlogSevPrintf(errlogMajor, "genstds checkChannelSettings error: "
                    "Channel 0 is not a valid channel for pseudo-differential mode.");
                return false;
            }
            if (inputMode == GenStdsHAL::Differential && (channelSingle % 2) == 1) {
                errlogSevPrintf(errlogMajor, "genstds checkChannelSettings error: "
                    "Only even single channels are allowed in differential mode, selected channel %d is odd.", channelSingle);
                return false;
            }
        } break;
        case GenStdsHAL::ChRange: {
            m_param_channel_single.setIrrelevant();
            
            if (!(channelRangeFirst >= 0 && channelRangeFirst < channelRangeLast && channelRangeLast < m_hw.max_channel_num)) {
                errlogSevPrintf(errlogMajor, "genstds checkChannelSettings error: "
                    "First channel in range (channel %d) and last channel in range (channel %d) "
                    "are incorrectly ordered or out of valid range.", channelRangeFirst, channelRangeLast);
                return false;
            }
            if (inputMode == GenStdsHAL::PseudoDifferential && channelRangeFirst == 0) {
                errlogSevPrintf(errlogMajor, "genstds checkChannelSettings error: "
                    "Channel 0 is not a valid channel for pseudo-differential mode.");
                return false;
            }
            if (inputMode == GenStdsHAL::Differential && !((channelRangeFirst % 2) == 0 && (channelRangeLast % 2) == 1)) {
                errlogSevPrintf(errlogMajor, "genstds checkChannelSettings error: "
                    "First channel in range must be even and last channel must be odd in differential mode, "
                    "current selected range is [%d, %d].", channelRangeFirst, channelRangeLast);
                return false;
            }
        } break;
        default: { // Predefined Range
            m_param_channel_single.setIrrelevant();
            m_param_channel_range_first.setIrrelevant();
            m_param_channel_range_last.setIrrelevant();
        } break;
    }

    return true;
}

bool TRGeneralStandards::checkClockSettings()
{
    int div_a = m_param_div_a_value.getSnapshot();
    int div_b = m_param_div_b_value.getSnapshot();
    int burstTriggerSource = m_param_burst_trigger_source.getSnapshot();

    if (m_using_external_clock && burstTriggerSource == ExternalTrigger) {
        errlogSevPrintf(errlogMajor, "genstds checkClockSettings error: Burst and clock sources both external.");
        return false;
    }

    if (!(div_a == 0 || (div_a >= m_hw.min_div && div_a <= m_hw.max_div))) {
        errlogSevPrintf(errlogMajor, "genstds checkClockSettings error: Divisor A is out of range.");
        return false;
    }
    
    if (!(div_b == 0 || (div_b >= m_hw.min_div && div_b <= m_hw.max_div))) {
        errlogSevPrintf(errlogMajor, "genstds checkClockSettings error: Divisor B is out of range.");
        return false;
    }
    
    if (!m_using_external_clock) {
        // Let's recalculate the rate instead of relying on getAchievableSampleRateSnapshot,
        // for more assurance that the check is done properly. This error is not supposed to
        // happen because the clock calculation should only return valid clock settings.
        double achievable_rate = calculateAchievableSampleRate(div_a, div_b);
        if (!(achievable_rate <= m_hw.max_sample_rate)) {
            errlogSevPrintf(errlogMajor, "genstds checkClockSettings error: Sample rate is too high.");
            return false;
        }
    }
    
    return true;
}

bool TRGeneralStandards::checkSampleSizeSettings()
{
    m_bp.all_channels   = calcNumDataChannels();
    int samplesPerBurst = getNumPostSamplesSnapshot();

    if (samplesPerBurst <= 0) {
        errlogSevPrintf(errlogMajor, "genstds checkSampleSizeSettings error: Burst size must be positive.");
        return false;
    }
    if (samplesPerBurst > (m_hw.buffer_size - 1) / m_bp.all_channels) {
        errlogSevPrintf(errlogMajor, "genstds checkSampleSizeSettings error: Burst size * number of channels too big to fit in buffer.");
        return false;
    }

    return true;
}

int TRGeneralStandards::calcNumDataChannels()
{
    int channelActiveRange = m_param_channel_active_range.getSnapshot();
    int channelRangeFirst  = m_param_channel_range_first.getSnapshot();
    int channelRangeLast   = m_param_channel_range_last.getSnapshot();
    int inputMode          = m_param_input_mode.getSnapshot();

    if (channelActiveRange == GenStdsHAL::ChSingle) {
        return 1;
    }

    int channelsInRange;
    if  (channelActiveRange == GenStdsHAL::ChRange) {
        channelsInRange = channelRangeLast - channelRangeFirst + 1;
    } else {
        channelsInRange = 1 << channelActiveRange;
    }

    if (inputMode == GenStdsHAL::Differential) {
        return channelsInRange / 2;
    } else {
        return channelsInRange;
    }
}

bool TRGeneralStandards::setDivisorAForAutocalibrate()
{
    int div_a = m_param_div_a_value.getSnapshot();
    int div_b = m_param_div_b_value.getSnapshot();

    double raw_autocal_a_div;

    if (m_using_external_clock){
        double aCand = m_hw.master_freq / m_private_rate_for_display;
        double floorCand = std::floor(aCand);
        double ceilCand  = std::ceil (aCand);
        double floorDiff = m_hw.master_freq/floorCand - m_private_rate_for_display;
        double ceilDiff  = m_private_rate_for_display - m_hw.master_freq/ceilCand;

        raw_autocal_a_div = (floorDiff <= ceilDiff) ? floorCand : ceilCand;
    } else {
        raw_autocal_a_div = (double)div_a * std::max(1, div_b);
    }

    int autocal_a_div = std::min((double)m_hw.max_div, std::max((double)m_hw.min_div, raw_autocal_a_div));
    
    if (m_hal->SetRateGenDivisor(GenStdsHAL::GeneratorA, autocal_a_div) != GenStdsHAL::success){
        errlogSevPrintf(errlogMajor, "genstds setDivisorAForAutocalibrate error: "
                " SetRateGenDivisor(GeneratorA, %d) failed.", autocal_a_div);
        return false;
    }

    return true;
}

bool TRGeneralStandards::commitConfigParams()
{
    GenStdsHAL::GenStatus result;

    // Low-level settings
    // If any of these fail, the underlying HAL might print its own error
    // messages. Since this is not required, we provide some additional error
    // diagnostics here.
    
    int inputMode = m_param_input_mode.getSnapshot();
    result = m_hal->SetInputMode((GenStdsHAL::InputMode) inputMode);       
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetInputMode(%d) failed.", inputMode);
        return false;
    }

    int channelActiveRange = m_param_channel_active_range.getSnapshot();
    result = m_hal->SetChannelActiveRange((GenStdsHAL::ChannelActiveRange) channelActiveRange);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetChannelActiveRange(%d) failed.", channelActiveRange);
        return false;
    }
    
    // The device sometimes has problems if the first/last channels
    // are at non-default values but we are arming without using the
    // channel range feature. To avoid problems, always configure all
    // the channel numbers (single, first, last). However, ignore any
    // errors setting the first/last channels if we are not using
    // channel range, because hardware may lack channel range support.
    
    int channel_single = 0;
    int channel_first = 0;
    int channel_last = 1;
    bool need_range = false;

    if (channelActiveRange == GenStdsHAL::ChSingle) {
        channel_single = m_param_channel_single.getSnapshot();
    }
    else if (channelActiveRange == GenStdsHAL::ChRange) {
        channel_first = m_param_channel_range_first.getSnapshot();
        channel_last = m_param_channel_range_last.getSnapshot();
        need_range = true;
    }
    
    result = m_hal->SetChannel(GenStdsHAL::ChTypeSingle, channel_single);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetChannel(single, %d) failed.", channel_single);
        return false;
    }
    
    result = m_hal->SetChannel(GenStdsHAL::ChTypeFirst, channel_first);
    if (result != GenStdsHAL::success && need_range) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetChannel(first, %d) failed.", channel_first);
        return false;
    }

    result = m_hal->SetChannel(GenStdsHAL::ChTypeLast, channel_last);
    if (result != GenStdsHAL::success && need_range) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetChannel(last, %d) failed.", channel_last);
        return false;
    }

    int voltageRange = m_param_voltage_range.getSnapshot();
    result = m_hal->SetVoltageRange((GenStdsHAL::VoltageRange) voltageRange);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetVoltageRange(%d) failed.", voltageRange);
        return false;
    }

    // Set burst source to software for safe continuation of arming
    result = m_hal->SetBurstTriggerSource(GenStdsHAL::BcrOutSync);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetBurstTriggerSource(BcrOutSync) failed.");
        return false;
    }


    // Clock settings

    int div_a = m_param_div_a_value.getSnapshot();
    int div_b = m_param_div_b_value.getSnapshot();
    
    // SampleClockSource
    GenStdsHAL::SampleClockSource sampleClockSource = m_using_external_clock ? GenStdsHAL::ClkSrcExternal :
        (div_b == 0) ? GenStdsHAL::ClkSrcRateA : GenStdsHAL::ClkSrcRateB;
    result = m_hal->SetSampleClockSource(sampleClockSource);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetSampleClockSource(%d) failed.",
                (int) sampleClockSource);
        return false;
    }

    // ExternalSync
    GenStdsHAL::OnOffState externalSync = m_using_external_clock ? GenStdsHAL::On : GenStdsHAL::Off;
    result = m_hal->SetExternalSync(externalSync);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetExternalSync(%d) failed.",
                (int) externalSync);
        return false;
    }
    
    // We will set divisors which are not needed to the DefaultDivisorValue.
    int set_div_a = (div_a == 0) ? DefaultDivisorValue : div_a;
    int set_div_b = (div_b == 0) ? DefaultDivisorValue : div_b;

    // Divisor A.
    result = m_hal->SetRateGenDivisor(GenStdsHAL::GeneratorA, set_div_a);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetRateGenDivisor(GeneratorA, %d) failed.", set_div_a);
        return false;
    }
    
    // Divisor B.
    result = m_hal->SetRateGenDivisor(GenStdsHAL::GeneratorB, set_div_b);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetRateGenDivisor(GeneratorB, %d) failed.", set_div_b);
        return false;
    }

    // RateGenBSource
    GenStdsHAL::RateBSource rateBSource = (div_b == 0) ? GenStdsHAL::RateBSrcMaster : GenStdsHAL::RateBSrcRateA;
    result = m_hal->SetRateGenBSource(rateBSource);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetRateGenBSource(%d) failed.",
                (int) rateBSource);
        return false;
    }

    // Enable/disable divisor A
    GenStdsHAL::OnOffState enableDivA = (div_a == 0) ? GenStdsHAL::Off : GenStdsHAL::On;
    result = m_hal->SetRateGenEnabled(GenStdsHAL::GeneratorA, enableDivA);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetRateGenEnabled(GeneratorA, %d) failed.",
                (int) enableDivA);
        return false;
    }

    // Enable/disable divisor B
    GenStdsHAL::OnOffState enableDivB = (div_b == 0) ? GenStdsHAL::Off : GenStdsHAL::On;
    result = m_hal->SetRateGenEnabled(GenStdsHAL::GeneratorB, enableDivB);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetRateGenEnabled(GeneratorB, %d) failed.",
                (int) enableDivB);
        return false;
    }

    // Burst size
    int burstSize = getNumPostSamplesSnapshot();
    result = m_hal->SetBurstSize(burstSize);
    if (result != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitConfigParams error: SetBurstSize(%d) failed.", burstSize);
        return false;
    }

    return true;
}

bool TRGeneralStandards::calcBurstParams()
{
    GenStdsHAL::ChannelActiveRange channelActiveRange = (GenStdsHAL::ChannelActiveRange) m_param_channel_active_range.getSnapshot();
    GenStdsHAL::InputMode inputMode = (GenStdsHAL::InputMode) m_param_input_mode.getSnapshot();

    // In pseudo-differential mode if channel 0 is selected, skip it in the data.
    // Note, in ChSingle and ChRange we do not allow selecting channel 0, but in
    // the other modes it cannot be avoided.
    if ((inputMode == GenStdsHAL::PseudoDifferential) &&
       channelActiveRange != GenStdsHAL::ChSingle &&
       channelActiveRange != GenStdsHAL::ChRange){
        m_bp.skip_count = 1;
    } else {
        m_bp.skip_count = 0;
    }

    // m_bp.all_channels has already been set in checkSampleSizeSettings
    m_bp.num_channels = m_bp.all_channels - m_bp.skip_count;

    if (channelActiveRange == GenStdsHAL::ChSingle){
        m_bp.first_channel = m_param_channel_single.getSnapshot();
    } else if (channelActiveRange == GenStdsHAL::ChRange){
        m_bp.first_channel = m_param_channel_range_first.getSnapshot();
    } else if (inputMode == GenStdsHAL::PseudoDifferential){
        m_bp.first_channel = 1;
    } else {
        m_bp.first_channel = 0;
    }

    m_bp.channel_increment = (inputMode == GenStdsHAL::Differential) ? 2 : 1;

    m_bp.burst_start_timestamp = m_param_read_burst_start_ts_en.getSnapshot();

    m_bp.burst_trigger_src = (m_param_burst_trigger_source.getSnapshot() == ExternalTrigger) ?
        GenStdsHAL::ExtInputSync : GenStdsHAL::BcrOutSync;

    m_bp.dmdma_enabled = m_param_read_dmdma_enabled.getSnapshot();

    // The capacity of the hardware buffer is really one less than advertised.
    m_bp.buffer_size = m_hw.buffer_size - 1;
    
    m_bp.num_samples = getNumPostSamplesSnapshot();

    // Calculate burst size in samples.
    m_bp.burst_size_samples = m_bp.num_samples * m_bp.all_channels;
    
    // Force not using DMDMA if we don't have more than DriverPioThreshold+2
    // samples per burst. Because in that case the driver would revert to PIO
    // with polling.
    if (m_bp.dmdma_enabled && m_bp.burst_size_samples <= DriverPioThreshold+2) {
        errlogSevPrintf(errlogMinor, "genstds RUN_READ_LOOP Warning: Not using DM-DMA due to small burst size.\n");
        m_bp.dmdma_enabled = false;
    }
    
    // Allocate the read buffer.
    try {
        std::vector<uint32_t> read_buffer;
        read_buffer.resize(m_bp.burst_size_samples);
        std::swap(m_read_buffer, read_buffer);
    } catch (std::bad_alloc const &) {
        errlogSevPrintf(errlogMajor, "genstds calcBurstParams Error: Buffer allocation failed.");
        return false;
    }
        
    return true;
}

bool TRGeneralStandards::commitBurstParams()
{
    // Clear FIFO buffer and overflow flag.
    if (!clearBufferAndOverflowFlag()) {
        return false;
    }
    
    // Set IO mode.
    int io_mode = m_bp.dmdma_enabled ? GenStdsHAL::IoModeDMDMA : GenStdsHAL::IoModeDMA;
    if (m_hal->SetIoMode((GenStdsHAL::IoMode) io_mode) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: SetIoMode(%d) failed.", io_mode);
        return false;
    }
    
    // Configure the buffer threshold.
    int buffer_threshold = m_bp.burst_size_samples - 1;
    if (m_hal->SetThresholdLevel(buffer_threshold) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: SetThresholdLevel(%d) failed.",
            buffer_threshold);
        return false;
    }
    
    // Read back the buffer threshold.
    GenStdsHAL::GetResult<int32_t> threshold_rb = m_hal->GetThresholdLevel();
    if (threshold_rb.status != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: GetThresholdLevel failed.");
        return false;
    }
    
    if (threshold_rb.value != buffer_threshold) {
        // For certain older boards, the driver rounds up the threshold to a multiple of 4
        // if it is above 65535, due to a hardware limitation. A larger threshold than
        // we need is inappropriate for us because it will not generate an interrupt at the
        // end of a burst. The vendor suggested to set the threshold to the lower multiple of
        // 4 instead, claiming this will work as long as there are at least 5 active channels,
        // because the hardware will copy the remaining samples faster than the reaction to
        // the interrupt.
        
        // The workaround is inapplicable if the threshold was already a multiple of 4
        // or we have fewer than 5 channels.
        bool is_mul_4 = (buffer_threshold % 4) == 0;
        if (is_mul_4 || m_bp.all_channels < 5) {
            errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: Threshold readback (%d) does not "
                "match configured threshold (%d).", (int)threshold_rb.value, buffer_threshold);
            if (!is_mul_4) {
                errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: Cannot use workaround for "
                    "multiple-of-4 threshold limitation because fewer than 5 channels are active.");
            }
            return false;
        }
        
        // Try to set the threshold to the previous multiple of 4.
        int reduced_threshold = (buffer_threshold / 4) * 4;
        if (m_hal->SetThresholdLevel(reduced_threshold) != GenStdsHAL::success) {
            errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: SetThresholdLevel(%d) failed.",
                reduced_threshold);
            return false;
        }
        
        // Read back the buffer threshold.
        threshold_rb = m_hal->GetThresholdLevel();
        if (threshold_rb.status != GenStdsHAL::success) {
            errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: GetThresholdLevel failed.");
            return false;
        }
        
        // Verify the readback.
        if (threshold_rb.value != reduced_threshold) {
            errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: Threshold readback (%d) does not "
                "match configured threshold (%d), after truncating to a multiple of 4.",
                (int)threshold_rb.value, reduced_threshold);
            return false;
        }
        
        errlogSevPrintf(errlogMinor, "genstds Notice: Using workaround for multiple-of-4 limitation "
            "on buffer threshold. If issues appear, try with fewer channels/samples so the workaround "
            "is not used.");
    }
    
    // Configure an infinite read timeout.
    // This is needed to avoid issues in DM-DMA mode.
    if (m_hal->SetReadTimeout(GenStdsHAL::ReadTimeoutInfinite) != GenStdsHAL::success) {
        if (m_hal->SetReadTimeout(3600) != GenStdsHAL::success) {
            errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: SetReadTimeout(Infinite) failed.");
            return false;
        }
        errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: Could not set the read timeout to infinite; please update the driver."
                                     "Proceeding anyway, but DM-DMA mode may suffer data corruption.");
    }
    
    // Disable overflow check.
    if (m_hal->SetReadOverflowCheck(GenStdsHAL::Off) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: SetReadOverflowCheck(Off) failed.");
        return false;
    }
    
    // Configure interrupt IRQ0.
    GenStdsHAL::Interrupt irq0 = m_bp.burst_start_timestamp ? GenStdsHAL::Irq0BurstStart : GenStdsHAL::Irq0None;
    if (m_hal->ConfigureInterrupt(irq0) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: ConfigureInterrupt Irq0 failed.");
        return false;
    }
    
    // Configure interrupt IRQ1.
    GenStdsHAL::Interrupt irq1 = !m_bp.dmdma_enabled ? GenStdsHAL::Irq1BufThrL2H : GenStdsHAL::Irq1None;
    if (m_hal->ConfigureInterrupt(irq1) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: ConfigureInterrupt Irq1 failed.");
        return false;
    }
    
    // Configure the desired burst trigger source.
    if (m_hal->SetBurstTriggerSource((GenStdsHAL::BurstTriggerSource)m_bp.burst_trigger_src) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: SetBurstTriggerSource(%d) failed.", m_bp.burst_trigger_src);
        return false;
    }
    
    return true;
}

bool TRGeneralStandards::clearBufferAndOverflowFlag()
{
    // Clear the FIFO buffer.
    if (m_hal->ClearFifoBuffer() != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: ClearFifoBuffer failed.");
        return false;
    }
    
    // Clear the buffer overflow flag.
    if (m_hal->ClearBufferOverflowFlag() != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds commitBurstParams Error: ClearBufferOverflowFlag failed.");
        return false;
    }
    
    return true;
}

double TRGeneralStandards::calcVoltageSpan(GenStdsHAL::VoltageRange voltageRange, GenStdsHAL::InputMode inputMode)
{
    double baseSpan = 5000000; // Ridiculous value, errors easy to spot
    switch (voltageRange) {
        case GenStdsHAL::R_Bi_2_5:
        case GenStdsHAL::R_Uni_5: {
            baseSpan = 5;
        } break;
        case GenStdsHAL::R_Bi_5:
        case GenStdsHAL::R_Uni_10: {
            baseSpan = 10;
        } break;
        case GenStdsHAL::R_Bi_10: {
            baseSpan = 20;
        } break;
    }

    int modeFactor = (inputMode == GenStdsHAL::PseudoDifferential || inputMode == GenStdsHAL::Differential) ?
        2 : 1;

    return modeFactor * baseSpan;
}

double TRGeneralStandards::calcVoltageOffset(GenStdsHAL::VoltageRange voltageRange, GenStdsHAL::InputMode inputMode,
        double voltageSpan)
{
    if (!(voltageRange == GenStdsHAL::R_Uni_5 || voltageRange == GenStdsHAL::R_Uni_10) ||
        inputMode == GenStdsHAL::PseudoDifferential || inputMode == GenStdsHAL::Differential)
    {
        // Bipolar data.
        return -voltageSpan/2;
    } else {
        // Unipolar data.
        return 0;
    }
}

bool TRGeneralStandards::readBurst()
{
    int buffer_threshold = m_bp.burst_size_samples - 1;
    int burst_size_bytes = 4 * m_bp.burst_size_samples;

    if (m_bp.burst_start_timestamp) {
        // Wait for a burst to have started.
        BurstStartCondition cond_start(m_hal);
        GenStdsHAL::GetResult<GenStdsHAL::WaitResult> wait_res = m_hal->WaitForInterrupt(GenStdsHAL::Irq0BurstStart, cond_start);
        if (wait_res.status != GenStdsHAL::success) {
            errlogSevPrintf(errlogMajor, "genstds readBurst Error: WaitForInterrupt burst-started failed.");
            return false;
        }
        
        // If the wait was not done, we must be getting stopped.
        if (wait_res.value != GenStdsHAL::WaitDone) {
            if (wait_res.value == GenStdsHAL::WaitCancelled) {
                return true;
            }
            errlogSevPrintf(errlogMajor, "genstds readBurst Error: WaitForInterrupt burst-started unknown result.");
            return false;
        }
        
        epicsTimeGetCurrent(&m_ts.burst_start);
        {
            epicsGuard<asynPortDriver> guard(*this);
            updateTimeStamp(&m_ts.epics_ts);
        }
    }
    
    uint32_t *buffer = &m_read_buffer[0];
    
    if (!m_bp.dmdma_enabled) {
        // Wait until we have a complete burst in the buffer.
        BufferThresholdCondition cond_threshold(m_hal, buffer_threshold);
        GenStdsHAL::GetResult<GenStdsHAL::WaitResult> wait_res =
            m_hal->WaitForInterrupt(GenStdsHAL::Irq1BufThrL2H, cond_threshold);

        if (wait_res.status != GenStdsHAL::success) {
            errlogSevPrintf(errlogMajor, "genstds readBurst Error: WaitForInterrupt buffer-threshold failed.");
            return false;
        } 
        
        // If the wait was not done, we must be getting stopped.
        if (wait_res.value != GenStdsHAL::WaitDone) {
            if (wait_res.value == GenStdsHAL::WaitCancelled) {
                return true;
            }
            errlogSevPrintf(errlogMajor, "genstds readBurst Error: WaitForInterrupt buffer-threshold unknown result.");
            return false;
        }
        
        epicsTimeGetCurrent(&m_ts.burst_completed);
        if (!m_bp.burst_start_timestamp) {
            epicsGuard<asynPortDriver> guard(*this);
            updateTimeStamp(&m_ts.epics_ts);
        }

        // Read data into the buffer.
        GenStdsHAL::GetResult<size_t> read_res = m_hal->ReadFIFOBuffer(buffer, burst_size_bytes);
        if (read_res.status != GenStdsHAL::success) {
            errlogSevPrintf(errlogMajor, "genstds readBurst Error: reading %d bytes from buffer "
                    "in ReadFIFOBuffer failed.", burst_size_bytes);
            return false;
        }
        
        // Expecting to read the entire burst.
        if (((int) read_res.value) != burst_size_bytes) {
            errlogSevPrintf(errlogMajor, "genstds readBurst Error: ReadFIFOBuffer read too little.");
            return false;
        }
        
        epicsTimeGetCurrent(&m_ts.read_completed);
    } else {
        // For DM-DMA, a hacky read procedure is needed due to hardware/driver anomalies.
        
        size_t write_index = 0;
        do {
            // See how many more bytes to end of burst.
            size_t read_bytes = burst_size_bytes - write_index;
            
            // Workaround for read anomaly at end of data.
            if (read_bytes > 8) {
                read_bytes -= 8;
            }
            
            // Perform a read.
            GenStdsHAL::GetResult<size_t> read_res = m_hal->ReadFIFOBuffer((char *)buffer + write_index, read_bytes);
            if (read_res.status != GenStdsHAL::success) {
                if (read_res.status == GenStdsHAL::timeout) {
                    errlogSevPrintf(errlogMajor, "genstds readBurst Error: ReadFIFOBuffer timed out; data corruption may have occurred."
                                 "Restarting read anyway");
                    continue;
                }
                errlogSevPrintf(errlogMajor, "genstds readBurst Error: reading %d bytes from buffer "
                    "in ReadFIFOBuffer failed (write_index=%u).\n", burst_size_bytes, (unsigned int)write_index);
                return false;
            }
            
            // Increment the write index.
            write_index += read_res.value;
        } while (((int) write_index) < burst_size_bytes);
        
        epicsTimeGetCurrent(&m_ts.read_completed);
        if (!m_bp.burst_start_timestamp) {
            epicsGuard<asynPortDriver> guard(*this);
            updateTimeStamp(&m_ts.epics_ts);
        }
    }
    
    // Compute the "timeStamp" field for the NDArrays. This timestamp is
    // obtained at the same time as epics_ts but via epicsTimeGetCurrent.
    epicsTimeStamp timestamp_base;
    if (m_bp.burst_start_timestamp) {
        timestamp_base = m_ts.burst_start;
    } else if (!m_bp.dmdma_enabled) {
        timestamp_base = m_ts.burst_completed;
    } else {
        timestamp_base = m_ts.read_completed;
    }
    m_ts.timestamp = timestamp_base.secPastEpoch + timestamp_base.nsec / 1.e9;
    
    // Increment burst ID.
    m_bp.burst_id = (m_bp.burst_id == INT_MAX) ? 0 : (m_bp.burst_id + 1);

    return true;
}

bool TRGeneralStandards::checkOverflow(bool* had_overflow, int *num_buffer_bursts)
{
    GenStdsHAL::GetResult<GenStdsHAL::BufferOverflow> overflow_result = m_hal->GetBufferOverflow();
    if (overflow_result.status != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds checkOverflow Error: GetBufferOverflow failed.");
        return false;
    }

    bool local_had_overflow = (overflow_result.value != GenStdsHAL::NoOverflow);
    *had_overflow = local_had_overflow;

    if (local_had_overflow) {
        // Calculate how many more non-corrupted bursts there are in the buffer, including the one just read
        *num_buffer_bursts = m_bp.buffer_size / m_bp.burst_size_samples;
        
        // Clear live to prevent software triggers.
        {
            epicsGuard<asynPortDriver> guard(*this);
            m_accepting_sw_triggers = false;
        }
        
        // Cancel acquisition.
        if (!cancelAcquisitionAfterOverflow()) {
            return false;
        }
    }

    return true;
}

bool TRGeneralStandards::cancelAcquisitionAfterOverflow()
{
    // Set the burst-trigger-source to Bursting Disabled, to cancel any ongoing burst.
    // This may result in samples being written to the buffer but that is not an issue.
    if (m_hal->SetBurstTriggerSource(GenStdsHAL::DisableBurst) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds cancelAcquisitionAfterOverflow error: SetBurstTriggerSource(DisableBurst) failed (after overflow).");
        return false;
    }
    
    // Small sleep for the hardware to cancel any burst.
    epicsThreadSleep(0.001);
    
    // Verify that no burst is in progress.
    GenStdsHAL::GetResult<GenStdsHAL::BurstStatus> burst_st = m_hal->GetBurstStatus();
    if (burst_st.status != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds cancelAcquisitionAfterOverflow Error: GetBurstStatus failed.");
        return false;
    }
    if (burst_st.value != GenStdsHAL::Idle) {
        errlogSevPrintf(errlogMajor, "genstds cancelAcquisitionAfterOverflow Error: Burst could not be canceled.");
        return false;
    }
    
    // Set the burst-trigger-source to Software Trigger.
    // From this point on, the device should not generate any more bursts.
    if (m_hal->SetBurstTriggerSource(GenStdsHAL::BcrOutSync) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds cancelAcquisitionAfterOverflow Error: SetBurstTriggerSource(BcrOutSync) failed (after overflow).");
        return false;
    }
    
    return true;
}

void TRGeneralStandards::getWaveformDisplayParams(double* voltageScale, double* voltageOffset)
{
    GenStdsHAL::VoltageRange voltageRange = (GenStdsHAL::VoltageRange) m_param_voltage_range.getSnapshot();
    GenStdsHAL::InputMode inputMode = (GenStdsHAL::InputMode) m_param_input_mode.getSnapshot();

    epicsGuard<asynPortDriver> guard(*this);

    int getting_raw_data;
    getIntegerParam(m_reg_params[GET_RAW_DATA], &getting_raw_data);

    if (getting_raw_data != 0) {
        *voltageScale  = 1;
        *voltageOffset = 0;
    } else {
        double zero_voltage_offset;
        getDoubleParam(m_reg_params[ZERO_VOLTAGE_OFFSET], &zero_voltage_offset);
        
        double voltageSpan = calcVoltageSpan(voltageRange, inputMode);

        *voltageScale = voltageSpan / 65536;
        *voltageOffset = calcVoltageOffset(voltageRange, inputMode, voltageSpan) + zero_voltage_offset;
    }
}

bool TRGeneralStandards::processBurstData()
{
    uint32_t *rawdata = &m_read_buffer[0];
    int num_samples = m_bp.num_samples;
    int num_channels = m_bp.num_channels;
    int channel_start = -m_bp.skip_count;
    
    assert(num_channels <= maxChannelNum);
    
    // Even though we won't always be using TRChannelDataSubmit objects for
    // all of the channels, we nevertheless allocate all of them so as to
    // conform to C++98 which does not support variable-length arrays.
    TRChannelDataSubmit dataSubmits[maxChannelNum];
    double *datas[maxChannelNum];

    int port_addr = m_bp.first_channel;
    for (int channel = 0; channel < num_channels; channel++) {
        dataSubmits[channel].allocateArray(*this, port_addr, NDFloat64, num_samples);
        datas[channel] = (double *) dataSubmits[channel].data();

        port_addr += m_bp.channel_increment;
    }

    // When there is only a single channel in the data, do not check the high parts of the words.
    // This is because some devices fail to write proper tags in single channel mode.
    bool check_high = num_channels - channel_start > 1;

    // Obtain current values of parameters affecting waveform display since
    // these may be set at any time, including in between bursts.
    double voltageScale;
    double voltageOffset;
    getWaveformDisplayParams(&voltageScale, &voltageOffset);

    // Process the data of each sample.
    for (int sample = 0; sample < num_samples; sample++) {
        // The first word in a set of samples must have the high part one, others zero.
        uint16_t expected_high = 1;

        for (int channel = channel_start; channel < num_channels; channel++) {
            // Get the data word.
            uint32_t word = *rawdata++;

            // Check high part of word.
            if (check_high) {
                uint16_t high = word >> 16;
                if (high != expected_high) {
                    errlogSevPrintf(errlogMajor, "genstds processBurstData Error: Word high half mismatch sample=%d channel=%d word=%u.",
                                    sample, channel, (unsigned int)word);
                    return false;
                }
                expected_high = 0;
            }

            // Get the low (data) part of the word.
            uint16_t low = word;

            // Store the value to the right place in the right NDArray.
            if (channel >= 0 && datas[channel] != NULL) {
                datas[channel][sample] = (voltageScale * low) + voltageOffset;
            }
        }
    }

    // Send the NDArrays to the framework.
    port_addr = m_bp.first_channel;
    for (int channel = 0; channel < num_channels; channel++) {
        dataSubmits[channel].submit(*this, port_addr, m_bp.burst_id, m_ts.timestamp, m_ts.epics_ts, NULL);
        port_addr += m_bp.channel_increment;
    }

    // Take the timestamp when the data was processed.
    epicsTimeGetCurrent(&m_ts.data_processed);
    
    // Publish burst information.
    publishBurstMetaInfoGenStds();

    return true;
}

void TRGeneralStandards::publishBurstMetaInfoGenStds()
{
    TRBurstMetaInfo info(m_bp.burst_id);
    
    // Calculate the relative times (us).
    if (m_bp.burst_start_timestamp) {
        epicsTimeStamp &burst_end = m_bp.dmdma_enabled ? m_ts.read_completed : m_ts.burst_completed;
        info.time_burst = 1.e6 * epicsTimeDiffInSeconds(&burst_end, &m_ts.burst_start);
    }
    if (!m_bp.dmdma_enabled) {
        info.time_read = 1.e6 * epicsTimeDiffInSeconds(&m_ts.read_completed, &m_ts.burst_completed);
    }
    info.time_process = 1.e6 * epicsTimeDiffInSeconds(&m_ts.data_processed, &m_ts.read_completed);
    
    // Sent the information to the framework.
    publishBurstMetaInfo(info);
}

void TRGeneralStandards::interruptReading()
{
    assert(!m_interrupting_read);

    // reset synchronization flags and events
    m_interrupting_read = true;
    m_interrupt_read_continue.tryWait();
    m_interrupt_read_done.tryWait();

    // Start the task in the worker thread.
    m_interrupt_read_req.start();
}

void TRGeneralStandards::interruptReadingFromWorker()
{
    assert(m_interrupting_read);

    // Make the HAL immediately return WaitCancelled result from ongoing
    // and future WaitForInterrupt calls.
    m_hal->CancelWait(GenStdsHAL::Irq0BurstStart);
    m_hal->CancelWait(GenStdsHAL::Irq1BufThrL2H);
    
    // Generate a fake burst to make sure that the read thread picks up
    // our stop request soon. This is especially important for DM-DMA,
    // where the read thread calls ReadFIFOBuffer without waiting for a
    // complete burst. The fake burst will be ignored by the read thread
    // as it will see m_read_stop being set when the burst is read.
    generateBurstForDisarm();
    
    // Note: generate_burst_for_disarm is probably not needed anymore since
    // the updated driver is able to abort a read, which we do just below.
    // But we leave it in, since the abort has not been completely tested.
    
    // Abort any pending read and wait for the read thread to report that
    // the read loop is completed. Use a small timeout in the wait, and retry
    // the abort on timeout.
    do {
        m_hal->AbortRead();
    } while (!m_interrupt_read_continue.wait(0.001));
    
    // Allow stopAcquisition to know we are done.
    m_interrupt_read_done.signal();
}

void TRGeneralStandards::generateBurstForDisarm()
{
    // Before changing the clock, ensure that external-sync is disabled.
    // This avoids accidentally outputting a high frequency on the Sync I/O line,
    // which normally during burst operation outputs the trigger.
    if (m_hal->SetExternalSync(GenStdsHAL::Off) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds generateBurstForDisarm Error: SetExternalSync(Off) failed.");
        return;
    }
    
    // Set the burst-trigger-source to Software Trigger.
    if (m_hal->SetBurstTriggerSource(GenStdsHAL::BcrOutSync) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds generateBurstForDisarm Error: SetBurstTriggerSource(BcrOutSync) failed.");
        return;
    }
    
    // Get the minimum divisor value.
    GenStdsHAL::GetResult<int32_t> min_div = m_hal->GetRateGenDivisorLimit(GenStdsHAL::Low);
    if (min_div.status != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds generateBurstForDisarm Error: GetRateGenDivisorLimit(Low) failed.");
        return;
    }
    
    // Configure the sample clock to the maximum possible.
    if (m_hal->SetRateGenDivisor(GenStdsHAL::GeneratorA, min_div.value) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds generateBurstForDisarm Error: SetRateGenDivisor(GeneratorA, minimum) failed.");
        return;
    }
    if (m_hal->SetRateGenEnabled(GenStdsHAL::GeneratorA, GenStdsHAL::On) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds generateBurstForDisarm Error: SetRateGenEnabled(GeneratorA, On) failed.");
        return;
    }
    if (m_hal->SetSampleClockSource(GenStdsHAL::ClkSrcRateA) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds generateBurstForDisarm Error: SetSampleClockSource(ClkSrcRateA) failed.");
        return;
    }
    
    // Wait a bit to make sure settings are effective before sending a trigger.
    epicsThreadSleep(0.001);
    
    // Send a software trigger.
    if (m_hal->SendSoftwareTrigger() != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds generateBurstForDisarm Error: SendSoftwareTrigger failed.");
        return;
    }
}

void TRGeneralStandards::stopAcquisition()
{
    {
        // Stop accepting software triggers.
        epicsGuard<asynPortDriver> guard(*this);
        m_accepting_sw_triggers = false;

        // Synchronization
        if (m_interrupting_read) {
            // Inform interruptReadingFromWorker interrupting was successful.
            m_interrupt_read_continue.signal();
            
            // Wait for interruptReadingFromWorker to complete.
            epicsGuardRelease<asynPortDriver> unlockGuard(guard);
            m_interrupt_read_done.wait();
        }
        m_interrupting_read = false;
    }

    // Reset cancel-wait requests. This must be done otherwise
    // the next read loop would immediately get canceled results
    // from the WaitForInterrupt calls.
    m_hal->ResetWait(GenStdsHAL::Irq0BurstStart);
    m_hal->ResetWait(GenStdsHAL::Irq1BufThrL2H);
    
    // Further deconfiguration
    if (m_hal->SetExternalSync(GenStdsHAL::Off) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds stopAcquisition Error: SetExternalSync(Off) failed.");
    }
    if (m_hal->SetRateGenEnabled(GenStdsHAL::GeneratorA, GenStdsHAL::Off) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds stopAcquisition Error: SetRateGenEnabled(GeneratorA, Off) failed.");
    }
    if (m_hal->SetRateGenEnabled(GenStdsHAL::GeneratorB, GenStdsHAL::Off) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds stopAcquisition Error: SetRateGenEnabled(GeneratorB, Off) failed.");
    }
    if (m_hal->SetSampleClockSource(GenStdsHAL::ClkSrcRateA) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds stopAcquisition Error: SetSampleClockSource(ClkSrcRateA) failed.");
    }
    if (m_hal->SetRateGenBSource(GenStdsHAL::RateBSrcMaster) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds stopAcquisition Error: SetRateGenBSource(RateBSrcMaster) failed.");
    }
    if (m_hal->SetBurstTriggerSource(GenStdsHAL::DisableBurst) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds stopAcquisition Error: SetBurstTriggerSource(Disable) failed.");
    }
    if (m_hal->SetBurstSize(0) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds stopAcquisition Error: SetBurstSize(0) failed.");
    }
    if (m_hal->SetRateGenDivisor(GenStdsHAL::GeneratorA, DefaultDivisorValue) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds stopAcquisition Error: SetRateGenDivisor(GeneratorA, Default) failed.");
    }
    if (m_hal->SetRateGenDivisor(GenStdsHAL::GeneratorB, DefaultDivisorValue) != GenStdsHAL::success) {
        errlogSevPrintf(errlogMajor, "genstds stopAcquisition Error: SetRateGenDivisor(GeneratorB, Default) failed.");
    }
    
    // Clear FIFO buffer and overflow flag, to avoid confusing readings
    // of the buffer status.
    clearBufferAndOverflowFlag();

    // Disable interrupts.
    m_hal->ConfigureInterrupt(GenStdsHAL::Irq0None);
    m_hal->ConfigureInterrupt(GenStdsHAL::Irq1None);
}

// Register the driver with epics
extern "C"
int TRGeneralStandardsConfigure(char const *port_name, char const *device_node,
                               int read_thread_prio_epics, int read_thread_stack_size,
                               int wait_thread_prio_pthread,
                               int max_ad_buffers, size_t max_ad_memory)
{

    if (port_name == NULL || device_node == NULL ||
        read_thread_prio_epics < epicsThreadPriorityMin || read_thread_prio_epics > epicsThreadPriorityMax)
    { 
        fprintf(stderr, "genStdsInitDevice Error: parameters are not valid.\n");
        return 1;
    }
    
    TRGeneralStandards *driver = new TRGeneralStandards(
        port_name, device_node, read_thread_prio_epics, read_thread_stack_size,
        wait_thread_prio_pthread, max_ad_buffers, max_ad_memory);
    
    driver->completeInit();
     
    if (!driver->Open()) {
        fprintf(stderr, "genStdsInitDevice Error: Failed to connect with driver.\n");
        // Must not delete driver to avoid crash in findAsynPortDriver.
        return 1;
    }
    
    return 0;
}

static const iocshArg initArg0 = {"port name", iocshArgString};
static const iocshArg initArg1 = {"device node", iocshArgString};
static const iocshArg initArg2 = {"read thread priority (EPICS units)", iocshArgInt};
static const iocshArg initArg3 = {"read thread stack size", iocshArgInt};
static const iocshArg initArg4 = {"waiter thread priority (pthread units)", iocshArgInt};
static const iocshArg initArg5 = {"max AreaDetector buffers", iocshArgInt};
static const iocshArg initArg6 = {"max AreaDetector memory", iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0, &initArg1, &initArg2, &initArg3, &initArg4, &initArg5, &initArg6};
static const iocshFuncDef initFuncDef = {"genStdsInitDevice", 7, initArgs};

static void initCallFunc(const iocshArgBuf *args)
{
    TRGeneralStandardsConfigure(args[0].sval, args[1].sval, args[2].ival,
                args[3].ival, args[4].ival, args[5].ival, args[6].ival);
}

extern "C" {
    void TRGeneralStandardsRegister(void)
    {
        iocshRegister(&initFuncDef, initCallFunc);
    }
    epicsExportRegistrar(TRGeneralStandardsRegister);
}
