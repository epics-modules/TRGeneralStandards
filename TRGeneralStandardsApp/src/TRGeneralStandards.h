/* This file is part of the General Standards Digitizer Driver.
 * It is subject to the license terms in the LICENSE.txt file found in the
 * top-level directory of this distribution and at
 * https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
 * of the General Standards Digitizer Driver, including this file, may be copied,
 * modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#ifndef GENSTDSATR_H
#define GENSTDSATR_H

#include "TRBaseDriver.h"
#include "GenStdsHAL.h"
#include "TRWorkerThread.h"

class TRGeneralStandards;

class TRGeneralStandards : public TRBaseDriver, private TRWorkerThreadRunnable
{
public:
    // CONSTRUCTOR

    TRGeneralStandards(
        char const *port_name, char const *device_node,
        int read_thread_prio_epics, int read_thread_stack_size,
        int wait_thread_prio_pthread, int max_ad_buffers,
        size_t max_ad_memory);

private:
    // CONSTANTS

    // We need to know the driver's PIO threshold to avoid using
    // DMDMA when the driver would use PIO anyway due to too little
    // data requested. This should match the driver's value.
    static int const DriverPioThreshold = 32;

    // We set clock divisors to this value if the divider is not needed.
    static int const DefaultDivisorValue = 4000;

    // Maximum number of channels that can exist.
    static int const maxChannelNum = 64;

    // INNER TYPES

    // This is used in Open, readInt32, and refreshStatusInfo to simplify
    // implementation.
    struct GenericGetResult {
        inline GenericGetResult()
        {}
        
        template <class ResultValueType>
        inline GenericGetResult(GenStdsHAL::GetResult<ResultValueType> get_result) :
            status(get_result.status),
            value((int32_t)get_result.value)
        {}
        
        GenStdsHAL::GenStatus status;
        int32_t value;
    };

    // WaitCondition for the number of samples in the buffer exceeding a given
    // value.
    class BufferThresholdCondition : public GenStdsHAL::WaitCondition {
    public:
        inline BufferThresholdCondition(GenStdsHAL *hal, int threshold) :
            m_cond_hal(hal),
            m_threshold(threshold)
        {}
        
        bool isMet ();
        
    private:
        GenStdsHAL *m_cond_hal;
        int m_threshold;
    };
    
    // WaitCondition for burst start.
    class BurstStartCondition : public GenStdsHAL::WaitCondition {
    public:
        inline BurstStartCondition(GenStdsHAL *hal) :
            m_cond_hal(hal)
        {}
        
        bool isMet ();
        
    private:
        GenStdsHAL *m_cond_hal;
    };

    // ENUMS

    // Enumeration of regular parameters
    enum GenStdsAsynParams {
        SEND_SW_TRIGGER,
        REFRESH_STATUS_INFO,
        INITIALIZE,

        FIRMWARE_REVISION,
        MAX_INPUT_CHANNELS,
        MASTER_CLOCK_FREQUENCY,
        MAX_SAMPLE_RATE,
        BUFFER_SIZE,
        DEVICE_TYPE,
        MIN_DIVISOR,
        MAX_DIVISOR,

        AUX_LINE_0_MODE,
        AUX_LINE_1_MODE,            
        AUX_LINE_2_MODE,
        AUX_LINE_3_MODE,
        AUX_INVERT_IN,
        AUX_INVERT_OUT,
        AUX_NOISE_SUPPRESSION,

        BUFFER_OVERFLOW_STATUS,
        BURST_STATUS,
        BUFFER_SAMPLES_NUMBER,

        EXTERNAL_CLOCK_RATE,
        ZERO_VOLTAGE_OFFSET,
        GET_RAW_DATA,

        REG_PARAM_NUMBER
    };

    // Possible states of clock divider calculations
    enum ClockCalcState {
        ClockCalcIdle,
        ClockCalcQueued,
        ClockCalcRunning,
        ClockCalcDirty, // specialization of ClockCalcRunning
    };

    // Possible helper thread requests
    enum HelperRequestType {
        InterruptReadHR,
        ClockCalcHR,
        InitializeHR
    };

    enum TriggerSource {
        ExternalTrigger,
        SoftTrigger,
        TriggerNotAvailable
    };

    enum HelperRequestState {
        HelperRequestFailed,
        HelperRequestSucceeded,
        HelperRequestRunning
    };

    // DATA MEMBERS

    // List of regular parameters' asyn indices
    int m_reg_params[REG_PARAM_NUMBER];

    // Concrete configuration parameters
    // NOTE: update CONF_PARAM_NUMBER on any change!
    TRConfigParam<int>         m_param_burst_trigger_source;
    TRConfigParam<int>         m_param_input_mode;   
    TRConfigParam<int>         m_param_voltage_range;
    TRConfigParam<int>         m_param_channel_active_range;
    TRConfigParam<int, double> m_param_channel_range_first;
    TRConfigParam<int, double> m_param_channel_range_last;
    TRConfigParam<int, double> m_param_channel_single;
    TRConfigParam<int>         m_param_read_burst_start_ts_en;
    TRConfigParam<int>         m_param_read_dmdma_enabled;
    TRConfigParam<int>         m_param_div_a_value;
    TRConfigParam<int>         m_param_div_b_value;

    static const int CONF_PARAM_NUMBER = 11;

    // Worker thread
    TRWorkerThread m_worker;

    // Worker request to interrupt reading
    TRWorkerThreadTask m_interrupt_read_req;

    // Synchronization flags and events for above request
    epicsEvent m_interrupt_read_continue;
    epicsEvent m_interrupt_read_done;
    bool m_interrupting_read;

    // Clock divisor calculation request and its state
    TRWorkerThreadTask m_clock_calc_req;
    ClockCalcState	    m_clock_calc_state;

    // Previous requested sample rate, remembered so that we do not send too
    // many requests to the worker thread.
    double m_previous_requested_rate;

    // Synchronization flags and events for clock divisor request
    bool m_read_loop_is_waiting_for_worker;
    epicsEvent m_worker_done;

    // Quantities related to external or internal clock selection
    bool m_using_external_clock;
    double m_private_rate_for_display;

    // (Optional) device initialization request and synchronization flag
    TRWorkerThreadTask m_initialize_req;
    bool m_init_req_running;

    // Hardware information only read at startup
    struct HardwareInfo {
        int max_channel_num;
        int master_freq;
        int max_sample_rate;
        int buffer_size;    // Size in samples
        int min_div;
        int max_div;
    } m_hw;

    bool m_opened;
    bool m_accepting_sw_triggers;

    // Quantities necessary for interpreting data in the device buffer and
    // assigning it to channels, set or calculated from user-supplied
    // parameters
    struct BurstParamsInfo {
        int all_channels;
        int num_channels;
        int num_samples;
        int burst_size_samples;
        int first_channel;
        int channel_increment;
        int skip_count;
        int burst_start_timestamp;
        int burst_trigger_src;
        int dmdma_enabled;
        int buffer_size;
        int burst_id;
    } m_bp;

    // Timestamp variables
    struct ReadTimestampInfo {
        epicsTimeStamp epics_ts;	    // the epicsTimeStamp for the NDArrays, from updateTimeStamp
        epicsTimeStamp burst_start;     // timestamp based on burst-timestamp interrupt (only if burst_start_timestamp enabled)
        epicsTimeStamp burst_completed; // timestamp when we found burst to be complete (only if not dmdma_enabled)
        epicsTimeStamp read_completed;  // timestamp when we have finished reading the burst data
        epicsTimeStamp data_processed;  // timestamp when we have finished processing the data
        double	timestamp;
    } m_ts;

    // Read buffer
    std::vector<uint32_t> m_read_buffer;

    // Pointer to Hardware Abstraction Layer instantiation
    GenStdsHAL *m_hal;

    // METHODS

public:
    bool Open();

private:
    // Parameter access for clients
    asynStatus writeInt32(asynUser *pasynUser, int value); // override
    asynStatus readInt32(asynUser *pasynUser, int32_t *value); // override

    // Handlers for requests via asyn parameter writes.
    GenStdsHAL::GenStatus handleSendSoftwareTrigger();
    asynStatus handleRefreshStatusInfo();
    asynStatus handleInitializeRequest();

    // Besides locking the port this override also waits for any running/queued
    // worker requests to finish.
    bool waitForPreconditions (); // override

    bool checkSettings(TRArmInfo &arm_info); // override

    // Prepare hardware for reading data.
    bool startAcquisition(bool had_overflow); // override

    // Wait for and read a burst of data.
    bool readBurst(); // override

    // Distribute data among output NDArrays.
    bool processBurstData(); // override

    // Communicates timestamps and burst id to the user.
    void publishBurstMetaInfoGenStds();

    // Do finalizing actions after reading a full burst, mostly setting
    // timestamps.
    bool finalizeBurstRead();

    bool checkOverflow (bool *had_overflow, int *num_buffer_bursts); // override
    bool cancelAcquisitionAfterOverflow();
    
    // Execute the Initialize request in the worker thread.
    void initializeFromWorker();

    // Divisor calculation-related quants
    void requestedSampleRateChanged(); // override
    void calculateClockFromWorker();
    void calculateRateGenDivisors(double rate, int* div_a, int* div_b);
    double calculateAchievableSampleRate(int div_a, int div_b);
    void updateClockCalcParams(int div_a, int div_b, double achievable_rate);

    // prepareAcquisition related
    bool checkChannelSettings();
    bool checkClockSettings();
    bool checkSampleSizeSettings();
    int  calcNumDataChannels();
    bool setDivisorAForAutocalibrate();
    bool commitConfigParams();
    bool calcBurstParams();
    bool commitBurstParams();
    bool clearBufferAndOverflowFlag();

    // Voltage offset and scale calculation
    double calcVoltageOffset(GenStdsHAL::VoltageRange, GenStdsHAL::InputMode, double voltageSpan);
    double calcVoltageSpan(GenStdsHAL::VoltageRange, GenStdsHAL::InputMode);
    void getWaveformDisplayParams(double* voltageScale, double* voltageOffset);

    void interruptReading();
    void interruptReadingFromWorker();
    void generateBurstForDisarm();
    void stopAcquisition(); // override

    // The only TRWorkerThreadRunnable method
    void runWorkerThreadTask(int id);
};

#endif /* GENSTDSATR_H */
