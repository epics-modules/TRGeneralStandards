List of General Standards PVs   {#genstds-pvs}
============

This page describes the public PVs specific to the General Standards digitzer driver
for models 16AI64SSA and 16AI64SSC.

The public [framework PVs](@ref framework-pvs) are automatically supported unless
stated otherwise on this page.
However this page does includes notes about certain framework PVs regarding driver-specific semantics.

## Device Information

These PVs contain static information about the device.
All of this information is obtained from the vendor’s driver.

<table>
    <tr>
        <th>PV name, record type</th>
        <th>Description</th>
    </tr>
    <tr>
        <td valign="top">`GET_FIRMWARE_REVISION` (longin)</td>
        <td>
            The firmware revision as an integer.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_MAX_INPUT_CHANNELS` (longin)</td>
        <td>
            The number of input channels supported by the device.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_MASTER_CLOCK_FREQUENCY` (longin)</td>
        <td>
            The master clock frequency (Hz).
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_MAX_SAMPLE_RATE` (longin)</td>
        <td>
            The maximum sample rate supported by the device (Hz).
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_BUFFER_SIZE` (longin)</td>
        <td>
            The FIFO buffer size (number of samples) as advertised by the driver.
            Note, the usable buffer size appears to be one sample less.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_DEVICE_TYPE` (mbbi)</td>
        <td>
            The type of device.
            
            Values: `16AI64SSA`, `16AI64SSC`, `Unknown`.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_MIN_DIVISOR` (longin)</td>
        <td>
            The minimum permitted divisor value for frequency dividers.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_MAX_DIVISOR` (longin)</td>
        <td>
            The maximum permitted divisor value for frequency dividers.
        </td>
    </tr>
</table>

## Device States

These PVs represent the dynamic states related to burst operation.
They are updated at the scan rate defined by the `REFRESH_STATES_SCAN` template parameter,
and in the test IOC by an environment variable of the same name in `st.cmd`.

<table>
    <tr>
        <th>PV name, record type</th>
        <th>Description</th>
    </tr>
    <tr>
        <td valign="top">`GET_BUFFER_OVERFLOW_STATUS` (bi)</td>
        <td>
            The value of the buffer-overflow flag,
            i.e. whether there has been a buffer overflow.
            
            Values: `No Overflow`, `Overflow`.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_BURST_STATUS` (bi)</td>
        <td>
            The current burst state, i.e. whether a burst is in progress.
            
            Values: `Idle`, `Active`.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_BUFFER_SAMPLES` (longin)</td>
        <td>
            The number of samples in the FIFO sample buffer.
        </td>
    </tr>
</table>

## AUX-IO Settings

These PVs are used for configuring the AUX-IO lines of the board.
Settings written into the `set` records are applied to the device immediately.

The `get` records contain the current value, and must not be written.
Each `set` record has a forward link to update the corresponding `get` record.

<table>
    <tr>
        <th>PV name, record type</th>
        <th>Description</th>
    </tr>
    <tr>
        <td valign="top">`SET_AUX_LINE_<N>_MODE` (mbbo)<br>\<N\>=0,1,2,3</td>
        <td valign="top">
            Write to set the mode of an AUX-IO line
            (`Inactive`, `Active Input`, `Active Output`).
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_AUX_LINE_<N>_MODE` (mbbi)</td>
        <td>
            The current mode of an AUX-IO line (values as above).
        </td>
    </tr>
    <tr>
        <td valign="top">`SET_AUX_INVERT_IN` (bo)</td>
        <td>
            Write to set whether AUX-IO inputs are inverted (`Disabled`, `Enabled`).
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_AUX_INVERT_IN` (bi)</td>
        <td>
            The current AUX-IO inputs inversion mode (values as above).
        </td>
    </tr>
    <tr>
        <td valign="top">`SET_AUX_INVERT_OUT` (bo)</td>
        <td>
            Write to set whether AUX-IO outputs are inverted (`Disabled`, `Enabled`).
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_AUX_INVERT_OUT` (bi)</td>
        <td>
            The current AUX-IO outputs inversion mode (values as above).
        </td>
    </tr>
    <tr>
        <td valign="top">`SET_AUX_NOISE_SUPPRESSION` (bo)</td>
        <td>
            Write to set whether noise suppression for AUX-IO inputs is enabled
            (`Disabled`, `Enabled`).
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_AUX_NOISE_SUPPRESSION` (bi)</td>
        <td>
            The current value of the AUX-IO noise suppression setting (values as above).
        </td>
    </tr>
</table>

## Desired Settings for Arming

These PVs should be set to the desired settings before an arm request is issued.
Their values are captured at the start of arming.

The hardware does not support pre-samples, therefore the PV `numberPPS`
(provided by the framework) must be left at the default value zero.

<table>
    <tr>
        <th>PV name, record type</th>
        <th>Description</th>
    </tr>
    <tr>
        <td valign="top">`DESIRED_INPUT_MODE` (mbbo)</td>
        <td>
            The input mode (“Single-ended”, “Pseudo-Differential”, “Full-Differential”,
            “Zero Test Voltage”, “Reference Voltage”).
        </td>
    </tr>
    <tr>
        <td valign="top">`DESIRED_CHANNEL_ACTIVE_RANGE` (mbbo)</td>
        <td>
            The channel range (“Single”, “Channels 0-1”, “Channels 0-3”, “Channels 0-7”,
            “Channels 0-15”, “Channels 0-31”, “Channels 0-63”, “Channel Range”).
            
            In case of “Single” and “Channel Range”, additional PVs define which
            channel or channel range is used.
        </td>
    </tr>
    <tr>
        <td valign="top">`DESIRED_CHANNEL_SINGLE` (longout)</td>
        <td>
            The channel index to use for single channel mode (otherwise irrelevant).
            
            In pseudo-differential mode, this must not be zero.
            In full-differential mode, this must be even.
        </td>
    </tr>
    <tr>
        <td valign="top">`DESIRED_CHANNEL_RANGE_FIRST` (longout)</td>
        <td>
            The first channel for channel range mode (otherwise irrelevant).
            
            In pseudo-differential mode, this must not be zero.
            In full-differential mode, this must be even.
        </td>
    </tr>
    <tr>
        <td valign="top">`DESIRED_CHANNEL_RANGE_LAST` (longout)</td>
        <td>
            The last channel for channel range mode (otherwise irrelevant).
            
            This must be greater than the first channel.
            In full-differential mode, this must be odd.
        </td>
    </tr>
    <tr>
        <td valign="top">`DESIRED_VOLTAGE_RANGE` (mbbo)</td>
        <td>
            The input voltage range
            (“+-2.5 V”, “0-5 V”, “+-5 V”, “0-10 V”, “+-10 V”).
        </td>
    </tr>
    <tr>
        <td valign="top">`clock` (mbbo)</td>
        <td>
            The desired sample clock source or rate. Supported choices are:
            
            - “Custom Frequency” (`CUSTOM_SAMPLE_RATE` applies),
            - “External Trigger” (external clock),
            - 14 predefined sample rate values (see database, these can easily be changed).
            
            This PV and `CUSTOM_SAMPLE_RATE` are considered inputs for clock calculation,
            and therefore define the value of the PV `ACHIEVABLE_SAMPLE_RATE`
            (provided by the framework). If external clock is selected, then
            `ACHIEVABLE_SAMPLE_RATE` will be calculated as `NAN`.
        </td>
    </tr>
    <tr>
        <td valign="top">`CUSTOM_SAMPLE_RATE` (ao)</td>
        <td>
            The desired sample rate in Hz, when custom frequency is selected.
            
            When using an external sample clock (clock is “External Trigger”), this should be
            set as close as possible to the expected frequency of the external clock. It will
            be used for autocalibration as part of an arming procedure, for the `TIME_DATA`
            waveform, and for sample-rate NDArray attributes.
        </td>
    </tr>
    <tr>
        <td valign="top">`trigger` (mbbo)</td>
        <td>
            Burst trigger source. Supported choices are:
            
            - “extTrigger” (external trigger),
            - “soft” (software trigger). 
        </td>
    </tr>
    <tr>
        <td valign="top">`BURST_START_TIMESTAMP_ENABLED` (bo)</td>
        <td>
            Whether to enable burst-start timestamps, using the burst-started interrupt
            (“Off”, “On”).
        </td>
    </tr>
    <tr>
        <td valign="top">`IO_MODE_DMDMA_ENABLED` (bo)</td>
        <td>
            Whether to enable Demand-Mode DMA (“Off”, “On”).
        </td>
    </tr>
</table>

## Current Armed Settings

With regard to the possible values and meaning of these PVs,
the text in the section of the same name in [framework PVs](@ref framework-pvs) applies.

<table>
    <tr>
        <th>PV name, record type</th>
        <th>Description</th>
    </tr>
    <tr>
        <td valign="top">`GET_ARMED_INPUT_MODE` (mbbi)</td>
        <td>
            Current armed input mode.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_ARMED_CHANNEL_ACTIVE_RANGE` (mbbi)</td>
        <td>
            Current armed channel range.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_ARMED_CHANNEL_SINGLE` (ai)</td>
        <td>
            Current armed channel index for single channel mode,
            `NAN` if single channel mode is not used.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_ARMED_CHANNEL_RANGE_FIRST` (ai)</td>
        <td>
            Current armed first channel for channel range mode,
            `NAN` if channel range mode is not used.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_ARMED_CHANNEL_RANGE_LAST` (ai)</td>
        <td>
            Current armed last channel for channel range mode,
            `NAN` if channel range mode is not used.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_ARMED_VOLTAGE_RANGE` (mbbi)</td>
        <td>
            Current armed voltage range.
        </td>
    </tr>
    <tr>
        <td valign="top">`get_trigger` (mbbi)</td>
        <td>
            Current armed burst trigger source.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_BURST_START_TIMESTAMP_ENABLED` (mbbi)</td>
        <td>
            Current armed selection for use of burst-started timestamps.
        </td>
    </tr>
    <tr>
        <td valign="top">`GET_IO_MODE_DMDMA_ENABLED` (mbbi)</td>
        <td>
            Current armed selection for use of DM-DMA.
        </td>
    </tr>
</table>

## Device Requests

Note that to actually start a request, the value 1 (or the associated string value)
must be written to the PV for the request. Writing any other value will have no effect.
This was done to accommodate EDM, where buttons will write zero when released.

<table>
    <tr>
        <th>PV name, record type</th>
        <th>Description</th>
    </tr>
    <tr>
        <td valign="top">`softTrigger` (bo)</td>
        <td>
            Write 1 or “softTrigger” to send a software trigger to the device.
            
            This only has an effect when the device is armed with software trigger.
        </td>
    </tr>
    <tr>
        <td valign="top">`START_INITIALIZE` (bo)</td>
        <td>
            Write 1 or “Start” to perform the Initialize operation on the device.
            
            The operation is asynchronous and the result will be provided in
            `INITIALIZE_STATUS`. If this is requested while Initialize is already
            running, the request is ignored. If it is requested while the device
            is not disarmed, it will be rejected (`fail` status).
        </td>
    </tr>
    <tr>
        <td valign="top">`INITIALIZE_STATUS` (mbbi)</td>
        <td>
            The state or result of the Initialize operation (empty, `running`, `success`, `fail`).
            
            The result clears to the empty value automatically a few seconds after completion.
            This PV must not be written.
        </td>
    </tr>
</table>

## Acquisition Information

The driver generates burst IDs as suggested in the description of the `BURST_ID`
PV provided by the framework.

The driver also provides the diagnostic times:
- `GET_BURST_START_TO_BURST_END_TIME` (except if armed with burst-timestamps disabled),
- `GET_BURST_END_TO_READ_END_TIME` (except if armed with DM-DMA enabled),
- `GET_READ_END_TO_DATA_PROCESSED_TIME`.

## Acquisition control

<table>
    <tr>
        <th>PV name, record type</th>
        <th>Description</th>
    </tr>
    <tr>
        <td valign="top">`VOLTAGE_DATA_OFFSET` (ai)</td>
        <td>
            The additional voltage offset applied to the sample data,
            after initial conversion to voltage according to the input
            voltage range and input mode.
            
            Changes to this PV will be reflected in newly processed bursts.
            The default is 0.
        </td>
    </tr>
</table>

## Testing PVs

This section lists PVs which are desired to facilitate testing.

There are several PVs which read back configuration from the hardware.
These are updated automatically whenever the arm state changes (`arm` PV) or the
Initialize operation completes, and can also be updated manually by writing to
the PV `REFRESH_READBACKS`.

<table>
    <tr>
        <th>PV name, record type</th>
        <th>Description</th>
    </tr>
    <tr>
        <td valign="top">`REFRESH_READBACKS` (bo)</td>
        <td>
            Write one or `Refresh` to refresh the readbacks listed in this section.
        </td>
    </tr>
    <tr>
        <td valign="top">`INPUT_MODE_RBV` (mbbi)</td>
        <td>
            Readback of the input mode (see `DESIRED_INPUT_MODE`).
        </td>
    </tr>
    <tr>
        <td valign="top">`VOLTAGE_RANGE_RBV` (mbbi)</td>
        <td>
            Readback of the voltage range (see `DESIRED_VOLTAGE_RANGE`).
        </td>
    </tr>
    <tr>
        <td valign="top">`CHANNEL_ACTIVE_RANGE_RBV` (mbbi)</td>
        <td>
            Readback of the channel-active-range (see `DESIRED_CHANNEL_ACTIVE_RANGE`).
        </td>
    </tr>
    <tr>
        <td valign="top">`CHANNEL_RANGE_FIRST_RBV` (longin)</td>
        <td>
            Readback of the first channel number (see `DESIRED_CHANNEL_RANGE_FIRST`).
        </td>
    </tr>
    <tr>
        <td valign="top">`CHANNEL_RANGE_LAST_RBV` (longin)</td>
        <td>
            Readback of the last channel number (see `DESIRED_CHANNEL_RANGE_LAST`).
        </td>
    </tr>
    <tr>
        <td valign="top">`CHANNEL_SINGLE_RBV` (longin)</td>
        <td>
            Readback of the single channel number (see `DESIRED_CHANNEL_SINGLE`).
        </td>
    </tr>
    <tr>
        <td valign="top">`CLOCK_SOURCE_RBV` (mbbi)</td>
        <td>
            Readback of the sample clock source setting
            (`Rate-A`, `Rate-B`, `External Sync`, `Software Trigger`).
        </td>
    </tr>
    <tr>
        <td valign="top">`CALC_DIVISOR_A` (longin)</td>
        <td>
            Calculated divisor of rate generator A corresponding
            to the desired clock configuration. Zero means this
            rate generator would not be used.
        </td>
    </tr>
    <tr>
        <td valign="top">`CALC_DIVISOR_B` (longin)</td>
        <td>
            Calculated divisor of rate generator B corresponding
            to the desired clock configuration. Zero means this
            rate generator would not be used.
        </td>
    </tr>
</table>

