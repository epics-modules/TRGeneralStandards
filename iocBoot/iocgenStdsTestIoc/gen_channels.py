# This file is part of the General Standards Digitizer Driver.
# It is subject to the license terms in the LICENSE.txt file found in the
# top-level directory of this distribution and at
# https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
# of the General Standards Digitizer Driver, including this file, may be copied,
# modified, propagated, or distributed except according to the terms
# contained in the LICENSE.txt file.


from __future__ import print_function
import os
import argparse

INIT_CHANNELS_FILE = 'genStdsInitChannels.cmd'
LOAD_CHANNELS_DB_FILE = 'genStdsLoadChannelsDb.cmd'

def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-n', '--num-channels', type=int, default=64, help='Number of channels to generate')
    args = parser.parse_args()
    
    dir_path = os.path.dirname(os.path.realpath(__file__))
    
    channels = [i for i in range(args.num_channels)]
    
    with open(os.path.join(dir_path, INIT_CHANNELS_FILE), 'w') as f:
        f.write("\n# Initialize the stdArrays plugins.\n")
        for channel in channels:
            f.write('NDStdArraysConfigure("$(DEVICE_NAME)_ch{0:}_stdarrays", $(STDARRAYS_QUEUE_SIZE), $(STDARRAYS_BLOCKING_CALLBACKS), "$(DEVICE_NAME)_channels", {0:}, $(STDARRAYS_MAX_MEMORY), $(STDARRAYS_PRIORITY), $(STDARRAYS_STACK_SIZE))\n'.format(channel))
    
    with open(os.path.join(dir_path, LOAD_CHANNELS_DB_FILE), 'w') as f:
        f.write("# Load records for each chanel.\n")
        for channel in channels:
            f.write('dbLoadRecords("$(TRANSRECORDER_CORE)/db/TRChannel.db", "PREFIX=$(PREFIX):CH{0:}, CHANNELS_PORT=$(DEVICE_NAME)_channels, CHANNEL={0:}")\n'
                    .format(channel))
            f.write('dbLoadRecords("$(TRANSRECORDER_CORE)/db/TRChannelWaveforms.db", "PREFIX=$(PREFIX):CH{0:}, STDARRAYS_PORT=$(DEVICE_NAME)_ch{0:}_stdarrays, SIZE=$(WAVEFORM_SIZE), REFRESH_SNAPSHOT_SCAN=$(REFRESH_SNAPSHOT_SCAN)")\n'
                    .format(channel))

if __name__ == '__main__':
    main()
