#!/usr/bin/env bash
set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"
exec ../../bin/linux-x86_64/genStdsTestIoc st.cmd
