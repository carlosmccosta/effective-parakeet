#!/bin/bash
echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" || exit $?

$SCRIPT_DIR/setup.sh || exit $?

UBUNTU_VERSION="$(lsb_release -rs)" || exit $?
if [[ "$UBUNTU_VERSION" != "16.04" ]]; then
    $SCRIPT_DIR/lint.sh || exit $?
fi

$SCRIPT_DIR/build.sh || exit $?

$SCRIPT_DIR/test.sh || exit $?
