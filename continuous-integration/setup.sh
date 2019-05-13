#!/bin/bash

export DEBIAN_FRONTEND=noninteractive

function apt-yes {
    apt-get --assume-yes "$@"
}

apt-yes update || exit $?
apt-yes dist-upgrade || exit $?

apt-yes install \
	wget \
    || exit $?

function install_www_deb {
    TMP_DIR=$(mktemp --tmpdir --directory install_www_deb-XXXX) || exit $?
    pushd $TMP_DIR || exit $?
    wget -q "$@" || exit $?
    apt-yes install --fix-broken ./*deb || exit $?
    popd || exit $?
    rm -r $TMP_DIR || exit $?
}

install_www_deb https://www.zivid.com/hubfs/softwarefiles/releases/1.3.0+bb9ee328-10/u18/telicam-sdk_2.0.0.1-1_amd64.deb || exit $?
install_www_deb https://www.zivid.com/hubfs/softwarefiles/releases/1.3.0+bb9ee328-10/u18/zivid_1.3.0+bb9ee328-10_amd64.deb || exit $?