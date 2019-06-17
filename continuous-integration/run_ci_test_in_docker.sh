#!/bin/bash

docker run \
    --volume $PWD:/host  \
    --workdir /host/continuous-integration  \
    $OS  \
    bash -c "./ci_test.sh" || exit $?
