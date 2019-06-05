#!/bin/bash

docker run \
    --volume $PWD:/host  \
    --workdir /host/continuous-integration  \
    $OS  \
    bash -c "./setup.sh && ./lint.sh && ./build.sh" || exit $?
