#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR=$(realpath "$SCRIPT_DIR/..")

bashFiles=$(find "$ROOT_DIR" -name '*.sh')

echo Running shellcheck on:
echo "$bashFiles"
shellcheck -x -e SC1090,SC2086,SC2046 $bashFiles || exit $?

echo Running code analysis on C++ code:
$SCRIPT_DIR/lint-cpp.sh || exit $?

echo Success! ["$(basename $0)"]
