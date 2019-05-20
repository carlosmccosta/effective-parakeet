#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR=$(realpath "$SCRIPT_DIR/..")

bashFiles=$(find "$ROOT_DIR" -name '*.sh')

echo Running code analysis on C++ code:
$SCRIPT_DIR/lint-cpp.sh || exit $?

echo Success! ["$0"]
