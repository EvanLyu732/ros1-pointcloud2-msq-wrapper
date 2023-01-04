#!/bin/bash

set -e
set -x

declare CUR_PATH=$(pwd)

format() {
    local TEST_CMD="clang-format -h"
    if command -v $TEST_CMD > /dev/null; then
        find $CUR_PATH -iname *.h -o -iname *.cpp | xargs clang-format -i
    else 
        echo "clang-format not installed !"
    fi
}

build() {
    if [[ ! -d ./build ]]; then
        mkdir build
        cmake -S. -Bbuild
    fi

    pushd $CUR_PATH/build > /dev/null;
        make
    popd > /dev/null;
}


format
build
