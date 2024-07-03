#!/bin/bash

if which nproc > /dev/null; then
    MAKEOPTS="-rRj$(nproc)"
else
    MAKEOPTS="-rRj$(sysctl -n hw.ncpu)"
fi

########################################################################################
# code formatting

function ci_code_formatting_setup {
    sudo apt-get install uncrustify
    pip3 install black
    uncrustify --version
    black --version
}

function ci_code_formatting_run {
    tools/codeformat.py -v
}

########################################################################################
# code spelling

function ci_code_spell_setup {
    pip install codespell
}

function ci_code_spell_run {
    codespell firmware/ src/
}

########################################################################################
# tests

function ci_tests_setup {
    sudo dpkg --add-architecture i386
    sudo apt-get update
    sudo apt-get install gcc-multilib
}

function ci_tests_build {
    make $MAKEOPTS -C tests/sdio
    make $MAKEOPTS -C tests/debug_print
}

function ci_tests_run {
    make $MAKEOPTS -C tests/sdio test
    make $MAKEOPTS -C tests/debug_print test
}

########################################################################################
# tests with clang

function ci_tests_clang_setup {
    sudo apt-get install clang gcc-multilib
}

function ci_tests_clang_build {
    make $MAKEOPTS CC=clang -C tests/sdio
    make $MAKEOPTS CC=clang -C tests/debug_print
}

function ci_tests_clang_run {
    make $MAKEOPTS CC=clang -C tests/sdio test
    make $MAKEOPTS CC=clang -C tests/debug_print test
}
