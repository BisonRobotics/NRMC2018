#!/bin/bash

find . -name '*.h' -or -name '*.cpp' | xargs clang-format-3.6 -i -style=file $1
