#!/bin/bash
rsync -rav -e ssh --exclude='.git/' --exclude='.git*' --exclude='.idea/' \
--exclude='build/' --exclude='devel/' --exclude='src/CMakeLists.txt' --exclude='src/cmake-build-debug' \
~/NRMC2018/ nrmc@her-name:/home/nrmc/NRMC2018