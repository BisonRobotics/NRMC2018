#!/bin/bash

sudo mkdir /usr/src/gmock/build
pushd /usr/src/gmock/build
sudo cmake ../
sudo make
popd


