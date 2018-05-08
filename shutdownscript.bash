#!/bin/bash

ssh -t nrmc@node0 'sudo shutdown now'
ssh -t nrmc@node1 'sudo shutdown now'
ssh -t nrmc@her-name 'sudo shutdown now'
