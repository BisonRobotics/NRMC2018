#!/usr/bin/python

import sys
for line in sys.stdin:
    if "inet addr:" in line:
        print line[20:34].split()[0]