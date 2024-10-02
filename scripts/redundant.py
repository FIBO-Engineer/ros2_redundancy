#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from redundant_node import *

if __name__ == '__main__':
    try:
        redundant(args=None)
    except Exception as e:
            print(f'The error is {e}')