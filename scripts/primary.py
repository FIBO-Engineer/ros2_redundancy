#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from primary_node import *

if __name__ == '__main__':
    try:
        primary(args=None)
    except Exception as e:
        print(f'The error is {e}')