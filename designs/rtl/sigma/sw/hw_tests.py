# -*- coding:utf-8 -*-
from __future__ import division

import sys

sys.path.append('../../udm/sw')
import udm
from udm import *

import sigma
from sigma import *

# verify_data = [0 for i in range (256)]
# print(verify_data[100])

udm = udm('COM6', 921600)
print("")

sigma = sigma(udm)
sigma.hw_test_tracer(sigma, 'test_1', 'benchmarks/my_test.riscv', 5)

udm.disconnect()
