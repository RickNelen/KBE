#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2016 ParaPy Holding B.V.
#
# This file is subject to the terms and conditions defined in
# the license agreement that you have received with this source code
#
# THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
# KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR
# PURPOSE.

from parapy.geom import *
from parapy.core import *


class Wheelrod(Box):



    @Part
    def wheelrod_profile(self):
        return Box(0.04, 0.04, 0.20)



if __name__ == '__main__':
    from parapy.gui import display
    obj = Wheelrod(label="wheelrod", mesh_deflection=0.0001)
    display(obj)
