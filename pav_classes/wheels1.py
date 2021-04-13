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

# from parapy.geom import *
# from parapy.core import *
#
import os.path

from kbeutils.geom import *
import kbeutils.avl as avl
from parapy.core import *
from parapy.geom import *

#
# class Wheels(LoftedSolid):
#
#     """Wheel geometry, a loft through circles."""
#
#     #: wheel radius
#     #: :type: float
#     wheel_radius = Input(0.2)
#     #: wheel sections
#     #: :type: collections.Sequence[float]
#     wheel_sections = Input([90, 100, 100, 100, 100, 100, 100, 100, 100, 100, 90])
#     #: wheel length
#     #: :type: float
#     wheel_length = Input(0.08)
#
#     @Attribute
#     def section_radius(self):
#         """section radius multiplied by the radius distribution
#         through the length. Note that the numbers are percentages.
#
#         :rtype: collections.Sequence[float]
#         """
#         return [i * self.wheel_radius / 100. for i in self.wheel_sections]
#
#     @Attribute
#     def section_length(self):
#         """section length is determined by dividing the wheel
#         length by the number of wheel sections.
#
#         :rtype: float
#         """
#         return self.wheel_length / (len(self.wheel_sections) - 1)
#
#     @Attribute  # used by the superclass LoftedSolid. It could be removed if the @Part profile_set /
#     # would be renamed "profiles" and LoftedSolid specified as superclass for Wheel
#     def profiles(self):
#         return self.profile_set  # collect the elements of the sequence profile_set
#
#     @Part
#     def profile_set(self):
#         return Circle(quantify=len(self.wheel_sections), color="Black",
#                       radius=self.section_radius[child.index],
#                       # wheel along the Y axis
#                       position=translate(self.position,  # circles are in XY plane, thus need rotation
#                                          Vector(0, 0, 1),
#                                          child.index * self.section_length))
#
#
#
#
# if __name__ == '__main__':
#     from parapy.gui import display
#     obj = Wheels(label="wheel", mesh_deflection=0.0001)
#     display(obj)
