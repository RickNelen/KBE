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



class Wheelbase(LoftedSolid):

    """Wheel geometry, a loft through circles."""

    #: wheel radius
    #: :type: float
    wheelb_radius = Input(0.18)
    #: wheel sections
    #: :type: collections.Sequence[float]
    wheelb_sections = Input([70, 80, 100, 100, 100, 100, 100, 100, 100, 80, 70])
    #: wheel length
    #: :type: float
    wheelb_length = Input(0.16)

    @Attribute
    def section_radius(self):
        """section radius multiplied by the radius distribution
        through the length. Note that the numbers are percentages.

        :rtype: collections.Sequence[float]
        """
        return [i * self.wheelb_radius / 100. for i in self.wheelb_sections]

    @Attribute
    def section_length(self):
        """section length is determined by dividing the wheel
        length by the number of wheel sections.

        :rtype: float
        """
        return self.wheelb_length / (len(self.wheelb_sections) - 1)

    @Attribute  # used by the superclass LoftedSolid. It could be removed if the @Part profile_set /
    # would be renamed "profiles" and LoftedSolid specified as superclass for Wheel
    def profiles(self):
        return self.profile_set  # collect the elements of the sequence profile_set

    @Part
    def profile_set(self):
        return Circle(quantify=len(self.wheelb_sections), color="Black",
                      radius=self.section_radius[child.index],
                      # wheel along the Y axis
                      position=translate(self.position,  # circles are in XY plane, thus need rotation
                                         Vector(0, 0, 1),
                                         child.index * self.section_length))




if __name__ == '__main__':
    from parapy.gui import display
    obj = Wheelbase(label="wheelbase", mesh_deflection=0.0001)
    display(obj)
