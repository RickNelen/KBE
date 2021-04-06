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

from math import radians, tan
from parapy.geom import *
from parapy.core import *
from airfoil import Airfoil
# from ref_frame import Frame


class Skidconnection(LoftedSolid):  # note use of loftedSolid as superclass
    skidc = Input('simm_airfoil')

    w_skidc = Input(0.3)
    t_factor = Input(0.08)

    w_skidc_length = Input(1.)
    sweep = Input(5.)
    twist = Input(0)

    @Attribute  # required input for the superclass LoftedSolid
    def profiles(self):
        return [self.root_airfoil, self.tip_airfoil]

    # @Part
    # def lifting_surf_frame(self):  # to visualize the given lifting surface reference frame
    #     return Frame(pos=self.position,
    #                  hidden=False)

    @Part
    def root_airfoil(self):  # root airfoil will receive self.position as default
        return Airfoil(airfoil_name=self.skidc,
                       chord=self.w_skidc,
                       # thickness_factor=self.t_factor,
                       mesh_deflection=0.0001)

    @Part
    def tip_airfoil(self):
        return Airfoil(airfoil_name=self.skidc,
                       chord=self.w_skidc,
                       # thickness_factor=self.t_factor,
                       position=translate(
                           rotate(self.position, "y", radians(self.twist)),  # apply twist angle
                           "y", self.w_skidc_length,
                           "x", self.w_skidc_length * tan(radians(self.sweep))),
                       # apply sweep
                       mesh_deflection=0.0001)

    # @Part
    # def lofted_surf(self):
    #     return LoftedSurface(profiles=self.profiles,
    #                          hidden=not(__name__ == '__main__'))


if __name__ == '__main__':
    from parapy.gui import display
    obj = Skidconnection(label="skid connection",
                         mesh_deflection=0.0001
                         )
    display(obj)
