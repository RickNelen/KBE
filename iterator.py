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

from pav_classes.pav import PAV


class Iterator(Base):
    allowable_mass_difference = Input(50)

    # @Attribute
    # def new_maximum_take_off_weight(self):
    #     pav = PAV(name='initial')
    #     while abs(pav.expected_maximum_take_off_weight -
    #               pav.maximum_take_off_weight) > self.allowable_mass_difference:
    #         mass = pav.expected_maximum_take_off_weight
    #         pav = PAV(name='intermediate',
    #                   maximum_take_off_weight=mass)
    #     return pav.expected_maximum_take_off_weight

    @Attribute
    def wing_positioning(self):
        start = 0.1
        step = 0.03
        pav = PAV(name='initial',
                  longitudinal_wing_position=start,
                  centre_of_gravity=[2, 0, 0.1])
        old_tail_surface = pav.horizontal_tail_area + pav.vertical_tail_area
        next_pav = PAV(name='next',
                       longitudinal_wing_position=start + step,
                       centre_of_gravity=[2, 0, 0.1])
        next_tail_surface = (next_pav.horizontal_tail_area
                             + pav.vertical_tail_area)
        ratio = start + step
        while next_tail_surface < old_tail_surface:
            old_tail_surface = next_tail_surface
            ratio += step
            next_pav = PAV(name='next',
                           longitudinal_wing_position=ratio,
                           centre_of_gravity=[2, 0, 0.1])
            next_tail_surface = (next_pav.horizontal_tail_area
                                 + pav.vertical_tail_area)
        return ratio - step

    @Part
    def initial_aircraft(self):
        return PAV(name='initial')

    @Part
    def new_aircraft(self):
        return PAV(name='new',
                   longitudinal_wing_position=self.wing_positioning,
                   centre_of_gravity=[2, 0, 0.1],
                   primary_colour='green')


if __name__ == '__main__':
    from parapy.gui import display

    obj = Iterator()
    display(obj)
