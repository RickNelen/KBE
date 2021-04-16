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
    n_passengers = Input(4)

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
        turns = 0
        start = 0.1
        step = 0.03
        pav = PAV(name='initial',
                  longitudinal_wing_position=start,
                  centre_of_gravity=[2, 0, 0.1],
                  number_of_passengers=self.n_passengers,
                  r_propeller=0.4)
        indicate = 0
        pos = start + (indicate - 1) * step
        while (abs(pav.expected_maximum_take_off_weight -
                   pav.maximum_take_off_weight) >
               self.allowable_mass_difference) and turns < 4:
            turns += 1
            print('Turns', turns)
            parts = 0
            mass = pav.expected_maximum_take_off_weight
            print('mass:', mass)
            cog = pav.centre_of_gravity_result
            print('cg:', cog)
            print('wing_position:', pos)
            pav = PAV(name='intermediate',
                      maximum_take_off_weight=mass,
                      longitudinal_wing_position=pos,
                      centre_of_gravity=cog,
                      number_of_passengers=self.n_passengers,
                      r_propeller=0.4)
        # pav = PAV(name='initial',
        #           longitudinal_wing_position=start,
        #           centre_of_gravity=[2, 0, 0.1],
        #           number_of_passengers=self.n_passengers,
        #           r_propeller=0.4)
            old_tail_surface = (pav.horizontal_tail_area +
                                pav.vertical_tail_area)
            cog = pav.centre_of_gravity_result
            next_pav = PAV(name='next',
                           maximum_take_off_weight=mass,
                           longitudinal_wing_position=pos,
                           centre_of_gravity=cog,
                           number_of_passengers=self.n_passengers,
                           r_propeller=0.4)
            next_tail_surface = (next_pav.horizontal_tail_area
                                 + pav.vertical_tail_area)
            ratio = start + step
            areas = []
            while ratio < 0.7:
                cog = pav.centre_of_gravity_result
                ratio += step
                parts += 1
                print('Part', parts)
                next_pav = PAV(name='next',
                               maximum_take_off_weight=mass,
                               longitudinal_wing_position=ratio,
                               centre_of_gravity=cog,
                               number_of_passengers=self.n_passengers,
                               r_propeller=0.4)
                next_tail_surface = (next_pav.horizontal_tail_area
                                     + pav.vertical_tail_area)
                areas.append(next_pav.horizontal_tail_area)
            indicate = areas.index(max(areas))
        pos = start + (indicate - 1) * step
        return [pos, mass, cog]

    # @Part(in_tree=False)
    # def initial_aircraft(self):
    #     return PAV(name='initial',
    #                longitudinal_wing_position=0.1,
    #                number_of_passengers=self.n_passengers,
    #                r_propeller=0.4)

    @Part
    def new_aircraft(self):
        return PAV(name='new',
                   maximum_take_off_weight=self.wing_positioning[1],
                   longitudinal_wing_position=self.wing_positioning[0],
                   centre_of_gravity=self.wing_positioning[2],
                   primary_colour='green',
                   number_of_passengers=self.n_passengers,
                   r_propeller=0.4)


if __name__ == '__main__':
    from parapy.gui import display

    obj = Iterator(label='PAV')
    display(obj)
