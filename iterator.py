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
    allowable_mass_difference = Input(500)
    n_passengers = Input(4)
    range_in_km = Input(200)
    max_span = Input(18)
    quality_choice = Input(2)
    wheels_choice = Input(True)
    cruise_speed = Input(300)
    primary_colour = Input('white')
    secondary_colour = Input('blue')
    propeller_radius = Input(0.5)

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
    def converge(self):
        position_start = 0.15
        position_end = 0.6
        position_step = 0.025

        initial = PAV(label='initial',
                      number_of_passengers=self.n_passengers,
                      range=self.range_in_km,
                      maximum_span=self.max_span,
                      quality_level=self.quality_choice,
                      wheels_included=self.wheels_choice,
                      cruise_velocity=self.cruise_speed,
                      primary_colour=self.primary_colour,
                      secondary_colour=self.secondary_colour,
                      r_propeller=self.propeller_radius,
                      name='PAV')
        original_mass = initial.maximum_take_off_weight
        resulting_mass = initial.expected_maximum_take_off_weight
        original_cg = initial.centre_of_gravity
        resulting_cg = initial.centre_of_gravity_result
        initial_position = initial.longitudinal_wing_position

        outer_loop = 0

        # while (abs(original_mass - resulting_mass) >
        #        self.allowable_mass_difference) and \
        while outer_loop < 3:

            outer_loop += 1
            inner_loop = 0
            print('Outer loop:', outer_loop)

            original_mass = resulting_mass
            original_cg = resulting_cg
            position = position_start

            position_list = []
            area_list = []
            mass_list = []
            cg_list = []

            while position <= position_end:
                inner_loop += 1
                # print('Inner loop:', inner_loop)

                intermediate = PAV(label='initial',
                                   number_of_passengers=self.n_passengers,
                                   range=self.range_in_km,
                                   maximum_span=self.max_span,
                                   quality_level=self.quality_choice,
                                   wheels_included=self.wheels_choice,
                                   cruise_velocity=self.cruise_speed,
                                   primary_colour=self.primary_colour,
                                   secondary_colour=self.secondary_colour,
                                   r_propeller=self.propeller_radius,
                                   maximum_take_off_weight=original_mass,
                                   centre_of_gravity=original_cg,
                                   longitudinal_wing_position=position,
                                   name='PAV')

                h_t_area = intermediate.horizontal_tail_area
                v_t_area = intermediate.vertical_tail_area
                mass = intermediate.expected_maximum_take_off_weight
                cg = intermediate.centre_of_gravity_result

                # print('HT area:', h_t_area)
                # print('Mass:', mass)

                position_list.append(position)
                area_list.append(h_t_area + v_t_area)
                mass_list.append(mass)
                cg_list.append(cg)

                position += position_step

            area = min(area_list)
            index = area_list.index(area)
            resulting_position = position_list[index]
            resulting_mass = mass_list[index]
            resulting_cg = cg_list[index]
            print('Combined areas:', area)
            print('Mass:', resulting_mass)
            print('C.G.:', resulting_cg)
            print('Wing position:', resulting_position)

        return [resulting_position, resulting_mass, resulting_cg]

    @Part
    def new_aircraft(self):
        return PAV(label='new',
                   number_of_passengers=self.n_passengers,
                   range=self.range_in_km,
                   maximum_span=self.max_span,
                   quality_level=self.quality_choice,
                   wheels_included=self.wheels_choice,
                   cruise_velocity=self.cruise_speed,
                   primary_colour=self.primary_colour,
                   secondary_colour=self.secondary_colour,
                   r_propeller=self.propeller_radius,
                   maximum_take_off_weight=self.converge[1],
                   longitudinal_wing_position=self.converge[0],
                   centre_of_gravity=self.converge[2],
                   name='PAV')


if __name__ == '__main__':
    from parapy.gui import display

    obj = Iterator(label='PAV')
    display(obj)

    # @Attribute
    # def wing_positioning(self):
    #     turns = 0
    #     start = 0.1
    #     step = 0.03
    #     pav = PAV(name='initial',
    #               longitudinal_wing_position=start,
    #               centre_of_gravity=[2, 0, 0.1],
    #               number_of_passengers=self.n_passengers,
    #               r_propeller=0.4)
    #     indicate = 0
    #     pos = start + (indicate - 1) * step
    #     while (abs(pav.expected_maximum_take_off_weight -
    #                pav.maximum_take_off_weight) >
    #            self.allowable_mass_difference) and turns < 4:
    #         turns += 1
    #         print('Turns', turns)
    #         parts = 0
    #         mass = pav.expected_maximum_take_off_weight
    #         print('mass:', mass)
    #         cog = pav.centre_of_gravity_result
    #         print('cg:', cog)
    #         print('wing_position:', pos)
    #         pav = PAV(name='intermediate',
    #                   maximum_take_off_weight=mass,
    #                   longitudinal_wing_position=pos,
    #                   centre_of_gravity=cog,
    #                   number_of_passengers=self.n_passengers,
    #                   r_propeller=0.4)
    #         # pav = PAV(name='initial',
    #         #           longitudinal_wing_position=start,
    #         #           centre_of_gravity=[2, 0, 0.1],
    #         #           number_of_passengers=self.n_passengers,
    #         #           r_propeller=0.4)
    #         old_tail_surface = (pav.horizontal_tail_area +
    #                             pav.vertical_tail_area)
    #         cog = pav.centre_of_gravity_result
    #         next_pav = PAV(name='next',
    #                        maximum_take_off_weight=mass,
    #                        longitudinal_wing_position=pos,
    #                        centre_of_gravity=cog,
    #                        number_of_passengers=self.n_passengers,
    #                        r_propeller=0.4)
    #         next_tail_surface = (next_pav.horizontal_tail_area
    #                              + pav.vertical_tail_area)
    #         ratio = start + step
    #         areas = []
    #         while ratio < 0.7:
    #             cog = pav.centre_of_gravity_result
    #             ratio += step
    #             parts += 1
    #             print('Part', parts)
    #             next_pav = PAV(name='next',
    #                            maximum_take_off_weight=mass,
    #                            longitudinal_wing_position=ratio,
    #                            centre_of_gravity=cog,
    #                            number_of_passengers=self.n_passengers,
    #                            r_propeller=0.4)
    #             next_tail_surface = (next_pav.horizontal_tail_area
    #                                  + pav.vertical_tail_area)
    #             areas.append(next_pav.horizontal_tail_area)
    #         indicate = areas.index(max(areas))
    #     pos = start + (indicate - 1) * step
    #     return [pos, mass, cog]
    #
    # # @Part(in_tree=False)
    # # def initial_aircraft(self):
    # #     return PAV(name='initial',
    # #                longitudinal_wing_position=0.1,
    # #                number_of_passengers=self.n_passengers,
    # #                r_propeller=0.4)
