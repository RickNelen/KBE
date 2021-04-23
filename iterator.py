from parapy.core import *

from pav_classes.pav import PAV


class Iterator(Base):
    iterate = Input(False)
    allowable_mass_difference = Input(500)
    n_passengers = Input(4)
    range_in_km = Input(200)
    max_span = Input(18)
    quality_choice = Input(2)
    wheels_choice = Input(True)
    cruise_speed = Input(300)
    primary_colour = Input('white')
    secondary_colour = Input('blue')

    @Part
    def initial_aircraft(self):
        return PAV(label='Quick_PAV',
                   number_of_passengers=self.n_passengers,
                   required_range=self.range_in_km,
                   maximum_span=self.max_span,
                   quality_level=self.quality_choice,
                   wheels_included=self.wheels_choice,
                   cruise_velocity=self.cruise_speed,
                   primary_colour=self.primary_colour,
                   secondary_colour=self.secondary_colour,
                   # suppress=self.iterate,
                   name='PAV_initial')

    @Attribute
    def converge(self):
        position_start = 0.15
        position_end = 0.6
        position_step = 0.025

        initial = self.initial_aircraft
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
                                   required_range=self.range_in_km,
                                   maximum_span=self.max_span,
                                   quality_level=self.quality_choice,
                                   wheels_included=self.wheels_choice,
                                   cruise_velocity=self.cruise_speed,
                                   primary_colour=self.primary_colour,
                                   secondary_colour=self.secondary_colour,
                                   maximum_take_off_weight=original_mass,
                                   centre_of_gravity=original_cg,
                                   longitudinal_wing_position=position,
                                   hide_warnings=True,
                                   name='PAV')

                h_t_area = intermediate.horizontal_tail_area
                v_t_area = intermediate.vertical_tail_area
                mass = intermediate.expected_maximum_take_off_weight
                cg = intermediate.centre_of_gravity_result

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
        return PAV(label='Iterated_PAV',
                   number_of_passengers=self.n_passengers,
                   required_range=self.range_in_km,
                   maximum_span=self.max_span,
                   quality_level=self.quality_choice,
                   wheels_included=self.wheels_choice,
                   cruise_velocity=self.cruise_speed,
                   primary_colour=self.primary_colour,
                   secondary_colour=self.secondary_colour,
                   maximum_take_off_weight=(self.converge[1]),
                   longitudinal_wing_position=(self.converge[0]),
                   centre_of_gravity=(self.converge[2]),
                   suppress=not self.iterate,
                   name='PAV_final')


if __name__ == '__main__':
    from parapy.gui import display

    obj = Iterator(label='PAV')
    display(obj)
