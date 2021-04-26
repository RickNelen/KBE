# -----------------------------------------------------------------------------
# IMPORTS
# -----------------------------------------------------------------------------

from parapy.core import *
from pav_classes.pav import PAV


# -----------------------------------------------------------------------------
# ITERATOR CLASS
# -----------------------------------------------------------------------------


class Iterator(Base):
    # -------------------------------------------------------------------------
    # INPUTS
    # -------------------------------------------------------------------------

    # The input for iterate is Boolean and determines whether or not the
    # design is iterated upon until convergence; setting it to False yields
    # faster results, while setting it to True provides more accurate results
    iterate = Input(False)

    # The allowable mass difference between iterations in Newtons.
    # This input is used to limit the number of iterations if iterate is set
    # to True; note that for the sake of time, the iterations are also
    # stopped after 3 attempts even if this convergence criterion is not met
    allowable_mass_difference = Input(500)

    # The inputs are basically those for the PAV class which can be decided
    # upon by the client, which will simply be passed on to the PAV.
    n_passengers = Input(4)
    range_in_km = Input(200)
    max_span = Input(18)
    quality_choice = Input(2)
    wheels_choice = Input(True)
    cruise_speed = Input(300)
    primary_colour = Input('white')
    secondary_colour = Input('blue')

    # -------------------------------------------------------------------------
    # ATTRIBUTES
    # -------------------------------------------------------------------------

    @Attribute
    def converge(self):
        # This attribute makes the vehicle design consistent through two
        # loops: an outer loop to converge the mass and an inner loop to
        # move the position of the wing and c.g. to generate a stable aircraft

        # The quarter chord point at the wing root can be moved from 20 % of
        # the fuselage length to 50 % of the fuselage length in steps of 2.5 %
        position_start = 0.2
        position_end = 0.5
        position_step = 0.025

        # Obtain the original guessed mass, the computed mass,
        # the longitudinal wing position and the
        # computed c.g. from the initial aircraft
        initial = self.initial_aircraft
        original_mass = initial.maximum_take_off_weight
        resulting_mass = initial.expected_maximum_take_off_weight
        resulting_position = initial.longitudinal_wing_position
        resulting_cg = initial.centre_of_gravity_result

        # Start counting how many iterations are performed
        outer_loop = 0

        # Perform mass converging iterations as long as the difference is
        # too large and 3 or less results have been obtained
        while (abs(original_mass - resulting_mass) >
               self.allowable_mass_difference) and outer_loop < 3:

            # Add 1 to the number of iterations and reset the number of wing
            # position iterations
            outer_loop += 1

            # Provide a print-out for the number of iterations such that the
            # user knows how fast the program runs
            print('Outer loop:', outer_loop)

            # Update the mass and c.g. to the value obtained in the previous
            # iteration
            original_mass = resulting_mass
            original_cg = resulting_cg

            # Reset the wing position to the start position to prepare for
            # wing moving iteration
            position = position_start

            # Create lists for the wing position, combined area of vertical
            # tail and horizontal tail, mass and c.g., such that in the end
            # the optimal value can be obtained
            position_list = []
            area_list = []
            mass_list = []
            cg_list = []

            # Perform wing positioning iterations for all wing positions
            # between the start position and end position
            while position <= position_end:

                # Obtain the design for each wing position per mass iteration
                intermediate = PAV(label='initial',
                                   number_of_passengers=self.n_passengers,
                                   required_range=self.range_in_km,
                                   maximum_span=self.max_span,
                                   quality_level=self.quality_choice,
                                   wheels_included=self.wheels_choice,
                                   cruise_velocity=self.cruise_speed,
                                   primary_colours=self.primary_colour,
                                   secondary_colours=self.secondary_colour,
                                   maximum_take_off_weight=original_mass,
                                   centre_of_gravity=original_cg,
                                   longitudinal_wing_position=position,
                                   hide_warnings=True,
                                   name='PAV')

                # Obtain the areas of the horizontal and vertical tail,
                # the total mass and the c.g. from this iteration
                h_t_area = intermediate.horizontal_tail_area
                v_t_area = intermediate.vertical_tail_area
                mass = intermediate.expected_maximum_take_off_weight
                cg = intermediate.centre_of_gravity_result

                # Add the values for the wing position, the total area of
                # the empennage, the mass and the c.g. location to their
                # corresponding lists
                position_list.append(position)
                area_list.append(h_t_area + v_t_area)
                mass_list.append(mass)
                cg_list.append(cg)

                # Make sure that the next iteration will start with the wing
                # moved aft by the position_step
                position += position_step

            # After all iterations for the wing position have been performed
            # within the current mass iteration, the minimum area for the
            # empennage is obtained
            area = min(area_list)

            # The index for this minimum area is then used to find the
            # corresponding wing position, mass and c.g.; these are then
            # provided as the final results of this mass iteration and the
            # mass and c.g. will be used as starting points for the next
            # mass iteration
            index = area_list.index(area)
            resulting_position = position_list[index]
            resulting_mass = mass_list[index]
            resulting_cg = cg_list[index]

            # The mass is printed for each mass iteration
            print('Mass:', resulting_mass)

        # Return the converged results for the wing position, mass and c.g.
        return [resulting_position, resulting_mass, resulting_cg]

    # For all the following attributes, it depends on the choice of the user
    # to iterate or not if an attribute is taken from the initial version or
    # the converged design; step_part is needed for the generation of a .stp
    # file, while the other parameters are used in the .pdf file

    @Attribute
    def step_part(self):
        return (self.new_aircraft.step_parts if self.iterate is True
                else self.initial_aircraft.step_parts)

    @Attribute
    def wing_span(self):
        return (self.new_aircraft.wing_span if self.iterate is True
                else self.initial_aircraft.wing_span)

    @Attribute
    def fuselage_length(self):
        return (self.new_aircraft.fuselage_length if self.iterate is True
                else self.initial_aircraft.fuselage_length)

    @Attribute
    def battery_energy(self):
        capacity = (self.new_aircraft.battery_energy if self.iterate is True
                    else self.initial_aircraft.battery_energy)
        # Convert the capacity from Joule to kWh
        return capacity / 3.6e6

    # -------------------------------------------------------------------------
    # PARTS
    # -------------------------------------------------------------------------

    @Part
    def initial_aircraft(self):
        # This is the aircraft without iterations
        return PAV(label='Quick_PAV',
                   number_of_passengers=self.n_passengers,
                   required_range=self.range_in_km,
                   maximum_span=self.max_span,
                   quality_level=self.quality_choice,
                   wheels_included=self.wheels_choice,
                   cruise_velocity=self.cruise_speed,
                   primary_colours=self.primary_colour,
                   secondary_colours=self.secondary_colour,
                   name='PAV_initial')

    @Part
    def new_aircraft(self):
        # This is the aircraft that results from the iterative process
        return PAV(label='Iterated_PAV',
                   number_of_passengers=self.n_passengers,
                   required_range=self.range_in_km,
                   maximum_span=self.max_span,
                   quality_level=self.quality_choice,
                   wheels_included=self.wheels_choice,
                   cruise_velocity=self.cruise_speed,
                   primary_colours=self.primary_colour,
                   secondary_colours=self.secondary_colour,
                   # Note how the following three lines are based on the
                   # outcomes of the converging process
                   maximum_take_off_weight=(self.converge[1]),
                   longitudinal_wing_position=(self.converge[0]),
                   centre_of_gravity=(self.converge[2]),
                   suppress=not self.iterate,
                   name='PAV_final')
