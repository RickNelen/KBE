from pav_classes.fuselage import Fuselage
from pav_classes.lifting_surface import LiftingSurface
from pav_classes.airfoil import Airfoil
from pav_classes.propeller import Propeller
from pav_classes.pav import PAV

if __name__ == '__main__':
    from parapy.gui import display

    obj = Fuselage(color='blue')
    wing = LiftingSurface(name='right_wing')
    prop = Propeller(color='red')
    pav = PAV(number_of_passengers=3,
              range=400,
              maximum_span=18,
              quality_level=2,
              wheels=False,
              cruise_velocity=300,
              primary_colour='green',
              secondary_colour='red',
              name='PAV')

    display(pav)
    pav.step_parts.write()
    #display(prop)
