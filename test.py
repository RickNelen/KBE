from pav_classes.fuselage import Fuselage
from pav_classes.lifting_surface import LiftingSurface
from pav_classes.airfoil import Airfoil

if __name__ == '__main__':
    from parapy.gui import display

    obj = Airfoil(airfoil_name=45015,
                  chord=4)
    wing = LiftingSurface(airfoil_root=4415,
                         airfoil_tip=43012)
    # display([obj, wing])
    display(wing)
