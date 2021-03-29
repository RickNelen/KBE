from pav_classes.fuselage import Fuselage
from pav_classes.lifting_surface import LiftingSurface
from pav_classes.airfoil import Airfoil
from pav_classes.propeller import Propeller

if __name__ == '__main__':
    from parapy.gui import display

    obj = Fuselage(color='blue')
    wing = LiftingSurface(name='right_wing')
    prop = Propeller(color='red')
    display([obj, wing, prop])
    #display(prop)
