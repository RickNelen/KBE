# -----------------------------------------------------------------------------
# IMPORTS
# -----------------------------------------------------------------------------

from parapy.core import *
import kbeutils.avl as avl

# -----------------------------------------------------------------------------
# AVL ANALYSIS CLASS
# -----------------------------------------------------------------------------


class AvlAnalysis(avl.Interface):
    # -------------------------------------------------------------------------
    # INPUTS
    # -------------------------------------------------------------------------

    # The aircraft that is analysed; default setting is the one that is
    # shown in the tree
    aircraft = Input(in_tree=True)

    # Case settings are a required input, defining for example the required
    # lift coefficient or the angle of attack
    case_settings = Input()

    # -------------------------------------------------------------------------
    # ATTRIBUTES
    # -------------------------------------------------------------------------

    @Attribute
    def configuration(self):
        # The analysis configuration is defined inside the aircraft
        return self.aircraft.avl_configuration

    @Attribute
    def induced_drag(self):
        # Return the induced drag
        return {case_name: result['Totals']['CDtot']
                for case_name, result in self.results.items()}

    @Attribute
    def lift(self):
        # Return the lift
        return {case_name: result['Totals']['CLtot']
                for case_name, result in self.results.items()}

    # -------------------------------------------------------------------------
    # PARTS
    # -------------------------------------------------------------------------

    @Part
    def cases(self):
        # Create the AVL case for each case that needs to be analysed
        return avl.Case(quantify=len(self.case_settings),
                        name=self.case_settings[child.index][0],
                        settings=self.case_settings[child.index][1])
