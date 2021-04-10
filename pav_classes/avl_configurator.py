from parapy.core import *
import kbeutils.avl as avl


class AvlAnalysis(avl.Interface):
    aircraft = Input(in_tree=True)
    case_settings = Input()

    @Attribute
    def configuration(self):
        return self.aircraft.avl_configuration

    @Attribute
    def induced_drag(self):
        return {case_name: result['Totals']['CDtot']
                for case_name, result in self.results.items()}

    @Attribute
    def lift_over_drag(self):
        return {case_name: result['Totals']['CLtot']
                / result['Totals']['CDtot']
                for case_name, result in self.results.items()}

    @Part
    def cases(self):
        return avl.Case(quantify=len(self.case_settings),
                        name=self.case_settings[child.index][0],
                        settings=self.case_settings[child.index][1])
