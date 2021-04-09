import os.path
from fpdf import FPDF
from datetime import date
from math import *
import kbeutils.avl as avl

from reportlab.lib.colors import blue
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import *
from reportlab.pdfgen.canvas import Canvas

from pav_classes.fuselage import Fuselage
from pav_classes.lifting_surface import LiftingSurface
from pav_classes.airfoil import Airfoil
from pav_classes.propeller import Propeller
from pav_classes.pav import PAV
from pav_classes.avl_configurator import AvlAnalysis

_module_dir = os.path.abspath(os.path.dirname(__file__))

INPUT_DIR = os.path.join(_module_dir, 'input_files', '')
INPUT_FILE = os.path.join(INPUT_DIR, 'Client_input.txt')

OUTPUT_DIR = os.path.join(_module_dir, 'output_files', '')
FILENAME = os.path.join(OUTPUT_DIR, 'Invoice.pdf')

# -----------------------------------------------------------------------------
# Read the input file and obtain the preferred parameters
# -----------------------------------------------------------------------------
with open(INPUT_FILE, encoding="Latin-1") as f:
    contents = f.read()

    # This is the start of the usable part
    start = contents.find('Number of passengers')
    contents = contents[start:].split()

    # Obtain the parameters directly from the input file
    passengers = ceil(float(contents[3]))
    range_in_km = float(contents[7])
    max_span = float(contents[12])
    quality_choice = int(float(contents[15]))
    cruise_speed = float(contents[22])
    primary_colour_in = str(contents[25])
    secondary_colour_in = str(contents[28])

    options = ['yes', 'Yes', 'no', 'No']
    # Return the choice True or False for wheels
    wheels_choice = (True if contents[17] == options[0]
                     or contents[17] == options[1] else False)


# -----------------------------------------------------------------------------
# Run the KBE app
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    from parapy.gui import display

    obj = Fuselage(color='blue')
    wing = LiftingSurface(name='right_wing')
    prop = Propeller(color='red')
    pav = PAV(label='PAV',
              number_of_passengers=passengers,
              range=range_in_km,
              maximum_span=max_span,
              quality_level=quality_choice,
              wheels=wheels_choice,
              cruise_velocity=cruise_speed,
              primary_colour=primary_colour_in,
              secondary_colour=secondary_colour_in,
              name='PAV')

    cases = [('fixed_aoa', {'alpha': 3}),
             ('fixed_cl', {'alpha': avl.Parameter(name='alpha',
                                                  value='0.3',
                                                  setting='CL')})]

    analysis = AvlAnalysis(aircraft=pav,
                           case_settings=cases)

    display(pav)

    # print(analysis.lift_over_drag)

#     pav.step_parts.write()

# -----------------------------------------------------------------------------
# Get all the parameters to generate a pdf output
# -----------------------------------------------------------------------------

n_passengers = str(pav.number_of_passengers)
baggage = format(pav.number_of_passengers * pav.quality_level * 22, '.0f')
range = format(pav.range, '.0f')
velocity = format(pav.cruise_velocity, '.0f')
quality = 'Economy' if pav.quality_level == 1 else 'Business'
wheels = 'Yes' if pav.wheels is True else 'No'
span = format(pav.wing_span, '.2f')
length = format(pav.fuselage_length, '.2f')
prim_col = pav.primary_colour.capitalize()
sec_col = pav.secondary_colour.capitalize()

char_names = ['Number of passengers: \n', 'Total baggage allowance: \n',
              'Maximum range: \n', 'Cruise velocity: \n', 'Cabin class: \n',
              'Wheels included: \n', 'Wing span: \n', 'Fuselage length: \n',
              'Primary colour: \n', 'Secondary colour: \n']
char_values = [n_passengers + ' \n', baggage + ' kg \n', range + ' km \n',
               velocity + ' km/h \n', quality + ' \n', wheels + ' \n',
               span + ' m \n', length + ' m \n', prim_col + ' \n',
               sec_col + ' \n']

base_price = 32424.141
quality_price = (0 if pav.quality_level == 1
                 else 1000 * pav.number_of_passengers)
wheel_price = 3000
primary_price = 0 if pav.primary_colour == 'white' else 250
secondary_price = 0 if pav.secondary_colour == 'red' else 250
total_price = (base_price + quality_price + wheel_price + primary_price
               + secondary_price)

base_cost = '${:,.2f}'.format(base_price)
quality_cost = '${:,.2f}'.format(quality_price)
wheel_cost = '${:,.2f}'.format(wheel_price)
prim_col_cost = '${:,.2f}'.format(primary_price)
sec_col_cost = '${:,.2f}'.format(secondary_price)
total_cost = '${:,.2f}'.format(total_price)

cost_names = ['Basic price: \n', 'Additional cost for cabin desgin: \n',
              'Cost for primary colour: \n', 'Cost for secondary colour: \n']
cost_names.insert(2, 'Additonal cost for wheels: \n') if pav.wheels is True \
    else None

cost_values = [base_cost + '\n', quality_cost + '\n', prim_col_cost + '\n',
               sec_col_cost + '\n']
cost_values.insert(2, wheel_cost + '\n') if pav.wheels is True else None


class PDF(FPDF):
    pdf_w = 210
    pdf_h = 297
    rect_inner_margin = 10
    rect_outer_margin = 5
    text_width = pdf_w - 2 * rect_inner_margin
    line_height = 8
    start_of_client = 0.1
    start_of_geom = 0.2
    start_of_finance = 0.6
    width_of_names = 0.3 * pdf_w
    width_of_values = 0.15 * pdf_w

    left_text = ['Client: G. La Rocca', 'Invoice number: 423423']
    right_text = ['Location: Delft',
                  'Date: ' + date.today().strftime('%d-%m-%Y')]

    def lines(self):
        self.set_fill_color(0, 0, 0)
        self.rect(self.rect_outer_margin, self.rect_outer_margin,
                  self.pdf_w - 2 * self.rect_outer_margin,
                  self.pdf_h - 2 * self.rect_outer_margin,
                  'D')
        self.line(self.rect_inner_margin, self.start_of_client * self.pdf_h
                  - self.line_height,
                  self.pdf_w - self.rect_inner_margin, self.start_of_client *
                  self.pdf_h - self.line_height)
        self.line(self.rect_inner_margin, self.start_of_geom * self.pdf_h
                  - self.line_height,
                  self.pdf_w - self.rect_inner_margin, self.start_of_geom *
                  self.pdf_h - self.line_height)
        self.line(self.rect_inner_margin, self.start_of_finance * self.pdf_h
                  - self.line_height,
                  self.pdf_w - self.rect_inner_margin, self.start_of_finance *
                  self.pdf_h - self.line_height)
        self.line(self.rect_inner_margin, (self.start_of_finance + 0.05) *
                  self.pdf_h + self.line_height * len(cost_names),
                  self.pdf_w / 2, (self.start_of_finance + 0.05) *
                  self.pdf_h + self.line_height * len(cost_names))
        self.line(self.pdf_w * 0.225, self.pdf_h * 0.9 + self.line_height,
                  self.pdf_w * 0.475, self.pdf_h * 0.9 + self.line_height)
        self.line(self.pdf_w * 0.70, self.pdf_h * 0.9 + self.line_height,
                  self.pdf_w * 0.95, self.pdf_h * 0.9 + self.line_height)

    def titles(self):
        self.set_xy(0., 0.)
        self.set_font('Arial', size=18)
        self.cell(w=self.pdf_w, h=0.1 * self.pdf_h, align='C',
                  txt='Invoice for PAV')

    def left_block(self):
        self.set_xy(self.rect_inner_margin, self.start_of_client * self.pdf_h)
        self.set_font('Arial', size=11)
        self.multi_cell(w=self.pdf_w / 2, h=self.line_height, align='L',
                        txt=self.left_text[0] + '\n' + self.left_text[1])

    def right_block(self):
        self.set_xy(self.pdf_w * 0.7, self.start_of_client * self.pdf_h)
        self.set_font('Arial', size=11)
        self.multi_cell(w=self.pdf_w * 0.3 - self.rect_inner_margin,
                        h=self.line_height, align='L',
                        txt=self.right_text[0] + '\n' + self.right_text[1])

    def header_geom(self):
        self.set_xy(0., self.start_of_geom * self.pdf_h)
        self.set_font('Arial', size=14)
        self.cell(w=self.pdf_w, h=self.line_height, align='C',
                  txt='PAV Characteristics')

    def geom_names(self):
        self.set_xy(self.rect_inner_margin, (self.start_of_geom + 0.05) *
                    self.pdf_h)
        self.set_font('Arial', size=11)
        self.multi_cell(w=self.width_of_names, h=self.line_height, align='L',
                        txt=''.join(char_names))

    def geom_values(self):
        self.set_xy(self.pdf_w / 2 - self.width_of_values,
                    (self.start_of_geom + 0.05) * self.pdf_h)
        self.set_font('Arial', size=11)
        self.multi_cell(w=self.width_of_values, h=self.line_height, align='R',
                        txt=''.join(char_values))

    def header_finance(self):
        self.set_xy(0., self.start_of_finance * self.pdf_h)
        self.set_font('Arial', size=14)
        self.cell(w=self.pdf_w, h=self.line_height, align='C',
                  txt='Cost Overview')

    def price_names(self):
        self.set_xy(self.rect_inner_margin, (self.start_of_finance + 0.05) *
                    self.pdf_h)
        self.set_font('Arial', size=11)
        self.multi_cell(w=self.width_of_names, h=self.line_height, align='L',
                        txt=''.join(cost_names) + 'Total cost:')

    def price_values(self):
        self.set_xy(self.pdf_w / 2 - self.width_of_values,
                    (self.start_of_finance + 0.05) * self.pdf_h)
        self.set_font('Arial', size=11)
        self.multi_cell(w=self.width_of_values, h=self.line_height, align='R',
                        txt=''.join(cost_values) + total_cost)

    def signature_client(self):
        self.set_xy(self.pdf_w * 0.05, self.pdf_h * 0.9)
        self.set_font('Arial', size=11)
        self.cell(w=self.pdf_w * 0.15, h=self.line_height, align='R',
                  txt='Signature client:')

    def signature_pav(self):
        self.set_xy(self.pdf_w * 0.525, self.pdf_h * 0.9)
        self.set_font('Arial', size=11)
        self.cell(w=self.pdf_w * 0.15, h=self.line_height, align='R',
                  txt='Signature PAV:')


pdf = PDF()
pdf.add_page()
pdf.lines()
pdf.titles()
pdf.left_block()
pdf.right_block()
pdf.header_geom()
pdf.geom_names()
pdf.geom_values()
pdf.header_finance()
pdf.price_names()
pdf.price_values()
pdf.signature_client()
pdf.signature_pav()
pdf.output(FILENAME)
