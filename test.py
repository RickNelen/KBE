from fpdf import FPDF
from datetime import date
import os.path

from reportlab.lib.colors import blue
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import inch
from reportlab.pdfgen.canvas import Canvas

from pav_classes.fuselage import Fuselage
from pav_classes.lifting_surface import LiftingSurface
from pav_classes.airfoil import Airfoil
from pav_classes.propeller import Propeller
from pav_classes.pav import PAV

_module_dir = os.path.abspath(os.path.dirname(__file__))
OUTPUT_DIR = os.path.join(_module_dir, 'output_files', '')

FILENAME = os.path.join(OUTPUT_DIR, 'Invoice.pdf')

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
#
#     display(pav)
#     pav.step_parts.write()
number = 51


# ext = str(round(pav.wing_span, 2))


class PDF(FPDF):
    pdf_w = 210
    pdf_h = 297
    rect_inner_margin = 10
    rect_outer_margin = 5
    text_width = pdf_w - 2 * rect_inner_margin
    line_height = 6

    # txtline = 'test \n' + str(number) + '\n' + ext
    left_text = ['Client: G. La Rocca', 'Invoice number: 423423']
    right_text = ['Location: Delft',
                  'Date: ' + date.today().strftime('%d-%m-%Y')]

    written = [0, 0]
    index = 0
    left_len = [len(x) + 100 for x in left_text]
    right_len = [30 - len(x) for x in right_text]
    for left, right in zip(left_text, right_text):
        written[index] = '{:<{lenleft}}{:>{lenright}}'.format(left, right,
                                                              lenleft=left_len[index],
                                                              lenright=right_len[index])
        index += 1

    def lines(self):
        self.set_fill_color(0, 0, 0)
        # self.rect(self.rect_inner_margin, self.rect_inner_margin,
        #           self.pdf_w - 2 * self.rect_inner_margin,
        #           self.pdf_h - 2 * self.rect_inner_margin,
        #           'D')
        self.rect(self.rect_outer_margin, self.rect_outer_margin,
                  self.pdf_w - 2 * self.rect_outer_margin,
                  self.pdf_h - 2 * self.rect_outer_margin,
                  'D')

    def titles(self):
        self.set_xy(0., 0.)
        self.set_font('Arial', size=18)
        self.cell(w=self.pdf_w, h=0.1 * self.pdf_h, align='C',
                  txt='Invoice for PAV')

    def client(self):
        self.set_xy(self.rect_inner_margin, 0.1 * self.pdf_h)
        self.set_font('Arial', size=11)
        self.multi_cell(w=self.text_width, h=self.line_height, align='L',
                        txt='Client: G. La Rocca'
                            + 'Location: Delft'.rjust(136))
        self.multi_cell(w=self.text_width, h=2 * self.line_height, align='L',
                        txt='{:<110s}{:>20s}'.format('Invoice number: 43251',
                                                     'Date: ' +
                                                     date.today().strftime(
                                                         '%d-%m-%Y')))
        self.multi_cell(w=self.text_width, h=self.line_height, align='L',
                        txt=self.written[0] + '\n' + self.written[1])

    def header_geom(self):
        self.set_xy(0., 0.2 * self.pdf_h)
        self.set_font('Arial', size=14)
        self.cell(w=self.pdf_w, h=0.1 * self.pdf_h, align='C',
                  txt='PAV Characteristics')

    def geom(self):
        self.set_xy(self.rect_inner_margin, 0.25 * self.pdf_h)
        self.set_font('Arial', size=11)
        self.multi_cell(w=0.6 * self.text_width, h=self.line_height, align='L',
                        txt='Wing span'
                            + str(pav.wing_span).rjust(136))


pdf = PDF()
pdf.add_page()
pdf.lines()
pdf.titles()
pdf.client()
pdf.header_geom()
pdf.geom()

# pdf.set_font('Arial', size=11)
# pdf.cell(200, 10, txt='Personal Aerial Vehicle', ln=1, align='C')
# pdf.cell(200, 50, txt='Invoice number: 5 \n'
#                       'Date of invoice: 16-04-2021'
#                       'Name of client: Gianfranco', align='L')
pdf.output(FILENAME)
#
# canvas = Canvas(FILENAME, pagesize=A4)
#
# # Set font to Times New Roman with 11-point size
# # canvas.setFont("Times-Roman", 11)
#
# # Draw blue text one inch from the left and ten
# # inches from the bottom
# # canvas.setFillColor(blue)
# number = 5.
#
# textobject = canvas.beginText(1 * inch, 12 * inch)
# textobject.setFont("Times-Roman", 11)
# lines = ['test 1', 'test 2', 'number', str(number)]
# for line in lines:
#     textobject.textLine(line)
#
#
# canvas.drawText(textobject)
#
# # Save the PDF file
# canvas.save()

# display(prop)
