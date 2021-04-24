from fpdf import FPDF
from datetime import date


def pdf_creator(n_passengers, baggage, range, velocity, quality, wheels, span,
                length, prim_col, sec_col, filename):
    char_names = ['Number of passengers: \n', 'Total baggage allowance: \n',
                  'Maximum range: \n', 'Cruise velocity: \n',
                  'Cabin class: \n',
                  'Wheels included: \n', 'Wing span: \n',
                  'Fuselage length: \n',
                  'Primary colour: \n', 'Secondary colour: \n']
    char_values = [n_passengers + ' \n', baggage + ' kg \n', range + ' km \n',
                   velocity + ' km/h \n', quality + ' \n', wheels + ' \n',
                   span + ' m \n', length + ' m \n', prim_col.capitalize() +
                   ' \n', sec_col.capitalize() + ' \n']

    base_price = ((float(span) + float(length) + 5 * float(n_passengers)) * 200
                  + (float(range) + float(velocity)) * 50) + 7500
    quality_price = (0 if quality == 'Economy'
                     else 1000 * float(n_passengers))
    wheel_price = 3000
    primary_price = 0 if prim_col == 'white' else 500
    secondary_price = 0 if sec_col == 'red' else 250
    total_price = (base_price + quality_price + wheel_price + primary_price
                   + secondary_price)

    base_cost = '${:,.2f}'.format(base_price)
    quality_cost = '${:,.2f}'.format(quality_price)
    wheel_cost = '${:,.2f}'.format(wheel_price)
    prim_col_cost = '${:,.2f}'.format(primary_price)
    sec_col_cost = '${:,.2f}'.format(secondary_price)
    total_cost = '${:,.2f}'.format(total_price)

    cost_names = ['Basic price: \n', 'Additional cost for cabin design: \n',
                  'Cost for primary colour: \n',
                  'Cost for secondary colour: \n']
    cost_names.insert(2,
                      'Additional cost for wheels: \n') if wheels == 'Yes' \
        else None

    cost_values = [base_cost + '\n', quality_cost + '\n', prim_col_cost + '\n',
                   sec_col_cost + '\n']
    cost_values.insert(2, wheel_cost + '\n') if wheels == 'Yes' else None

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
                      self.pdf_w - self.rect_inner_margin,
                      self.start_of_client *
                      self.pdf_h - self.line_height)
            self.line(self.rect_inner_margin, self.start_of_geom * self.pdf_h
                      - self.line_height,
                      self.pdf_w - self.rect_inner_margin, self.start_of_geom *
                      self.pdf_h - self.line_height)
            self.line(self.rect_inner_margin,
                      self.start_of_finance * self.pdf_h
                      - self.line_height,
                      self.pdf_w - self.rect_inner_margin,
                      self.start_of_finance *
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
            self.set_xy(self.rect_inner_margin,
                        self.start_of_client * self.pdf_h)
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
            self.multi_cell(w=self.width_of_names, h=self.line_height,
                            align='L',
                            txt=''.join(char_names))

        def geom_values(self):
            self.set_xy(self.pdf_w / 2 - self.width_of_values,
                        (self.start_of_geom + 0.05) * self.pdf_h)
            self.set_font('Arial', size=11)
            self.multi_cell(w=self.width_of_values, h=self.line_height,
                            align='R',
                            txt=''.join(char_values))

        def header_finance(self):
            self.set_xy(0., self.start_of_finance * self.pdf_h)
            self.set_font('Arial', size=14)
            self.cell(w=self.pdf_w, h=self.line_height, align='C',
                      txt='Cost Overview')

        def price_names(self):
            self.set_xy(self.rect_inner_margin,
                        (self.start_of_finance + 0.05) *
                        self.pdf_h)
            self.set_font('Arial', size=11)
            self.multi_cell(w=self.width_of_names, h=self.line_height,
                            align='L',
                            txt=''.join(cost_names) + 'Total cost:')

        def price_values(self):
            self.set_xy(self.pdf_w / 2 - self.width_of_values,
                        (self.start_of_finance + 0.05) * self.pdf_h)
            self.set_font('Arial', size=11)
            self.multi_cell(w=self.width_of_values, h=self.line_height,
                            align='R',
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
    return pdf.output(filename)