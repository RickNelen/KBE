# -----------------------------------------------------------------------------
# IMPORTS
# -----------------------------------------------------------------------------

import os.path
from math import *
from iterator import Iterator
from pdf_generator import pdf_creator

# -----------------------------------------------------------------------------
# PATHS
# -----------------------------------------------------------------------------

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

    pav = Iterator(label='PAV',
                   iterate=True,
                   n_passengers=passengers,
                   range_in_km=range_in_km,
                   max_span=max_span,
                   quality_choice=quality_choice,
                   wheels_choice=wheels_choice,
                   cruise_speed=cruise_speed,
                   primary_colour=primary_colour_in,
                   secondary_colour=secondary_colour_in)

    # As the client is assumed to be a non-expert, they are not provided
    # with the AVL analysis that is being run behind the scenes. They get
    # the GUI as clean as possible. If the client does not need to see the
    # vehicle at all, the following line can be commented out.

    display(pav)

    # However, if required, one of the following lines can be uncommented to
    # either show the AVL results for the initial aircraft or the iterated
    # aircraft

    # display(pav.initial_aircraft.analysis)

    # display(pav.new_aircraft.analysis)

    # -----------------------------------------------------------------------------
    # Generate a .stp output
    # -----------------------------------------------------------------------------

    pav.step_part.write()

    # -----------------------------------------------------------------------------
    # Get all the parameters to generate a pdf output
    # -----------------------------------------------------------------------------

    # Obtain the required inputs to generate the pdf
    n_passengers = str(pav.n_passengers)
    baggage = format(pav.n_passengers * pav.quality_choice * 15, '.0f')
    range = format(pav.range_in_km, '.0f')
    velocity = format(pav.cruise_speed, '.0f')
    battery_energy = format(pav.battery_energy, '.0f')
    quality = 'Economy' if pav.quality_choice == 1 else 'Business'
    wheels = 'Yes' if pav.wheels_choice is True else 'No'
    span = format(pav.wing_span, '.2f')
    length = format(pav.fuselage_length, '.2f')
    prim_col = pav.primary_colour.capitalize()
    sec_col = pav.secondary_colour.capitalize()

    # Create the pdf using the separate function pdf_creator
    pdf_creator(n_passengers, baggage, range, velocity, battery_energy,
                quality, wheels, span, length, prim_col, sec_col, FILENAME)
