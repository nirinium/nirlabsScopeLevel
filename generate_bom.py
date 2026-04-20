from fpdf import FPDF
import os

class BOM_PDF(FPDF):
    def header(self):
        self.set_font('Helvetica', 'B', 16)
        self.cell(0, 10, 'NIRINIUM|LABS', new_x="LMARGIN", new_y="NEXT", align='C')
        self.set_font('Helvetica', '', 12)
        self.cell(0, 8, 'Anti-Cant Scope Level - Bill of Materials', new_x="LMARGIN", new_y="NEXT", align='C')
        self.cell(0, 6, 'Rev 2.0 | XIAO SAMD21 Build', new_x="LMARGIN", new_y="NEXT", align='C')
        self.ln(5)

    def footer(self):
        self.set_y(-15)
        self.set_font('Helvetica', 'I', 8)
        self.cell(0, 10, f'Page {self.page_no()}', align='C')

pdf = BOM_PDF()
pdf.add_page()
pdf.set_auto_page_break(auto=True, margin=15)

# Column widths
col_w = [10, 55, 12, 65, 25]
headers = ['#', 'Component', 'Qty', 'Specification', 'Est. Cost']

# Table header
pdf.set_font('Helvetica', 'B', 10)
pdf.set_fill_color(40, 40, 40)
pdf.set_text_color(255, 255, 255)
for i, h in enumerate(headers):
    pdf.cell(col_w[i], 8, h, border=1, align='C', fill=True)
pdf.ln()

# Table data
bom = [
    ['1', 'Seeeduino XIAO SAMD21', '1', 'ARM Cortex-M0+, USB-C, 3.3V', '$5.40'],
    ['2', 'ADXL345 Breakout Board', '1', 'I2C 3-axis accelerometer, 3.3V', '$3.50'],
    ['3', 'Green LED (3mm)', '1', 'Diffused, 20mA, ~2.0V Vf', '$0.10'],
    ['4', 'Red LED (3mm)', '2', 'Diffused, 20mA, ~1.8V Vf', '$0.20'],
    ['5', 'Resistor 56 Ohm', '1', '1/4W, for green LED', '$0.05'],
    ['6', 'Resistor 68 Ohm', '2', '1/4W, for red LEDs', '$0.05'],
    ['7', 'Resistor 100K Ohm', '2', '1/4W, battery voltage divider', '$0.05'],
    ['8', 'LiPo Battery', '1', '3.7V 250mAh (502030 pouch cell)', '$4.00'],
    ['9', 'Slide Switch (SPDT)', '1', 'Power on/off', '$0.30'],
    ['10', 'Perfboard', '1', '2x3cm or Seeed XIAO ProtoBoard', '$1.50'],
    ['11', 'Female Pin Headers', '1', '7-pin x 2 strips (for XIAO socket)', '$0.30'],
    ['12', 'Hookup Wire (30AWG)', '~15cm', 'Silicone or Kynar', '$0.10'],
]

pdf.set_text_color(0, 0, 0)
pdf.set_font('Helvetica', '', 9)
for i, row in enumerate(bom):
    fill = i % 2 == 0
    if fill:
        pdf.set_fill_color(240, 240, 240)
    for j, cell in enumerate(row):
        align = 'C' if j in [0, 2, 4] else 'L'
        pdf.cell(col_w[j], 7, cell, border=1, align=align, fill=fill)
    pdf.ln()

# Total row
pdf.set_font('Helvetica', 'B', 10)
pdf.set_fill_color(220, 255, 220)
pdf.cell(col_w[0] + col_w[1] + col_w[2] + col_w[3], 8, 'TOTAL', border=1, align='R', fill=True)
pdf.cell(col_w[4], 8, '$15.55', border=1, align='C', fill=True)
pdf.ln(15)

# Optional components
pdf.set_font('Helvetica', 'B', 12)
pdf.cell(0, 8, 'Optional Components', new_x="LMARGIN", new_y="NEXT")
pdf.ln(2)

opt_col = [55, 80, 25]
opt_headers = ['Component', 'Purpose', 'Est. Cost']

pdf.set_font('Helvetica', 'B', 10)
pdf.set_fill_color(40, 40, 40)
pdf.set_text_color(255, 255, 255)
for i, h in enumerate(opt_headers):
    pdf.cell(opt_col[i], 8, h, border=1, align='C', fill=True)
pdf.ln()

optional = [
    ['JST-PH 2-pin connector', 'Clean battery disconnect', '$0.50'],
    ['Heat shrink tubing', 'Waterproofing connections', '$1.00'],
    ['Conformal coating spray', 'Weather/moisture protection', '$8.00'],
    ['Picatinny rail clamp', 'Scope mount (3D print or aluminum)', 'Varies'],
]

pdf.set_text_color(0, 0, 0)
pdf.set_font('Helvetica', '', 9)
for i, row in enumerate(optional):
    fill = i % 2 == 0
    if fill:
        pdf.set_fill_color(240, 240, 240)
    for j, cell in enumerate(row):
        align = 'C' if j == 2 else 'L'
        pdf.cell(opt_col[j], 7, cell, border=1, align=align, fill=fill)
    pdf.ln()

# Wiring section
pdf.ln(10)
pdf.set_font('Helvetica', 'B', 12)
pdf.cell(0, 8, 'Wiring Diagram', new_x="LMARGIN", new_y="NEXT")
pdf.ln(2)

pdf.set_font('Courier', '', 9)
wiring = [
    'XIAO Pin    ->  Component',
    '---------------------------------------',
    'D4 (SDA)    ->  ADXL345 SDA',
    'D5 (SCL)    ->  ADXL345 SCL',
    'D1          ->  56 Ohm -> Green LED -> GND',
    'D2          ->  68 Ohm -> Red Left  -> GND',
    'D3          ->  68 Ohm -> Red Right -> GND',
    'A0          ->  Voltage divider midpoint',
    '               (100K from BAT+, 100K to GND)',
    '3V3         ->  ADXL345 VCC',
    'GND         ->  ADXL345 GND, LEDs, divider',
    'BAT pads    ->  LiPo+ (via slide switch)',
]

for line in wiring:
    pdf.cell(0, 5, line, new_x="LMARGIN", new_y="NEXT")

# Notes
pdf.ln(10)
pdf.set_font('Helvetica', 'B', 12)
pdf.cell(0, 8, 'Notes', new_x="LMARGIN", new_y="NEXT")
pdf.ln(2)
pdf.set_font('Helvetica', '', 9)
notes = [
    '- XIAO SAMD21 has built-in LiPo charger on BAT pads (charges via USB-C)',
    '- Voltage divider: 100K + 100K gives ratio of 2.0 (set BATT_DIVIDER = 2.0)',
    '- Hold device level during first 2 seconds after power-on for calibration',
    '- Set DEBUG_MODE = false in firmware for production (removes serial output)',
    '- Total current draw: ~15mA active, <0.5mA in sleep mode',
    '- Estimated battery life with 250mAh: 12-16 hours continuous, weeks with sleep',
]
for note in notes:
    pdf.cell(0, 5, note, new_x="LMARGIN", new_y="NEXT")

# Save
output_path = os.path.join(os.path.dirname(__file__), 'NIRINIUM_ScopeLevel_BOM.pdf')
pdf.output(output_path)
print(f"BOM PDF saved to: {output_path}")
