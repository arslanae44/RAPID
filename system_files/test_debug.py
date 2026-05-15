import sys
import os

# Fix sys.path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import planform_opti

disp = planform_opti.WingDisplay()
print("Default WingDisplay.verbose:", getattr(disp, 'verbose', 'N/A'))
