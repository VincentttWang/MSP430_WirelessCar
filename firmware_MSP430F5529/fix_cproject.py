import os
import glob
import re

targets = glob.glob(r'C:\Users\14437\workspace_ccstheia\*\.cproject') + [
    r'C:\Users\14437\Desktop\MSP430\第1课\LAB1\.cproject',
    r'C:\Users\14437\Desktop\MSP430\第3课\第3课\LAB2\.cproject'
]

for p in targets:
    if os.path.exists(p):
        with open(p, 'r', encoding='latin1') as f:
            c = f.read()
        c_new = re.sub(r'OPT_CODEGEN_VERSION[^\n]*?value="[^"]*"', 'OPT_CODEGEN_VERSION" value="21.6.3.LTS"', c)
        if c_new != c:
            with open(p, 'w', encoding='latin1') as f:
                f.write(c_new)
            print(f'Updated compiler version in: {p}')
        else:
            print(f'No change needed for: {p}')
