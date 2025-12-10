#!/usr/bin/env python3
"""
Simple script to print L matrix for copy-paste into C code.
Run once, copy the output, paste into your C file.
"""

import numpy as np
from design_L import compute_L
from eso import AttitudeESO

# Create ESO
Ts = 0.001  # 1kHz
mass = 0.027  # kg
L_dummy = np.zeros((12, 6))
eso = AttitudeESO(Ts=Ts, L=L_dummy, m=mass, g=9.81)

# Compute L
print("Computing observer gain L...\n")
L = compute_L(eso, mass)

# Print for copy-paste
print("\n" + "="*70)
print("COPY THIS INTO YOUR C CODE:")
print("="*70)
print("\nstatic float ESO_L[12][6] = {")
for i in range(12):
    row_str = ", ".join([f"{L[i,j]:11.6f}f" for j in range(6)])
    comma = "," if i < 11 else ""
    print(f"    {{{row_str}}}{comma}")
print("};")
print("\n" + "="*70)
print(f"Max gain: {np.max(np.abs(L)):.6f}")
print(f"Min gain: {np.min(np.abs(L[L != 0])):.6f}")
print("="*70)