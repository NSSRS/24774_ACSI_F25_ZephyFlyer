#!/usr/bin/env python3
"""
Generate C code for ESO gain matrix L.
Run this whenever you retune the observer.
"""

import numpy as np
from design_L import compute_L
from eso import AttitudeESO

def generate_c_code(L, output_file="eso_gains.h"):
    """
    Generate C header file with L matrix.
    
    Args:
        L: 12x6 numpy array (discrete-time observer gain)
        output_file: path to output .h file
    """
    
    with open(output_file, 'w') as f:
        f.write("// Auto-generated ESO gain matrix\n")
        f.write("// DO NOT EDIT MANUALLY - regenerate with generate_L_for_c.py\n\n")
        f.write("#ifndef ESO_GAINS_H\n")
        f.write("#define ESO_GAINS_H\n\n")
        
        f.write("// Observer gain matrix L (12x6)\n")
        f.write("// Computed at hover with:\n")
        f.write(f"//   Sample time: {eso.Ts} s\n")
        f.write(f"//   Mass: {eso.m} kg\n")
        f.write(f"//   Max gain: {np.max(np.abs(L)):.3f}\n\n")
        
        f.write("static const float ESO_L[12][6] = {\n")
        for i in range(12):
            row_str = ", ".join([f"{L[i,j]:11.6f}f" for j in range(6)])
            f.write(f"    {{{row_str}}}")
            if i < 11:
                f.write(",\n")
            else:
                f.write("\n")
        f.write("};\n\n")
        f.write("#endif // ESO_GAINS_H\n")
    
    print(f"✓ Generated {output_file}")
    print(f"  Max gain: {np.max(np.abs(L)):.3f}")
    print(f"  Min gain: {np.min(np.abs(L)):.6f}")


if __name__ == "__main__":
    # Create ESO instance
    Ts = 0.001  # 1kHz
    mass = 0.027  # kg
    
    # Dummy L for initialization (will be replaced)
    L_dummy = np.zeros((12, 6))
    eso = AttitudeESO(Ts=Ts, L=L_dummy, m=mass, g=9.81)
    
    # Compute observer gain
    print("Computing observer gain L...")
    L = compute_L(eso, mass)
    
    # Update ESO with computed gain
    eso.L = L
    
    # Generate C header file
    generate_c_code(L, "eso_gains.h")
    
    print("\nUsage in your C code:")
    print("  1. #include \"eso_gains.h\"")
    print("  2. Remove the static float ESO_L[12][6] = {{...}} declaration")
    print("  3. The header provides ESO_L automatically")
