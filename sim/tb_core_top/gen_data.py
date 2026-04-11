import argparse
import sys
import random

def generate_conv2d_data(cin, cout, k, hin, win):
    # Set random seed for reproducibility
    random.seed(42)
    
    # Generate random IFM (INT8)
    # Range -128 to 127
    ifm = [[[random.randint(-128, 127) for _ in range(cin)] for _ in range(win)] for _ in range(hin)]
    
    # Generate random Weights (INT8)
    # Shape: (k, k, cout, cin)
    weights = [[[[random.randint(-128, 127) for _ in range(cin)] for _ in range(cout)] for _ in range(k)] for _ in range(k)]
    
    # Calculate OFM dimensions
    hout = hin - k + 1
    wout = win - k + 1
    
    if hout <= 0 or wout <= 0:
        print(f"Error: Output dimensions must be positive. Got hout={hout}, wout={wout}")
        sys.exit(1)
        
    # Compute Conv2D
    # OFM is accumulated in INT32
    ofm = [[[0 for _ in range(cout)] for _ in range(wout)] for _ in range(hout)]
    
    for y in range(hout):
        for x in range(wout):
            for co in range(cout):
                sum_val = 0
                for ky in range(k):
                    for kx in range(k):
                        for ci in range(cin):
                            val = ifm[y+ky][x+kx][ci] * weights[ky][kx][co][ci]
                            sum_val += val
                ofm[y][x][co] = sum_val
                
    return ifm, weights, ofm

def save_hex_files(ifm, weights, ofm, k):
    hin = len(ifm)
    win = len(ifm[0])
    cin = len(ifm[0][0])
    hout = len(ofm)
    wout = len(ofm[0])
    cout = len(ofm[0][0])
    
    # Save IFM
    with open("ifm.txt", "w") as f:
        for y in range(hin):
            for x in range(win):
                val_hex = ""
                for c in reversed(range(cin)):
                    hx = format(ifm[y][x][c] & 0xFF, '02x')
                    val_hex += hx
                f.write(val_hex + "\n")
                
    # Save Weights
    with open("weight.txt", "w") as f:
        for ky in range(k):
            for kx in range(k):
                val_hex = ""
                for co in reversed(range(cout)):
                    for c in reversed(range(cin)):
                        hx = format(weights[ky][kx][co][c] & 0xFF, '02x')
                        val_hex += hx
                f.write(val_hex + "\n")
                
    # Save OFM
    with open("ofm.txt", "w") as f:
        for y in range(hout):
            for x in range(wout):
                val_hex = ""
                for co in reversed(range(cout)):
                    hx = format(ofm[y][x][co] & 0xFFFFFFFF, '08x')
                    val_hex += hx
                f.write(val_hex + "\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cin", type=int, default=8)
    parser.add_argument("--cout", type=int, default=8)
    parser.add_argument("--k", type=int, default=3)
    parser.add_argument("--hin", type=int, default=3)
    parser.add_argument("--win", type=int, default=34)
    args = parser.parse_args()
    
    ifm, weights, ofm = generate_conv2d_data(args.cin, args.cout, args.k, args.hin, args.win)
    save_hex_files(ifm, weights, ofm, args.k)
    
    print(f"Generated data for Conv2D: {args.hin}x{args.win}x{args.cin} IFM, {args.k}x{args.k} Kernel, {args.cout} Cout.")
    print(f"Output shape: {len(ofm)}x{len(ofm[0])}x{args.cout}")
