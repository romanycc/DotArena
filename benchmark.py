import _dotarena
import time
import json
import numpy as np

def benchmark_collisions(num_circles, frames, method):
    size = (1000.0, 1000.0, 1000.0)
    friction = 0.1
    da = _dotarena.DotArena(size, friction)
    da.add_random_circles(num_circles)
    
    start = time.time()
    for _ in range(frames):
        da.step(0.05, method)
    end = time.time()
    
    return end - start

def test_zero_copy():
    print("\n========================================")
    print("   Zero-Copy Buffer Protocol Proof")
    print("========================================\n")
    
    size = (1000.0, 1000.0, 1000.0)
    da = _dotarena.DotArena(size, 0.1)
    
    # 1. Add 10 Million Circles to prove O(1) fetch
    print("Adding 10,000,000 circles (takes a second)...")
    da.add_random_circles(10000000)
    
    start = time.time()
    px_array = da.px  # Fetch the massive array to Python
    end = time.time()
    
    print(f"Time to fetch 10 Million elements to Python: {(end - start)*1000:.4f} ms")
    if (end - start) < 0.01:
        print("-> SUCCESS: Fetch was O(1) Instantaneous (Zero-Copy)!")
    else:
        print("-> FAILED: Fetch was slow (Deep Copy occurred).")
        
    # 2. Prove Memory Address Identity
    print("\nProving Memory Identity...")
    original_val = px_array[0]
    print(f"Original px[0] in Python/NumPy: {original_val}")
    
    # Mutate in NumPy
    px_array[0] = 9999.99
    print("Mutated px[0] to 9999.99 directly in Python.")
    
    # Check if C++ sees it by fetching again (which returns same pointer)
    px_check = da.px
    print(f"Value read directly from C++ engine: {px_check[0]}")
    
    if px_check[0] == 9999.99:
        print("-> SUCCESS: C++ Engine and Python NumPy share the exact same memory!")
    else:
        print("-> FAILED: Memory was copied, C++ did not see the change.")
    
    print("\nDone!")

def main():
    print("========================================")
    print("   DotArena Collision Benchmark Tool")
    print("========================================\n")
    
    frames = 20
    circle_counts = [500, 1000, 2000, 3000, 10000, 100000]
    
    print(f"Running {frames} frames per test...\n")
    print(f"{'Circles':<10} | {'Brute (s)':<12} | {'2D Grid (s)':<12} | {'1D Grid (s)':<12} | {'Speedup (1D vs Brute)'}")
    print("-" * 75)
    
    for count in circle_counts:
        time_1d = benchmark_collisions(count, frames, method=2)
        time_2d = benchmark_collisions(count, frames, method=1)
        time_brute = benchmark_collisions(count, frames, method=0)
        
        speedup = time_brute / time_1d if time_1d > 0 else 0
        print(f"{count:<10} | {time_brute:<12.4f} | {time_2d:<12.4f} | {time_1d:<12.4f} | {speedup:.2f}x faster")
        
    test_zero_copy()

if __name__ == "__main__":
    main()
