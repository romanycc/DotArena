import _dotarena
import time
import json

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

def main():
    print("========================================")
    print("   DotArena Collision Benchmark Tool")
    print("========================================\n")
    
    frames = 100
    circle_counts = [500, 1000, 2000, 3000]
    
    print(f"Running {frames} frames per test...\n")
    print(f"{'Circles':<10} | {'Brute (s)':<12} | {'2D Grid (s)':<12} | {'1D Grid (s)':<12} | {'Speedup (1D vs Brute)'}")
    print("-" * 75)
    
    for count in circle_counts:
        time_1d = benchmark_collisions(count, frames, method=2)
        time_2d = benchmark_collisions(count, frames, method=1)
        time_brute = benchmark_collisions(count, frames, method=0)
        
        speedup = time_brute / time_1d if time_1d > 0 else 0
        print(f"{count:<10} | {time_brute:<12.4f} | {time_2d:<12.4f} | {time_1d:<12.4f} | {speedup:.2f}x faster")

if __name__ == "__main__":
    main()
