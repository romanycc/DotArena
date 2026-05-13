import _dotarena
import time
import json

def benchmark_collisions(num_circles, frames, use_grid):
    size = (1000.0, 1000.0, 1000.0)
    friction = 0.1
    da = _dotarena.DotArena(size, friction)
    da.add_random_circles(num_circles)
    
    start = time.time()
    for _ in range(frames):
        da.step(0.05, use_grid)
    end = time.time()
    
    return end - start

def main():
    print("========================================")
    print("   DotArena Collision Benchmark Tool")
    print("========================================\n")
    
    frames = 100
    circle_counts = [500, 1000, 2000, 3000, 10000]
    
    print(f"Running {frames} frames per test...\n")
    print(f"{'Circles':<10} | {'Brute Force (s)':<17} | {'Grid O(N) (s)':<15} | {'Speedup'}")
    print("-" * 65)
    
    for count in circle_counts:
        # Run grid
        time_grid = benchmark_collisions(count, frames, use_grid=True)
        # Run brute force
        time_brute = benchmark_collisions(count, frames, use_grid=False)
        
        speedup = time_brute / time_grid if time_grid > 0 else 0
        print(f"{count:<10} | {time_brute:<17.4f} | {time_grid:<15.4f} | {speedup:.2f}x faster")

if __name__ == "__main__":
    main()
