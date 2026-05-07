import _dotarena
import time
import json

def main():
    # Load configuration
    with open('config.json', 'r') as f:
        config = json.load(f)
        
    print("Initialize DotArena...")
    # Create an arena
    arena_cfg = config['arena']
    size = (arena_cfg['size_x'], arena_cfg['size_y'], arena_cfg['size_z'])
    friction = arena_cfg['friction']
    da = _dotarena.DotArena(size, friction)

    # Add random circles
    sim_cfg = config['simulation']
    num_circles = sim_cfg['num_circles']
    print(f"Adding {num_circles} random circles...")
    da.add_random_circles(num_circles)

    # Generate a GIF
    frames = sim_cfg['frames']
    dt = sim_cfg['dt']
    
    out_cfg = config['output']
    filename = out_cfg['filename']
    width = out_cfg['width']
    height = out_cfg['height']
    
    print(f"Rendering {frames} frames to {filename} (this may take a moment)...")
    start_time = time.time()
    
    renderer = _dotarena.Renderer(width, height)
    renderer.renderToGif(da, filename, frames, dt)
    
    end_time = time.time()
    print(f"Successfully generated {filename} in {end_time - start_time:.2f} seconds!")

if __name__ == "__main__":
    main()
