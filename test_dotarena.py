import pytest
import _dotarena
import os
import numpy as np

def test_dotarena_init():
    size = (100.0, 100.0, 10.0)
    friction = 0.1
    try:
        da = _dotarena.DotArena(size, friction)
    except Exception as e:
        pytest.fail(f"DotArena 初始化失敗: {e}")

def test_add_circle():
    size = (500.0, 500.0, 0.0)
    da = _dotarena.DotArena(size, 0.0)
    try:
        da.add_circle(1, 10.5, (0.0, 0.0, 0.0), (1.0, 1.0, 0.0))
        da.add_circle(2, 5.0, (10.0, 10.0, 0.0), (-1.0, 0.0, 0.0))
    except Exception as e:
        pytest.fail(f"add_circle 執行失敗: {e}")

def test_check_collision_loop():
    size = (100.0, 100.0, 100.0)
    da = _dotarena.DotArena(size, 0.0)
    
    da.add_circle(1, 10.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    da.add_circle(2, 10.0, (5.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
    da.add_circle(3, 10.0, (2.5, 2.5, 0.0), (0.0, -1.0, 0.0))
    
    try:
        da.checkCollisionGrid1D()
    except Exception as e:
        pytest.fail(f"checkCollisionGrid1D 執行時崩潰: {e}")

def test_render_gif():
    size = (1000.0, 1000.0, 1000.0)
    da = _dotarena.DotArena(size, 0.5)
    
    da.add_random_circles(5)
    
    renderer = _dotarena.Renderer(200, 200)
    try:
        renderer.renderToGif(da, "gif/test.gif", 5, 0.1)
        assert os.path.exists("gif/test.gif")
    except Exception as e:
        pytest.fail(f"renderToGif 執行失敗: {e}")
    finally:
        if os.path.exists("gif/test.gif"):
            os.remove("gif/test.gif")

def test_boundary_bounce():
    """Test if a circle correctly bounces off the arena walls."""
    size = (100.0, 100.0, 100.0) # Arena is from -50 to 50
    da = _dotarena.DotArena(size, 0.0)
    
    # Place circle near the right wall moving right
    # pos_x = 40, r = 5. Wall is at 50. Collision happens at 45.
    da.add_circle(1, 5.0, (40.0, 0.0, 0.0), (20.0, 0.0, 0.0))
    
    # Step 1: dt = 0.5. Circle moves to 40 + 10 = 50. 
    # But wall is at 50, radius is 5, so max allowed is 45.
    # It should bounce and reverse velocity.
    da.step(0.5)
    
    # Check velocity reversed
    assert da.vx[0] == -20.0, "Circle should have bounced off the X wall and reversed velocity."
    assert da.px[0] <= 45.0, "Circle position should be clamped inside the arena."

def test_zero_copy_memory_identity():
    """Test that the NumPy buffer protocol shares memory with the C++ engine without copying."""
    da = _dotarena.DotArena((100.0, 100.0, 10.0), 0.0)
    da.add_circle(1, 10.0, (15.0, 20.0, 25.0), (0.0, 0.0, 0.0))
    
    # Get numpy array directly from C++
    px_array = da.px
    assert px_array[0] == 15.0
    
    # Mutate the numpy array in Python
    px_array[0] = 99.9
    
    # Verify the C++ engine immediately sees the change via a new fetch
    assert da.px[0] == 99.9, "C++ Engine and Python NumPy did not share the exact same memory!"

def test_circle_collision_physics():
    """Test that two circles colliding transfer momentum (change velocity)."""
    da = _dotarena.DotArena((100.0, 100.0, 100.0), 0.0)
    
    # Two circles moving directly at each other
    da.add_circle(1, 10.0, (-10.0, 0.0, 0.0), (5.0, 0.0, 0.0))
    da.add_circle(2, 10.0, (10.0, 0.0, 0.0), (-5.0, 0.0, 0.0))
    
    # Check initial
    assert da.vx[0] == 5.0
    assert da.vx[1] == -5.0
    
    # Step forward so they collide
    da.step(0.5, method=2) # 1D Grid
    
    # After collision, they should bounce back
    assert da.vx[0] < 0.0, "Circle 1 should bounce left after collision."
    assert da.vx[1] > 0.0, "Circle 2 should bounce right after collision."

def test_brute_vs_grid_parity():
    """Test that the NEON Brute Force and NEON 1D Grid produce identical results."""
    # Setup Engine A (Brute Force)
    da_brute = _dotarena.DotArena((1000.0, 1000.0, 1000.0), 0.1)
    da_brute.add_circle(1, 10.0, (0.0, 0.0, 0.0), (5.0, 0.0, 0.0))
    da_brute.add_circle(2, 10.0, (15.0, 0.0, 0.0), (-5.0, 0.0, 0.0))
    
    # Setup Engine B (1D Grid)
    da_grid = _dotarena.DotArena((1000.0, 1000.0, 1000.0), 0.1)
    da_grid.add_circle(1, 10.0, (0.0, 0.0, 0.0), (5.0, 0.0, 0.0))
    da_grid.add_circle(2, 10.0, (15.0, 0.0, 0.0), (-5.0, 0.0, 0.0))
    
    # Step both
    da_brute.step(1.0, method=0) # 0 = Brute
    da_grid.step(1.0, method=2)  # 2 = 1D Grid
    
    # Compare results
    np.testing.assert_allclose(da_brute.px, da_grid.px, err_msg="X Positions mismatch between algorithms")
    np.testing.assert_allclose(da_brute.vx, da_grid.vx, err_msg="X Velocities mismatch between algorithms")