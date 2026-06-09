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

def test_momentum_conservation():
    """Test that linear momentum is conserved in 3D elastic collisions (friction=0)."""
    da = _dotarena.DotArena((500.0, 500.0, 500.0), 0.0) # Zero friction
    
    # Add three circles moving towards each other
    da.add_circle(1, 10.0, (-30.0, 0.0, 0.0), (20.0, 5.0, -2.0))
    da.add_circle(2, 15.0, (30.0, 0.0, 0.0), (-15.0, -2.0, 4.0))
    da.add_circle(3, 12.0, (0.0, 40.0, 0.0), (0.0, -30.0, 0.0))
    
    # Calculate masses (mass = r^3)
    r1, r2, r3 = 10.0, 15.0, 12.0
    m1, m2, m3 = r1**3, r2**3, r3**3
    
    # Initial momentum
    init_px = m1 * da.vx[0] + m2 * da.vx[1] + m3 * da.vx[2]
    init_py = m1 * da.vy[0] + m2 * da.vy[1] + m3 * da.vy[2]
    init_pz = m1 * da.vz[0] + m2 * da.vz[1] + m3 * da.vz[2]
    
    # Run multiple steps to trigger multiple collisions
    for _ in range(50):
        da.step(0.02, method=2) # 1D grid
        
    # Final momentum
    final_px = m1 * da.vx[0] + m2 * da.vx[1] + m3 * da.vx[2]
    final_py = m1 * da.vy[0] + m2 * da.vy[1] + m3 * da.vy[2]
    final_pz = m1 * da.vz[0] + m2 * da.vz[1] + m3 * da.vz[2]
    
    # Verify conservation within numerical limits
    np.testing.assert_allclose(final_px, init_px, rtol=1e-10, atol=1e-10)
    np.testing.assert_allclose(final_py, init_py, rtol=1e-10, atol=1e-10)
    np.testing.assert_allclose(final_pz, init_pz, rtol=1e-10, atol=1e-10)

def test_energy_conservation():
    """Test that kinetic energy is conserved in 3D elastic collisions (friction=0)."""
    da = _dotarena.DotArena((500.0, 500.0, 500.0), 0.0) # Zero friction
    
    da.add_circle(1, 10.0, (-20.0, 0.0, 0.0), (30.0, 0.0, 0.0))
    da.add_circle(2, 10.0, (20.0, 0.0, 0.0), (-30.0, 0.0, 0.0))
    da.add_circle(3, 8.0, (0.0, -20.0, 0.0), (0.0, 25.0, 10.0))
    
    # Calculate masses (mass = r^3)
    r1, r2, r3 = 10.0, 10.0, 8.0
    m1, m2, m3 = r1**3, r2**3, r3**3
    
    def get_kinetic_energy():
        ke = 0.0
        for i, m in enumerate([m1, m2, m3]):
            v_sq = da.vx[i]**2 + da.vy[i]**2 + da.vz[i]**2
            ke += 0.5 * m * v_sq
        return ke
        
    init_ke = get_kinetic_energy()
    
    # Advance until they collide
    for _ in range(30):
        da.step(0.01, method=2)
        
    final_ke = get_kinetic_energy()
    
    # Verify kinetic energy conservation
    np.testing.assert_allclose(final_ke, init_ke, rtol=1e-10, atol=1e-10)

def test_all_boundary_bounces():
    """Test that circles correctly bounce off all 6 box boundaries (+/- X, Y, Z)."""
    size = (100.0, 100.0, 100.0) # Boundaries at +/- 50.0
    da = _dotarena.DotArena(size, 0.0)
    
    # Add particles near walls moving outward
    da.add_circle(1, 5.0, (44.0, 0.0, 0.0), (20.0, 0.0, 0.0))    # +X wall at 50, radius 5 => bounce point at 45
    da.add_circle(2, 5.0, (-44.0, 0.0, 0.0), (-20.0, 0.0, 0.0))  # -X wall at -50
    da.add_circle(3, 5.0, (0.0, 44.0, 0.0), (0.0, 20.0, 0.0))    # +Y wall
    da.add_circle(4, 5.0, (0.0, -44.0, 0.0), (0.0, -20.0, 0.0))  # -Y wall
    da.add_circle(5, 5.0, (0.0, 0.0, 44.0), (0.0, 0.0, 20.0))    # +Z wall
    da.add_circle(6, 5.0, (0.0, 0.0, -44.0), (0.0, 0.0, -20.0))  # -Z wall
    
    da.step(0.5) # Should trigger bounces
    
    # Assert velocities are reversed
    assert da.vx[0] == -20.0
    assert da.vx[1] == 20.0
    assert da.vy[2] == -20.0
    assert da.vy[3] == 20.0
    assert da.vz[4] == -20.0
    assert da.vz[5] == 20.0
    
    # Assert clamped inside box
    assert np.all(da.px <= 45.0) and np.all(da.px >= -45.0)
    assert np.all(da.py <= 45.0) and np.all(da.py >= -45.0)
    assert np.all(da.pz <= 45.0) and np.all(da.pz >= -45.0)

def test_division_by_zero_safety():
    """Test safety and stability when circles overlap at exactly the same coordinates (dist=0)."""
    da = _dotarena.DotArena((100.0, 100.0, 100.0), 0.0)
    da.add_circle(1, 10.0, (0.0, 0.0, 0.0), (5.0, 0.0, 0.0))
    da.add_circle(2, 10.0, (0.0, 0.0, 0.0), (-5.0, 0.0, 0.0))
    
    # Should not crash or produce NaNs/Infs
    try:
        da.step(0.1, method=2)
        assert not np.isnan(da.vx).any()
        assert not np.isinf(da.vx).any()
    except ZeroDivisionError:
        pytest.fail("Physics engine crashed with division by zero on identical particle positions")

def test_asymmetric_mass_collision():
    """Test collision kinetics with highly asymmetric masses (heavy vs light particle)."""
    da = _dotarena.DotArena((200.0, 200.0, 200.0), 0.0)
    
    # Large radius ratio implies massive weight difference (m = r^3)
    # heavy_r = 20.0 => heavy_m = 8000
    # light_r = 2.0  => light_m = 8
    da.add_circle(1, 20.0, (-30.0, 0.0, 0.0), (5.0, 0.0, 0.0)) # Heavy moving right
    da.add_circle(2, 2.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))     # Light stationary
    
    # Step until they collide
    for _ in range(10):
        da.step(1.0, method=2)
        
    # After collision:
    # Heavy should barely slow down, light should gain high velocity
    assert da.vx[0] > 4.9, "Heavy particle should maintain almost all its velocity"
    assert da.vx[1] > 8.0, "Light particle should be propelled forward at high velocity"

def test_large_scale_random_parity():
    """Verify parity of brute force and grid hash algorithms under randomized scenarios."""
    np.random.seed(42)
    da_brute = _dotarena.DotArena((1000.0, 1000.0, 1000.0), 0.05)
    da_grid = _dotarena.DotArena((1000.0, 1000.0, 1000.0), 0.05)
    
    # Generate 100 particles
    for i in range(100):
        r = np.random.uniform(5.0, 15.0)
        pos = tuple(np.random.uniform(-400.0, 400.0, size=3))
        vel = tuple(np.random.uniform(-50.0, 50.0, size=3))
        da_brute.add_circle(i, r, pos, vel)
        da_grid.add_circle(i, r, pos, vel)
        
    # Simulate for 50 steps
    for _ in range(50):
        da_brute.step(0.05, method=0) # Brute force
        da_grid.step(0.05, method=2)  # 1D Grid Hashing
        
    # Verify results are identical
    np.testing.assert_allclose(da_brute.px, da_grid.px, rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(da_brute.py, da_grid.py, rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(da_brute.pz, da_grid.pz, rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(da_brute.vx, da_grid.vx, rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(da_brute.vy, da_grid.vy, rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(da_brute.vz, da_grid.vz, rtol=1e-9, atol=1e-9)

def test_friction_deceleration_decay():
    """Verify that friction deceleration decays velocity exponentially according to the exact math formula."""
    friction = 0.25
    dt = 0.1
    da = _dotarena.DotArena((1000.0, 1000.0, 1000.0), friction)
    da.add_circle(1, 10.0, (0.0, 0.0, 0.0), (100.0, 200.0, 300.0))
    
    # Mathematical prediction:
    # frictionFactor = max(0.0, 1.0 - friction * dt) = max(0.0, 1.0 - 0.25 * 0.1) = 0.975
    # vx_step1 = 100 * 0.975 = 97.5
    # vy_step1 = 200 * 0.975 = 195.0
    # vz_step1 = 300 * 0.975 = 292.5
    da.step(dt, method=2)
    
    np.testing.assert_allclose(da.vx[0], 97.5, rtol=1e-10)
    np.testing.assert_allclose(da.vy[0], 195.0, rtol=1e-10)
    np.testing.assert_allclose(da.vz[0], 292.5, rtol=1e-10)
    
    # Step again
    # vx_step2 = 97.5 * 0.975 = 95.0625
    da.step(dt, method=2)
    np.testing.assert_allclose(da.vx[0], 95.0625, rtol=1e-10)

def test_elasticity_velocity_swap():
    """Test that two equal-mass particles colliding head-on swap their velocities (characteristic of e=1.0 elastic collision)."""
    da = _dotarena.DotArena((100.0, 100.0, 100.0), 0.0) # No friction
    # Particle 1: mass=1000, moving right at 10.0
    da.add_circle(1, 10.0, (-12.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    # Particle 2: mass=1000, moving left at -5.0
    da.add_circle(2, 10.0, (5.0, 0.0, 0.0), (-5.0, 0.0, 0.0))
    
    # Step simulation to trigger collision (dt=0.5, movement = 5.0 and -2.5, overlapping occurs)
    da.step(0.5, method=2)
    
    # Elastic collision for equal mass particles in 1D results in swapping velocities:
    # v1_final should be -5.0, v2_final should be 10.0
    np.testing.assert_allclose(da.vx[0], -5.0, rtol=1e-10)
    np.testing.assert_allclose(da.vx[1], 10.0, rtol=1e-10)

def test_newton_first_law_free_motion():
    """Test that particles in free motion (no collisions/boundaries) travel in a straight line with decayed speed."""
    friction = 0.1
    dt = 0.5
    da = _dotarena.DotArena((1000.0, 1000.0, 1000.0), friction)
    
    # Initial position (0,0,0), velocity (10, 20, 30)
    da.add_circle(1, 5.0, (0.0, 0.0, 0.0), (10.0, 20.0, 30.0))
    
    # Step 1:
    # 1. Decay velocity: v = v * (1 - 0.1 * 0.5) = v * 0.95 => (9.5, 19.0, 28.5)
    # 2. Update position: p = p_init + v_decayed * dt = (0.0, 0.0, 0.0) + (9.5, 19.0, 28.5) * 0.5 = (4.75, 9.5, 14.25)
    da.step(dt, method=2)
    
    np.testing.assert_allclose(da.vx[0], 9.5, rtol=1e-10)
    np.testing.assert_allclose(da.vy[0], 19.0, rtol=1e-10)
    np.testing.assert_allclose(da.vz[0], 28.5, rtol=1e-10)
    
    np.testing.assert_allclose(da.px[0], 4.75, rtol=1e-10)
    np.testing.assert_allclose(da.py[0], 9.5, rtol=1e-10)
    np.testing.assert_allclose(da.pz[0], 14.25, rtol=1e-10)