import pytest
import _dotarena
import os

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
        renderer.renderToGif(da, "test.gif", 5, 0.1)
        assert os.path.exists("test.gif")
    except Exception as e:
        pytest.fail(f"renderToGif 執行失敗: {e}")
    finally:
        if os.path.exists("test.gif"):
            os.remove("test.gif")