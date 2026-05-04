import pytest
import _dotarena  

def test_dotarena_init():
    # 測試初始化
    size = (100.0, 100.0, 10.0)
    shrink_rate = 0.99
    friction = 0.1
    
    try:
        da = _dotarena.DotArena(size, shrink_rate, friction)
        print("\nDotArena 初始化成功！")
    except Exception as e:
        pytest.fail(f"DotArena 初始化失敗: {e}")

def test_add_circle():
    # 測試添加圓形
    size = (500.0, 500.0, 0.0)
    da = _dotarena.DotArena(size, 1.0, 0.0)
    
    # 參數：id (int), r (double), pos (tuple), dir (tuple)
    try:
        da.add_circle(1, 10.5, (0.0, 0.0, 0.0), (1.0, 1.0, 0.0))
        da.add_circle(2, 5.0, (10.0, 10.0, 0.0), (-1.0, 0.0, 0.0))
        print("成功添加兩個圓形！")
    except Exception as e:
        pytest.fail(f"add_circle 執行失敗: {e}")

if __name__ == "__main__":
    # 讓你可以直接用 python test_dotarena.py 執行
    test_dotarena_init()
    test_add_circle()
    print("所有手動測試通過！")