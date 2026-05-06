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
def test_collision_detection():
    """測試碰撞偵測邏輯"""
    size = (100.0, 100.0, 100.0)
    da = _dotarena.DotArena(size, 1.0, 0.0)
    
    # 建立兩個重疊的圓 (半徑各為 5，距離只有 8 < 10)
    c1 = _dotarena.Circle(1, 5.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    c2 = _dotarena.Circle(2, 5.0, (8.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    
    # 建立兩個分開的圓 (距離 12 > 10)
    c3 = _dotarena.Circle(3, 5.0, (20.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    
    assert da.isCollision(c1, c2) == True, "c1 和 c2 應該發生碰撞"
    assert da.isCollision(c1, c3) == False, "c1 和 c3 不應該碰撞"

def test_resolve_collision():
    """測試碰撞後的物理反應（方向/速度改變）"""
    size = (100.0, 100.0, 100.0)
    da = _dotarena.DotArena(size, 1.0, 0.0)
    
    # 準備兩個迎面對撞的圓
    # c1 向右 (1,0,0), c2 向左 (-1,0,0)
    pos1, dir1 = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    pos2, dir2 = (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)
    
    c1 = _dotarena.Circle(1, 1.0, pos1, dir1)
    c2 = _dotarena.Circle(2, 1.0, pos2, dir2)
    
    # 執行碰撞處理
    da.resolveCollision(c1, c2)
    
    # 驗證方向是否改變 (簡單彈性碰撞後，方向應該反轉或改變)
    new_dir1 = c1.getDir()
    new_dir2 = c2.getDir()
    
    assert new_dir1[0] < 1.0, "c1 碰撞後 X 軸速度應該減小或反轉"
    assert new_dir2[0] > -1.0, "c2 碰撞後 X 軸速度應該增加或反轉"

def test_check_collision_loop():
    """測試 DotArena 內部的批量碰撞檢查"""
    size = (100.0, 100.0, 100.0)
    da = _dotarena.DotArena(size, 1.0, 0.0)
    
    # 加入一堆擠在一起的圓形
    da.add_circle(1, 10.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    da.add_circle(2, 10.0, (5.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
    da.add_circle(3, 10.0, (2.5, 2.5, 0.0), (0.0, -1.0, 0.0))
    
    # 只要執行不崩潰且能跑完循環即通過基本測試
    try:
        da.checkCollision()
    except Exception as e:
        pytest.fail(f"checkCollision 執行時崩潰: {e}")

def test_no_collision_when_moving_away():
    """測試如果兩圓已經在遠離，不應觸發衝量(Impulse)"""
    size = (100.0, 100.0, 100.0)
    da = _dotarena.DotArena(size, 1.0, 0.0)
    
    # 雖然位置重疊，但 c1 往左，c2 往右 (正在分離)
    c1 = _dotarena.Circle(1, 10.0, (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
    c2 = _dotarena.Circle(2, 10.0, (5.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    
    original_dir1 = c1.getDir()
    da.resolveCollision(c1, c2)
    
    assert c1.getDir() == original_dir1, "正在遠離的圓形不應受到碰撞衝量影響"

def test_render_gif():
    """測試生成 3D GIF 動畫"""
    size = (1000.0, 1000.0, 1000.0)
    da = _dotarena.DotArena(size, 0.0, 0.5) # friction = 0.5
    
    # 參數：id, radius, (x,y,z) pos, (vx,vy,vz) dir
    # 放置幾個不同深度的圓球，讓它們互相碰撞
    da.add_circle(1, 50.0, (-200.0, 0.0, 0.0), (200.0, 0.0, 0.0))    # 從左向右
    da.add_circle(2, 70.0, (200.0, 50.0, 100.0), (-150.0, 0.0, -50.0)) # 從右向左，稍微靠後
    da.add_circle(3, 40.0, (0.0, -200.0, 50.0), (0.0, 200.0, 0.0))   # 從下向上
    da.add_circle(4, 30.0, (0.0, 200.0, -50.0), (0.0, -150.0, 50.0))  # 從上向下
    
    try:
        # 生成 100 幀，每幀 dt=0.05
        da.renderToGif("test_simulation.gif", 100, 0.05, 800, 800)
        print("\n成功生成 test_simulation.gif 動畫！")
    except Exception as e:
        pytest.fail(f"renderToGif 執行時崩潰: {e}")
    
if __name__ == "__main__":
    # 讓你可以直接用 python test_dotarena.py 執行
    test_dotarena_init()
    test_add_circle()
    test_render_gif()
    print("所有手動測試通過！")