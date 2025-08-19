import numpy as np
from scipy.interpolate import CubicSpline, splprep, splev
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 1. 生成曲线测试数据（非直线）
def generate_curve_data(num_segments=5, points_per_segment=10):
    """生成连接的空间曲线段（使用正弦函数构造曲线）"""
    np.random.seed(42)
    segments = []
    last_point = np.array([0, 0, 0])
    
    for i in range(num_segments):
        # 随机生成曲线段方向
        direction = np.random.uniform(-1, 1, 3)
        direction /= np.linalg.norm(direction)
        
        t = np.linspace(0, 1, points_per_segment)
        x = last_point[0] + t * (i+1) + 0.3 * np.sin(2*np.pi*t)
        y = last_point[1] + t * (i+1) * direction[1] + 0.3 * np.cos(2*np.pi*t)
        z = last_point[2] + t * (i+1) * direction[2] + 0.2 * np.sin(4*np.pi*t)
        
        segment = np.vstack([x, y, z]).T
        segments.append(segment)
        last_point = segment[-1]
    
    return segments

# 2. 强制拟合（严格通过所有点）
def strict_fit(curve_segments):
    """分段三次样条拟合（保证C2连续性）"""
    all_points = np.concatenate(curve_segments)
    t = np.linspace(0, 1, len(all_points))
    
    # 每个维度单独插值
    cs_x = CubicSpline(t, all_points[:, 0], bc_type='clamped')
    cs_y = CubicSpline(t, all_points[:, 1], bc_type='clamped')
    cs_z = CubicSpline(t, all_points[:, 2], bc_type='clamped')
    
    # 生成拟合曲线
    t_new = np.linspace(0, 1, 200)
    fitted_curve = np.vstack([cs_x(t_new), cs_y(t_new), cs_z(t_new)]).T
    
    return fitted_curve

def plot_compare(segments, fitted_curve, title):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制原始线段
    for seg in segments:
        ax.plot(seg[:, 0], seg[:, 1], seg[:, 2], 'ro-', 
                markersize=5, linewidth=1, alpha=0.5)
    
    # 绘制拟合曲线
    ax.plot(fitted_curve[:, 0], fitted_curve[:, 1], fitted_curve[:, 2],
            'b-', linewidth=3, label='Fitted Curve')
    
    ax.set_title(title)
    ax.legend()
    plt.show()

# 主程序
if __name__ == "__main__":
    # 生成测试数据（曲线段）
    curve_segments = generate_curve_data()
    
    # 强制拟合
    strict_curve = strict_fit(curve_segments)
    
    # 可视化
    plot_compare(curve_segments, strict_curve, 
                "Strict Fitting (Passes Through All Points)")