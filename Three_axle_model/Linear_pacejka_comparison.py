import numpy as np
import matplotlib.pyplot as plt
from Three_axle_model_linear import ThreeAxleBicycleModel
from Three_axle_model_pacejka import ThreeAxleVehiclePacejka

def run_comparison():
    # 统一车辆参数 (统一为6x4卡车参数)
    # common_params = {
    #     'mass': 12000,       # kg
    #     'yaw_inertia': 45000,# kg·m²
    #     'wheel_radius': 0.5, # m
    #     'wheelbase': [2.5, 1.0, 2.0]  # [a, b, c] 前、中、后轴距
    # }
    
    # 初始化模型
    linear_model = ThreeAxleBicycleModel()
    nonlinear_model = ThreeAxleVehiclePacejka()
    
    # 调整线性模型参数
    # linear_model.m = common_params['mass']
    # linear_model.Iz = common_params['yaw_inertia']
    # linear_model.R = common_params['wheel_radius']
    # linear_model.lf = common_params['wheelbase'][0]
    # linear_model.lm = common_params['wheelbase'][1]
    # linear_model.lr = common_params['wheelbase'][2]
    
    # # 设置等效侧偏刚度 (从Pacejka参数推导)
    # C_alpha_base = [  # D_alpha*BCD_alpha (Pacejka线性区斜率)
    #     nonlinear_model.tire.p['D_alpha'][i] * nonlinear_model.tire.p['BCD_alpha'][i]
    #     for i in range(3)
    # ]
    # linear_model.C_alpha = [base * 10000 for base in C_alpha_base]
    # print(linear_model.C_alpha)
    
    # 生成控制序列
    total_time = 15.0
    dt = 0.05
    time = np.arange(0, total_time, dt)
    
    # 动态转向输入（前轴正弦，后轴反向）
    steering_angle = 5 * np.sin(2*np.pi/4 * time)  # 4秒周期
    delta_sequence = [
        np.deg2rad(steering_angle),  # 前轴
        np.zeros_like(time),         # 中轴
        -1 * np.deg2rad(steering_angle)  # 后轴
    ]
    
    # 动态扭矩输入（带相位差）
    base_torque = np.array([3000, 2000, 1500])
    torque_fluct = np.array([
        100 * np.sin(2*np.pi/2.5 * time),
        100 * np.sin(2*np.pi/3.0 * time + 0.5),
        100 * np.sin(2*np.pi/3.5 * time + 1.0)
    ])
    torque_sequence = (base_torque[:, None] + torque_fluct).T
    
    # 运行仿真
    def simulate_model(model):
        data = {'time': [], 'vx': [], 'vy': [], 'yaw_rate': [], 'X': [], 'Y': []}
        for i, t in enumerate(time):
            state = model.update(dt, 
                               [delta_sequence[0][i], delta_sequence[1][i], delta_sequence[2][i]],
                               torque_sequence[i])
            
            data['time'].append(t)
            data['vx'].append(state['velocity']['longitudinal'])
            data['vy'].append(state['velocity']['lateral'])
            data['yaw_rate'].append(np.rad2deg(state['velocity']['yaw_rate']))
            data['X'].append(state['position']['x'])
            data['Y'].append(state['position']['y'])
            # print(state['velocity']['longitudinal'])
            # print(model.Fy)
        return data
    
    print("Running linear model...")
    linear_data = simulate_model(linear_model)
    
    print("Running nonlinear model...")
    nonlinear_data = simulate_model(nonlinear_model)
    
    # 专业可视化
    plt.figure(figsize=(18, 12))
    
    # 1. 运动状态对比
    plt.subplot(2, 2, 1)
    plt.plot(linear_data['time'], linear_data['vx'], 'b--', label='Linear Vx')
    plt.plot(nonlinear_data['time'], nonlinear_data['vx'], 'r-', label='Nonlinear Vx')
    plt.plot(linear_data['time'], linear_data['vy'], 'g--', label='Linear Vy')
    plt.plot(nonlinear_data['time'], nonlinear_data['vy'], 'm-', label='Nonlinear Vy')
    plt.title('Velocity Components Comparison')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 2. 横摆响应对比
    plt.subplot(2, 2, 2)
    plt.plot(linear_data['time'], linear_data['yaw_rate'], 'b--', label='Linear')
    plt.plot(nonlinear_data['time'], nonlinear_data['yaw_rate'], 'r-', label='Nonlinear')
    plt.title('Yaw Rate Response')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Rate (deg/s)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 3. 运动轨迹对比
    plt.subplot(2, 2, 3)
    plt.plot(linear_data['X'], linear_data['Y'], 'b--', label='Linear')
    plt.plot(nonlinear_data['X'], nonlinear_data['Y'], 'r-', label='Nonlinear')
    plt.title('Trajectory Comparison')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 4. 速度相图对比
    plt.subplot(2, 2, 4)
    plt.plot(linear_data['vx'], linear_data['vy'], 'b--', label='Linear')
    plt.plot(nonlinear_data['vx'], nonlinear_data['vy'], 'r-', label='Nonlinear')
    plt.title('Velocity Phase Portrait')
    plt.xlabel('Longitudinal Speed (m/s)')
    plt.ylabel('Lateral Speed (m/s)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_comparison()