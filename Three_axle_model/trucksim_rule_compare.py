# 新增数据预处理函数
import numpy as np
import matplotlib.pyplot as plt
from Three_axle_model_linear import ThreeAxleBicycleModel
from Three_axle_model_pacejka import ThreeAxleVehiclePacejka
from parse_data import load_vehicle_data

def preprocess_trucksim_data(parsed_data):
    """将Trucksim的6轮数据转换为3轴平均数据"""
    # 获取原始数据引用
    actuators = parsed_data['actuators']
    
    # 转角和扭矩按轴分组索引
    axis_indices = [
        (0,1),   # 前轴 FL,FR
        (2,3),    # 中轴 ML,MR
        (4,5)     # 后轴 RL,RR
    ]
    
    # 计算各轴平均转角和总扭矩
    delta_axis = []
    torque_axis = []
    for idx in axis_indices:
        # 转角取平均（注意单位是弧度）
        delta_avg = np.mean(actuators['steer_angles']['data'][:, idx], axis=1)
        delta_axis.append(delta_avg)
        
        # 扭矩求和
        torque_sum = np.sum(actuators['wheel_torques']['data'][:, idx], axis=1)
        torque_axis.append(torque_sum)
    
    # 存储处理后的控制输入
    parsed_data['control_input'] = {
        'delta': np.column_stack(delta_axis),  # 各轴转角 [前,中,后]
        'torque': np.column_stack(torque_axis) # 各轴扭矩 [前,中,后]
    }
    
    return parsed_data

# 修改模型初始化部分
def init_models_from_trucksim(trucksim_data):
    """根据Trucksim数据初始化模型"""
    # 获取初始条件
    init_cond = trucksim_data['initial_conditions']
    
    # 初始化线性模型
    linear_model = ThreeAxleBicycleModel()
    # 设置初始状态 [vx, vy, omega, X, Y, psi]
    linear_model.state = np.array([
        init_cond['vx0'], 
        init_cond['vy0'], 
        init_cond['wz0'],
        0, 0, 0  # 初始位置和航向设为0
    ])
    
    # 初始化Pacejka模型
    pacejka_model = ThreeAxleVehiclePacejka()
    # 设置初始状态 [vx, vy, omega, X, Y, psi, w1, w2, w3]
    # 计算初始轮速 (假设无滑移)
    wheel_speed = init_cond['vx0'] / pacejka_model.R  
    pacejka_model.state = np.array([
        init_cond['vx0'],
        init_cond['vy0'],
        init_cond['wz0'],
        0, 0, 0,
        wheel_speed, wheel_speed, wheel_speed
    ])
    
    return linear_model, pacejka_model

# 对比仿真主函数
def run_comparison(trucksim_data, dt):
    """运行模型对比仿真"""
    # 预处理数据
    trucksim_data = preprocess_trucksim_data(trucksim_data)
    
    # 初始化模型
    linear_model, pacejka_model = init_models_from_trucksim(trucksim_data)
    
    # 获取参考数据
    ref_time = np.arange(0, len(trucksim_data['control_input']['delta']))*dt
    ref_states = trucksim_data['vehicle_dynamics']
    
    # 结果容器
    results = {
        'time': ref_time,
        'trucksim': {
            'vx': ref_states['Vx'].values,
            'vy': ref_states['Vy'].values,
            'yaw': ref_states['Yaw'].values,
            'X': ref_states['Xpos'].values,
            'Y': ref_states['Ypos'].values,
            'Fx_FL':ref_states['Fx_FL'].values,
            'Fy_FL':ref_states['Fy_FL'].values
        },
        'linear': {'vx': [], 
                   'vy': [], 
                   'vy_center':[],
                   'yaw_rate':[],
                   'yaw': [], 
                   'X': [], 
                   'Y': [],
                   'Fx_FL':[],
                   'Fy_FL':[],
                   'alpha_1':[],
                   'alpha_2':[],
                   'alpha_3':[],

        },
        'pacejka': {'vx': [],
                     'vy': [], 
                     'vy_center':[],
                     'yaw_rate':[],
                     'yaw': [], 
                     'X': [], 
                     'Y': [],
                     'Fx_FL':[],
                     'Fy_FL':[],
        }
    }
    
    # 运行仿真循环
    for i in range(len(ref_time)):
        # 获取当前控制输入
        delta = trucksim_data['control_input']['delta'][i]
        torque = trucksim_data['control_input']['torque'][i]
        
        # 更新线性模型
        linear_state = linear_model.update(dt, delta, torque)
        results['linear']['vx'].append(linear_state['velocity']['longitudinal'])
        results['linear']['vy'].append(linear_state['velocity']['lateral'])
        results['linear']['vy_center'].append(linear_state['velocity']['lateral_center'])
        results['linear']['yaw'].append(linear_state['attitude']['yaw'])
        results['linear']['X'].append(linear_state['position']['x'])
        results['linear']['Y'].append(linear_state['position']['y'])
        results['linear']['Fx_FL'].append(linear_state['tire_forces']['front']['Fx']/2)
        results['linear']['Fy_FL'].append(linear_state['tire_forces']['front']['Fy']/2)
        results['linear']['alpha_1'].append(linear_state['tire_alpha']['alpha_1'])
        results['linear']['alpha_2'].append(linear_state['tire_alpha']['alpha_2'])
        results['linear']['alpha_3'].append(linear_state['tire_alpha']['alpha_3'])
        results['linear']['yaw_rate'].append(linear_state['velocity']['yaw_rate'])
        

        
        # 更新Pacejka模型
        pacejka_state = pacejka_model.update(dt, delta, torque)
        results['pacejka']['vx'].append(pacejka_state['velocity']['longitudinal'])
        results['pacejka']['vy'].append(pacejka_state['velocity']['lateral'])
        results['pacejka']['vy_center'].append(pacejka_state['velocity']['lateral_center'])
        results['pacejka']['yaw'].append(pacejka_state['attitude']['yaw'])
        results['pacejka']['X'].append(pacejka_state['position']['x'])
        results['pacejka']['Y'].append(pacejka_state['position']['y'])
        results['pacejka']['Fx_FL'].append(pacejka_state['wheel']['Fx'][0]/2)
        results['pacejka']['Fy_FL'].append(pacejka_state['wheel']['Fy'][0]/2)
        results['pacejka']['yaw_rate'].append(pacejka_state['velocity']['yaw_rate'])
    
    return results

# 结果可视化
def plot_comparison(results):
    plt.figure(figsize=(15, 10))
    
    # 速度分量对比
    plt.subplot(2,3,1)
    plt.plot(results['time'], results['trucksim']['vx'], 'k-', label='Trucksim')
    plt.plot(results['time'], results['linear']['vx'], 'r--', label='Linear')
    plt.plot(results['time'], results['pacejka']['vx'], 'b-.', label='Pacejka')
    plt.title('Longitudinal Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Vx (m/s)')
    plt.grid(True)
    plt.legend()
    
    plt.subplot(2,3,2)
    plt.plot(results['time'], results['trucksim']['vy'], 'k-', label='Trucksim')
    plt.plot(results['time'], results['linear']['vy_center'], 'r--', label='Linear')
    plt.plot(results['time'], results['pacejka']['vy_center'], 'b-.', label='Pacejka')
    plt.title('Lateral Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Vy (m/s)')
    plt.grid(True)
    plt.legend()
    
    # 轨迹对比
    plt.subplot(2,3,3)
    plt.plot(results['trucksim']['X'], results['trucksim']['Y'], 'k-', label='Trucksim')
    plt.plot(results['linear']['X'], results['linear']['Y'], 'r--', label='Linear')
    plt.plot(results['pacejka']['X'], results['pacejka']['Y'], 'b-.', label='Pacejka')
    plt.title('Vehicle Trajectory')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()

    #侧偏角对比
    # plt.subplot(2,3,3)
    # plt.plot(results['time'], results['linear']['alpha_1'], 'k-', label='Linear_alpha_1')
    # plt.plot(results['time'], results['linear']['alpha_2'], 'r--', label='Linear_alpha_2')
    # plt.plot(results['time'], results['linear']['alpha_3'], 'b-', label='Linear_alpha_3')
    # plt.title('Tire alpha')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Alpha (rad)')
    # plt.axis('equal')
    # plt.grid(True)
    # plt.legend()

    
    # 横摆角对比
    plt.subplot(2,3,4)
    plt.plot(results['time'], np.rad2deg(results['trucksim']['yaw']), 'k-', label='Trucksim')
    plt.plot(results['time'], np.rad2deg(results['linear']['yaw']), 'r--', label='Linear')
    plt.plot(results['time'], np.rad2deg(results['pacejka']['yaw']), 'b-.', label='Pacejka')
    plt.title('Yaw Angle')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw (deg)')
    plt.grid(True)
    plt.legend()

    # 左前轮受力对比
    plt.subplot(2,3,5)
    plt.plot(results['time'], results['trucksim']['Fx_FL'], 'k-', label='Trucksim')
    plt.plot(results['time'], results['linear']['Fx_FL'], 'r--', label='Linear')
    plt.plot(results['time'], results['pacejka']['Fx_FL'], 'b-.', label='Pacejka')
    plt.title('Fx_FL')
    plt.xlabel('Time (s)')
    plt.ylabel('Fx_FL (N)')
    plt.grid(True)
    plt.legend()


    plt.subplot(2,3,6)
    plt.plot(results['time'], results['trucksim']['Fy_FL'], 'k-', label='Trucksim')
    plt.plot(results['time'], results['linear']['Fy_FL'], 'r--', label='Linear')
    plt.plot(results['time'], results['pacejka']['Fy_FL'], 'b-.', label='Pacejka')
    plt.title('Fy_FL')
    plt.xlabel('Time (s)')
    plt.ylabel('Fy_FL (N)')
    plt.grid(True)
    plt.legend()

    
    plt.tight_layout()
    plt.show()

# 主程序
if __name__ == "__main__":
    # 加载Trucksim数据
    trucksim_data = load_vehicle_data('./data/trucksim/specific_control/data_v_test_3/data019.mat')
    
    # 运行对比仿真
    dt = 0.01  # 需要与Trucksim数据时间步长一致
    results = run_comparison(trucksim_data, dt)
    
    # 可视化结果
    plot_comparison(results)