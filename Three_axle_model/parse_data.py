import scipy.io as scio
import numpy as np
import pandas as pd

def load_vehicle_data(mat_path):
    """
    :param mat_path: MAT文件路径
    :return: 结构化字典数据，包含actuators、vehicle_dynamics和initial_conditions
    """
    try:
        # 加载原始数据（自动转换struct为字典）
        raw_data = scio.loadmat(mat_path, simplify_cells=True)
        
        # 数据架构容器
        parsed_data = {
            'actuators': {},
            'vehicle_dynamics': {},
            'initial_conditions': {}  # 新增初始条件容器
        }

        #-------------------------- 初始条件提取 --------------------------#
        # 检查是否存在initial_conditions字段
        if 'initial_conditions' in raw_data:
            # MATLAB结构体可能被转换为字典或numpy.void对象
            # 这里假设已经是简化后的字典结构
            initial_cond = raw_data['initial_conditions']
            
            # 提取目标变量（注意MATLAB变量名大小写敏感性）
            parsed_data['initial_conditions'] = {
                'wz0': initial_cond.get('wz0', None),  # 初始横摆角速度
                'vx0': initial_cond.get('vx0', None),  # 初始纵向速度
                'vy0': initial_cond.get('vy0', None),  # 初始横向速度
                'w0': initial_cond.get('w0', None)      # 初始车轮转速（可能需要确认命名）
            }
        else:
            print("警告: 未找到initial_conditions字段")

        #-------------------------- Actuator Data --------------------------#
        # Steering Angles (Unit: rad)
        parsed_data['actuators']['steer_angles'] = {
            'data': raw_data['Delta_act'],
            'columns': [
                'FL_steer', 'FR_steer',
                'ML_steer', 'MR_steer',
                'RL_steer', 'RR_steer'
            ],
            'unit': 'rad'
        }

        # Wheel Torques (Unit: Nm)
        parsed_data['actuators']['wheel_torques'] = {
            'data': raw_data['T_motor'],
            'columns': [
                'FL_trq', 'FR_trq',
                'ML_trq', 'MR_trq',
                'RL_trq', 'RR_trq'
            ],
            'unit': 'Nm'
        }

        #-------------------------- Vehicle Dynamics --------------------------#
        state_labels = [
            'Xpos', 'Ypos', 'Yaw',          # 0-2
            'Vx', 'Vy', 'YawRate',         # 3-5
            'Ax', 'Ay', 'YawAccel',         # 6-8
            *[f'WhlSpd_{pos}' for pos in ['FL', 'FR', 'ML', 'MR', 'RL', 'RR']],  # 9-14
            *[f'{force_direction}_{pos}' 
              for force_direction in ['Fx', 'Fy', 'Fz']
              for pos in ['FL', 'FR', 'ML', 'MR', 'RL', 'RR']
            ]  # 15-32
        ]

        parsed_data['vehicle_dynamics'] = pd.DataFrame(
            raw_data['Vehicle_state_trucksim_33d'],
            columns=state_labels
        )

        return parsed_data

    except Exception as e:
        print(f"数据解析失败: {str(e)}")
        return None

# 使用示例
if __name__ == "__main__":
    data_dict = load_vehicle_data('./data/trucksim/specific_control/data_v_test_1/data010.mat')
    
    # 查看初始条件
    print("\n初始条件:")
    print(data_dict['initial_conditions'])
    
    # 验证车辆状态初始值是否匹配
    first_state = data_dict['vehicle_dynamics'].iloc[0]
    print("\n车辆状态初始值:")
    print(f"Vx0 from states: {first_state['Vx']} (should be {data_dict['initial_conditions']['vx0']})")
    print(f"Vy0 from states: {first_state['Vy']} (should be {data_dict['initial_conditions']['vy0']})")