import numpy as np
from scipy.integrate import odeint

class ThreeAxleBicycleModel:
    def __init__(self):

        self.sprung_mass = 4455 #簧上质量[kg]
        self.load_mass = 5000 #货物质量[kg]
        self.unsprung_mass = 350*3 #簧下质量[kg]
        self.Iw = 14*2 + 10 #等效轮胎转动惯量[kg·m²]
        self.sprung_Iz = 34823.2 #簧上转动惯量[kg·m²]
        self.load_Iz = 5416.67 #簧上货物转动惯量[kg·m²]
        
        self.m = self.sprung_mass + self.unsprung_mass + self.load_mass # 质量 [kg]

        self.g = 9.81
        self.sprung_c = 1.250 #簧上质量质心位置[m]
        self.load_c = 4 #货物质心位置[m]
        self.axis1 = 0 #轴1位置[m]
        self.axis2 = 3.250 #轴2位置[m]
        self.axis3 = 6.500 #轴3位置[m]
        
        self.total_c = (self.sprung_mass*self.sprung_c + \
                      self.unsprung_mass/3 * (self.axis1+self.axis2+self.axis3) +\
                      self.load_mass * self.load_c)/self.m #总的等效质心位置[m]
        
        self.Iz =  self.sprung_Iz + self.sprung_mass*(self.sprung_c - self.total_c)**2 +\
                   self.load_Iz + self.load_mass*(self.load_c - self.total_c)**2
                  # 横摆惯量 [kg·m²]

        # 几何参数（各轴到质心距离）
        self.lf = self.total_c - self.axis1    # 前轴 [m](在质心前为正)
        self.lm = self.axis2 - self.total_c    # 中轴 [m](在质心后为正)
        self.lr = self.axis3 - self.total_c    # 后轴 [m](在质心后为正)
        
        # 轮胎参数
        self.mu = 1.0   # 地面系数
        self.R = 0.51    # 轮胎半径 [m]
        self.roll_resistant = 0.0055 #滚动阻力系数
        
        
        # 轮胎特性
        self.BCD_alpha = 9.5031  ## 9.5031
        self.C_alpha = [self.BCD_alpha*self.m*self.g/3] * 3 # 各轴侧偏刚度 [N/rad]
        
        
        # 初始化状态
        self.state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [vx, vy, omega, X, Y, psi]
        
        # 轮胎力
        self.alpha = [0.0, 0.0, 0.0]
        self.Fx = [0.0, 0.0, 0.0]
        self.Fy = [0.0, 0.0, 0.0]
        self.F_longi = [0.0, 0.0, 0.0]  # 各轴纵向力 [前，中，后]
        self.F_lateral = [0.0, 0.0, 0.0]  # 各轴侧向力 [前，中，后]

    def dynamics(self, state, t, delta, torques):
        vx, vy, omega, X, Y, psi = state
        
        # 1. 驱动力计算
        Fx_drive = [t/self.R - self.roll_resistant * self.m * self.g /3 for t in torques]
        # print(Fx_drive)
        
        # 2. 侧偏角计算
        epsilon = 1e-3
        vx_safe = vx if abs(vx) > epsilon else epsilon * np.sign(vx + epsilon)

        alpha = [
            # 前轴：vy + lf*omega
            delta[0] - np.arctan2(vy + self.lf * omega, vx_safe),
            # 中轴：vy - lm*omega
            delta[1] - np.arctan2(vy - self.lm * omega, vx_safe),
            # 后轴：vy - lr*omega
            delta[2] - np.arctan2(vy - self.lr * omega, vx_safe)
        ]
        alpha = [np.clip(a, -np.pi/2, np.pi/2) for a in alpha]
        # print(delta[0]-alpha[0])
        # print(vy)
        self.alpha = alpha
        # print(f'车辆侧偏角alpha为:\n{self.alpha}')
                
        # 3. 轮胎力计算（线性模型）
        F_alpha = [c * a for c, a in zip(self.C_alpha, alpha)]  #可能需要修改一下乘以2表示一侧有一对轮子
        
        # 4. 力分解与状态记录（新增部分）
        self.F_x = []
        self.F_y = []
        self.F_longi = []
        self.F_lateral = []

        Fx_total = 0.0
        Fy_total = 0.0
        Mz_total = 0.0

        
        for i in range(3):
            # 驱动分量分解
            fx_d = Fx_drive[i] * np.cos(delta[i])
            fy_d = Fx_drive[i] * np.sin(delta[i])
            
            # 侧偏力分解
            fx_a = -F_alpha[i] * np.sin(delta[i])
            # print(F_alpha[i])
            fy_a = F_alpha[i] * np.cos(delta[i])
            
            # 合力计算
            fx_total = fx_d + fx_a
            fy_total = fy_d + fy_a
            
            # 记录各轴力（新增）
            self.Fx.append(fx_total)  
            self.Fy.append(fy_total)  
            self.F_longi.append(Fx_drive[i])  
            self.F_lateral.append(F_alpha[i])
            
            
            # 累计总力和力矩
            Fx_total += fx_total
            Fy_total += fy_total
            if i == 0:
                Mz_total += self.lf * fy_total
            else:
                Mz_total -= [self.lm, self.lr][i-1] * fy_total

        ###使用空气阻力或者滚动阻力修正###
        # Fx_total -= 150*vx
        
        # print(Fx_total)

        # 5. 动力学方程
        # dvx = (Fx_total + vy*omega) / self.m
        # dvy = (Fy_total - vx*omega) / self.m
        dvx = Fx_total/self.m + vy*omega 
        dvy = Fy_total/self.m - vx*omega 
        domega = Mz_total / self.Iz
        
        # 6. 运动学方程
        dX = vx*np.cos(psi) - vy*np.sin(psi)
        dY = vx*np.sin(psi) + vy*np.cos(psi)
        dpsi = omega
        
        return [dvx, dvy, domega, dX, dY, dpsi]

    def update(self, dt, delta, torques):
        """ 状态更新方法 """
        t_span = [0, dt]
        new_state = odeint(self.dynamics, self.state, t_span, 
                          args=(delta, torques))[-1]
        self.state = new_state
        return self.get_state()

    def get_state(self):
        """ 状态获取接口 """
        # print(self.alpha[0])
        return {
            # 位置状态
            'position': {
                'x': self.state[3],
                'y': self.state[4]
            },
            # 姿态状态
            'attitude': {
                'yaw': self.state[5]  # 航向角 psi [rad]
            },
            # 运动状态
            'velocity': {
                'longitudinal': self.state[0],  # vx [m/s]
                'lateral': self.state[1],       # vy [m/s]
                'yaw_rate': self.state[2],       # omega [rad/s]
                'lateral_center':self.state[1] - (6.5/2 - self.total_c)*self.state[2] #vy_sprung
            },
            # 轮胎力状态
            'tire_forces': {
                'front': {'Fx': self.F_longi[0], 'Fy': self.F_lateral[0]},
                'mid': {'Fx': self.F_longi[1], 'Fy': self.F_lateral[1]},
                'rear': {'Fx': self.F_longi[2], 'Fy': self.F_lateral[2]}
            },
            #轮胎测偏角
            'tire_alpha':{
                'alpha_1': self.alpha[0],
                'alpha_2': self.alpha[1],
                'alpha_3': self.alpha[2]
            }
        }



# 验证代码

def run_validation_tests():
    """运行一系列验证测试来检查模型行为"""
    print("\n" + "="*50)
    print("开始三轴车辆模型验证测试")
    print("="*50)
    
    # 创建车辆实例
    vehicle = ThreeAxleBicycleModel()
    
    # 测试1: 零输入直线行驶
    print("\n测试1: 零输入直线行驶 (应保持直线)")
    test_straight(vehicle)
    
    # 测试2: 纯转向输入
    print("\n测试2: 纯转向输入 (应产生侧向加速度)")
    test_steering_only(vehicle)
    
    # 测试3: 纯驱动输入
    print("\n测试3: 纯驱动输入 (应直线加速)")
    test_driving_only(vehicle)
    
    # 测试4: 低速转向验证（重点关注）
    print("\n测试4: 低速转向验证 (应产生明显侧偏角)")
    test_low_speed_steering(vehicle)
    
    # 测试5: 力平衡验证
    print("\n测试5: 力平衡验证 (检查侧向力计算)")
    test_force_balance(vehicle)
    
    print("\n所有测试完成!")
    print(vehicle.alpha)

def test_straight(vehicle):
    """测试1: 零输入直线行驶"""
    # 重置状态
    vehicle.state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # 零输入
    steering = [0, 0, 0]
    torques = [0, 0, 0]
    
    # 运行仿真
    states = []
    for t in np.arange(0, 5.0, 0.1):
        state = vehicle.update(0.1, steering, torques)
        states.append(state)
        
    print("  测试1通过: 车辆保持直线行驶")

def test_steering_only(vehicle):
    """测试2: 纯转向输入"""
    # 重置状态
    vehicle.state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # 固定前轮转向角
    steering = [np.deg2rad(5), 0, 0]
    torques = [0, 0, 0]
    
    # 运行仿真
    states = []
    for t in np.arange(0, 5.0, 0.1):
        state = vehicle.update(0.1, steering, torques)
        states.append(state)
    
    # 检查侧向响应
    final_vy = states[-1]['velocity']['lateral']
    final_omega = states[-1]['velocity']['yaw_rate']
    
    print(f"  最终侧向速度: {final_vy:.3f} m/s")
    print(f"  最终横摆率: {np.rad2deg(final_omega):.3f} deg/s")
    
    # 检查侧偏角
    alpha_front = np.rad2deg(states[-1]['tire_alpha']['alpha_1'] / 10000)
    print(f"  前轴侧偏角: {alpha_front:.3f} 度")

def test_driving_only(vehicle):
    """测试3: 纯驱动输入"""
    # 重置状态
    vehicle.state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # 施加驱动扭矩
    steering = [0, 0, 0]
    torques = [2000, 1500, 1000]
    
    # 运行仿真
    states = []
    for t in np.arange(0, 5.0, 0.1):
        state = vehicle.update(0.1, steering, torques)
        states.append(state)
    
    # 检查纵向加速度
    initial_vx = states[0]['velocity']['longitudinal']
    final_vx = states[-1]['velocity']['longitudinal']
    acceleration = (final_vx - initial_vx) / 5.0
    
    print(f"  纵向加速度: {acceleration:.3f} m/s²")
    print(f"  预期加速度: {sum(torques)/(vehicle.m * vehicle.R):.3f} m/s²")

def test_low_speed_steering(vehicle):
    """测试4: 低速转向验证（重点调试）"""
    # 重置为低速状态
    vehicle.state = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 2 m/s
    
    # 固定前轮转向角
    steering = [np.deg2rad(10), 0, 0]
    torques = [0, 0, 0]
    
    # 运行仿真
    states = []
    for t in np.arange(0, 5.0, 0.1):
        state = vehicle.update(0.1, steering, torques)
        states.append(state)
        
        # 获取当前状态
        vx = state['velocity']['longitudinal']
        vy = state['velocity']['lateral']
        omega = state['velocity']['yaw_rate']
        
        # 计算理论侧偏角
        lf = vehicle.lf
        arctan_val = np.arctan2(vy + lf * omega, max(abs(vx), 0.1))
        theoretical_alpha = np.rad2deg(steering[0] - arctan_val)
        
        
        # 打印调试信息
        print(f"  t={t:.1f}s: vx={vx:.2f}m/s, vy={vy:.4f}m/s, omega={np.rad2deg(omega):.4f}deg/s")
        print(f"    arctan计算值: {np.rad2deg(arctan_val):.4f}deg")
        print(f"    前轴侧向力: {state['tire_forces']['front']['Fy']:.2f}N")
    
    # 最终检查
    final_alpha = np.rad2deg(states[-1]['tire_alpha']['alpha_1'] / 10000)
    print(f"  最终前轴侧偏角: {final_alpha:.3f} 度")

def test_force_balance(vehicle):
    """测试5: 力平衡验证（重点调试）"""
    # 重置状态
    vehicle.state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # 施加转向和扭矩
    steering = [np.deg2rad(5), 0, 0]
    torques = [2000, 1500, 1000]
    
    # 单步仿真
    state = vehicle.update(0.1, steering, torques)
    
    # 获取状态变化
    vx, vy, omega = vehicle.state[0:3]
    dvx = (vx - 10.0) / 0.1
    dvy = vy / 0.1
    
    # 计算理论合力
    Fx_calculated = dvx * vehicle.m - vy * omega * vehicle.m
    Fy_calculated = dvy * vehicle.m + vx * omega * vehicle.m
    
    # 获取轮胎合力
    # print(vehicle.Fx)
    Fx_tires = sum(vehicle.Fx)
    Fy_tires = sum(vehicle.Fy)
    
    print("  力平衡检查:")
    print(f"  轮胎纵向合力: {Fx_tires:.1f} N")
    print(f"  轮胎侧向合力: {Fy_tires:.1f} N")
    print(f"  动力学方程计算的纵向合力: {Fx_calculated:.1f} N")
    print(f"  动力学方程计算的侧向合力: {Fy_calculated:.1f} N")
    
    # 打印各轴力详情
    print("\n  各轴力详情:")
    print(f"  前轴: Fx={vehicle.Fx[0]:.1f}N, Fy={vehicle.Fy[0]:.1f}N")
    print(f"  中轴: Fx={vehicle.Fx[1]:.1f}N, Fy={vehicle.Fy[1]:.1f}N")
    print(f"  后轴: Fx={vehicle.Fx[2]:.1f}N, Fy={vehicle.Fy[2]:.1f}N")
    
    # 检查侧偏刚度
    print("\n  侧偏刚度检查:")
    print(f"  前轴侧偏刚度: {vehicle.C_alpha[0]:.1f} N/rad")
    print(f"  中轴侧偏刚度: {vehicle.C_alpha[1]:.1f} N/rad")
    print(f"  后轴侧偏刚度: {vehicle.C_alpha[2]:.1f} N/rad")

# 在文件末尾添加调用
if __name__ == "__main__":
    # 运行验证测试
    run_validation_tests()
    

    








# # 使用示例（带动态输入和扩展可视化）
# if __name__ == "__main__":
#     import matplotlib.pyplot as plt
    
#     vehicle = ThreeAxleBicycleModel()
    
#     # 数据记录初始化
#     history = {
#         'time': [],
#         'vx': [], 'vy': [], 'omega': [],
#         'X': [], 'Y': [],
#         'delta': {'front': [], 'mid': [], 'rear': []},
#         'torque': {'front': [], 'mid': [], 'rear': []}
#     }
    
#     # 仿真参数
#     total_time = 15.0  # 总仿真时间
#     dt = 0.1  # 时间步长
    
#     print("开始动态输入仿真...")
#     for t in np.arange(0, total_time, dt):
#         # 动态生成控制输入
#         # 转向角设置：前轴正弦变化，后轴反相，中轴保持0
#         delta_front = np.sin(t * 2*np.pi/5) * np.deg2rad(5)  # 5秒周期的正弦波
#         steering = [delta_front, 0, -delta_front]  # 后轴反向
        
#         # 扭矩设置：各轴在基准值附近波动
#         base_torques = [2000, 1500, 1000]  # 基准扭矩
#         torque_fluctuation = [
#             200 * np.sin(t * 2*np.pi/3),  # 前轴3秒周期
#             150 * np.sin(t * 2*np.pi/4),  # 中轴4秒周期 
#             100 * np.sin(t * 2*np.pi/5)   # 后轴5秒周期
#         ]
#         torques = base_torques + torque_fluctuation
        
#         # 更新状态
#         state = vehicle.update(dt, steering, torques)
        
#         # 记录数据
#         history['time'].append(t)
#         history['vx'].append(state['velocity']['longitudinal'])
#         history['vy'].append(state['velocity']['lateral'])
#         history['omega'].append(state['velocity']['yaw_rate'])
#         history['X'].append(state['position']['x'])
#         history['Y'].append(state['position']['y'])
#         history['delta']['front'].append(np.rad2deg(steering[0]))
#         history['delta']['mid'].append(np.rad2deg(steering[1]))
#         history['delta']['rear'].append(np.rad2deg(steering[2]))
#         history['torque']['front'].append(torques[0])
#         history['torque']['mid'].append(torques[1])
#         history['torque']['rear'].append(torques[2])
    
#     # 创建可视化面板
#     plt.figure(figsize=(16, 12))
    
#     # 1. 输入信号可视化
#     plt.subplot(3, 2, 1)
#     plt.plot(history['time'], history['delta']['front'], label='Front Steering')
#     plt.plot(history['time'], history['delta']['rear'], '--', label='Rear Steering')
#     plt.title('Steering Angle Input')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Degrees')
#     plt.grid(True)
#     plt.legend()
    
#     plt.subplot(3, 2, 2)
#     plt.plot(history['time'], history['torque']['front'], label='Front Torque')
#     plt.plot(history['time'], history['torque']['mid'], label='Mid Torque')
#     plt.plot(history['time'], history['torque']['rear'], label='Rear Torque')
#     plt.title('Torque Input')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Nm')
#     plt.grid(True)
#     plt.legend()
    
#     # 2. 车辆状态可视化
#     plt.subplot(3, 2, 3)
#     plt.plot(history['time'], history['vx'], label='Vx')
#     plt.plot(history['time'], history['vy'], label='Vy')
#     plt.title('Vehicle Speed Components')
#     plt.xlabel('Time (s)')
#     plt.ylabel('m/s')
#     plt.grid(True)
#     plt.legend()
    
#     plt.subplot(3, 2, 4)
#     plt.plot(history['time'], np.rad2deg(history['omega']), 'g-')
#     plt.title('Yaw Rate')
#     plt.xlabel('Time (s)')
#     plt.ylabel('deg/s')
#     plt.grid(True)
    
#     # 3. 轨迹可视化
#     plt.subplot(3, 2, 5)
#     plt.plot(history['X'], history['Y'], 'b-', label='Path')
#     # 添加航向箭头
#     for i in range(0, len(history['X']), 30):  # 每3秒标记一个
#         psi = np.arctan2(history['Y'][min(i+1,len(history['Y'])-1)]-history['Y'][i],
#                         history['X'][min(i+1,len(history['X'])-1)]-history['X'][i])
#         plt.arrow(history['X'][i], history['Y'][i], 
#                 1.5*np.cos(psi), 1.5*np.sin(psi),
#                 head_width=0.8, fc='r', ec='r')
#     plt.title('Vehicle Trajectory')
#     plt.xlabel('X (m)')
#     plt.ylabel('Y (m)')
#     plt.axis('equal')
#     plt.grid(True)
    
#     # 4. 相空间图
#     plt.subplot(3, 2, 6)
#     plt.plot(history['vx'], history['vy'], 'm-')
#     plt.title('Velocity Phase Portrait')
#     plt.xlabel('Vx (m/s)')
#     plt.ylabel('Vy (m/s)')
#     plt.grid(True)
    
#     plt.tight_layout()
#     plt.show()