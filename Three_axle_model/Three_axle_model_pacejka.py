import numpy as np
from scipy.integrate import odeint

class PacejkaTire:
    def __init__(self):
        # Pacejka 96模型参数（示例参数，卡车轮胎）
        self.p = {
            'C_alpha': [1.5996]*3,      # 形状因子
            'D_alpha': [0.8640]*3,      # 峰值因子
            'BCD_alpha': [9.5031]*3,       # 刚度参数
            'E_alpha': [-0.3028]*3,   # 曲率因子
            
            'C_k': [1.7753]*3,
            'D_k': [0.8644]*3,
            'BCD_k': [12.43]*3,
            'E_k': [0.5624]*3,
        }
        # 垂向载荷 14715.0 N 时:
        # C_k: 1.7753
        # D_k: 0.8644
        # BCD_k: 12.4334
        # E_k: 0.5624
        # C_alpha: 1.5996
        # D_alpha: 0.8640
        # BCD_alpha: 9.5031
        # E_alpha: -0.3028

        
    def magic_formula(self, S, C, D, BCD, E):
        """Pacejka魔法公式核心计算"""
        B = BCD / (C * D)
        return D * np.sin(C * np.arctan(B * S - E * (B * S - np.arctan(B * S))))

class ThreeAxleVehiclePacejka:
    def __init__(self):
        # 车辆基本参数

        self.sprung_mass = 4455 #簧上质量[kg]
        self.load_mass = 5000 #货物质量[kg]
        self.unsprung_mass = 350*3 #簧下质量[kg]
        self.Iw = 14*2 + 10 #等效轮胎转动惯量[kg·m²]
        self.sprung_Iz = 34823.2 #簧上转动惯量[kg·m²]
        self.load_Iz = 5416.67 #簧上转动惯量[kg·m²]
        
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
        # print(self.total_c, self.sprung_c)
        
        self.Iz =  self.sprung_Iz + self.sprung_mass*(self.sprung_c - self.total_c)**2 +\
                   self.load_Iz + self.load_mass*(self.load_c - self.total_c)**2
                  # 横摆惯量 [kg·m²]

        # 几何参数（各轴到质心距离）
        self.a = self.total_c - self.axis1    # 前轴 [m](在质心前为正)
        self.b = self.axis2 - self.total_c    # 中轴 [m](在质心后为正)
        self.c = self.axis3 - self.total_c    # 后轴 [m](在质心后为正)
        
        # 轮胎参数
        self.tire = PacejkaTire()
        self.R = 0.51    # 轮胎半径 [m]
        self.roll_resistant = 0.0055 #滚动阻力系数
        
        # 垂直载荷分配（静态）
        # self.Fz = [
        #     self.m * self.g * self.b / (self.a + self.b + self.c),  # 前轴
        #     self.m * self.g * self.c / (self.a + self.b + self.c),  # 中轴
        #     self.m * self.g * self.a / (self.a + self.b + self.c)   # 后轴
        # ]
        self.Fz = [
            self.m * self.g * (self.b + 2*self.c) / (3*(self.a + self.c)),  # 前轴
            self.m * self.g  / 3,  # 中轴
            self.m * self.g * (2*self.a - self.b) / (3*(self.a + self.c))   # 后轴
        ]
        
        self.Fx = [0.0, 0.0, 0.0]  # 各轴在地面坐标系下的x方向合力
        self.Fy = [0.0, 0.0, 0.0]  # 各轴在地面坐标系下的y方向合力

        self.F_lateral = [0.0, 0.0, 0.0]  # 记录各轴横纵向力
        self.F_longi = [0.0, 0.0, 0.0]    # 记录各轴横纵向力

        # 初始状态 [vx, vy, omega, X, Y, psi, w1, w2, w3]
        self.state = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                             10/0.5, 10/0.5, 10/0.5])  # 初始轮速
        
        

    def tire_forces(self, state, delta, torque):
        """计算各轴轮胎力（含轮速动力学）"""
        vx, vy, omega, *_ , w1, w2, w3 = state
        Fx_total = []
        Fy_total = []
        F_longi = []
        F_lateral = []
        Mz_total = 0.0
        dw_list = []
        
        # 各轴处理
        for i, (Fz_axle, delta_i, torque_i, wi) in enumerate(zip(
            self.Fz, delta, torque, [w1, w2, w3])):
            
            # 1.滑移率计算
            epsilon = 1e-3  # 防止除零的小量
            sign = lambda x: 1 if x >=0 else -1  # 符号函数

            # 改进后的滑移率计算
            vx_axle = vx
            re = self.R

            # 处理低速情况（保持符号连续性）
            vx_safe_slip = vx_axle + epsilon * sign(vx_axle) if abs(vx_axle) < epsilon else vx_axle

            # 区分驱动/制动工况（根据扭矩符号）
            if torque_i > 0:  # 驱动工况
                s = (wi * re - vx_safe_slip) / abs(vx_safe_slip) 
            else:             # 制动工况
                s = (vx_safe_slip - wi * re) / abs(vx_safe_slip)

            # 限制滑移率在物理合理范围
            s = np.clip(s, -1.0, 1.0)

            # 2.侧偏角计算
            # 处理vx接近零的情况（统一使用vx_safe）
            vx_safe_alpha = vx + epsilon * sign(vx) if abs(vx) < epsilon else vx

            # 根据轴位置符号调整计算
            alpha = delta_i - np.arctan2(
                (vy + self.a*omega) if i == 0 else (vy - self.b*omega) if i ==1 else (vy - self.c*omega),
                vx_safe_alpha
            )

            
            # 3. Pacejka模型计算
            Fx =   Fz_axle * self.tire.magic_formula(
                s, 
                self.tire.p['C_k'][i],
                self.tire.p['D_k'][i],
                self.tire.p['BCD_k'][i],
                self.tire.p['E_k'][i]
            )
            
            Fy =   Fz_axle * self.tire.magic_formula(
                alpha,
                self.tire.p['C_alpha'][i],
                self.tire.p['D_alpha'][i],
                self.tire.p['BCD_alpha'][i],
                self.tire.p['E_alpha'][i]
            )
            
            # 4. 考虑转向几何的力分解

            
            Fx_rot = Fx * np.cos(delta_i) - Fy * np.sin(delta_i)
            Fy_rot = Fx * np.sin(delta_i) + Fy * np.cos(delta_i)
            
            Fx_total.append(Fx_rot)
            Fy_total.append(Fy_rot)

            F_lateral.append(Fy)
            F_longi.append(Fx)
            
            # 5. 轮速动力学
            T_drive = torque_i
            # 车轮转动惯量 [kg·m²]
            Iw = self.Iw
            dw = (T_drive - Fx * re - self.roll_resistant * Fz_axle * self.R) / Iw
            dw_list.append(dw)
            
            # 累计力矩
            if i == 0:
                Mz_total += self.a * Fy_rot
            else:
                Mz_total -= [self.b, self.c][i-1] * Fy_rot
                
        return F_lateral, F_longi, Fx_total, Fy_total, Mz_total, dw_list

    def dynamics(self, state, t, delta, torque):
        vx, vy, omega, X, Y, psi, w1, w2, w3 = state
        
        # 计算轮胎力
        F_lateral, F_longi, Fx, Fy, Mz, dw = self.tire_forces(state, delta, torque)

        self.Fx = Fx  #记录在地面坐标系下的x方向合力
        self.Fy = Fy  #记录在地面坐标系下的y方向合力
        self.F_lateral = F_lateral  # 记录当前各轴纵向力
        self.F_longi = F_longi  # 记录当前各轴纵向力
        
        # 车辆动力学
        dvx = sum(Fx)/ self.m + vy*omega 
        dvy = sum(Fy)/ self.m - vx*omega
        domega = Mz / self.Iz
        
        # 运动学
        dX = vx*np.cos(psi) - vy*np.sin(psi)
        dY = vx*np.sin(psi) + vy*np.cos(psi)
        dpsi = omega
        
        # 轮速更新
        dw1, dw2, dw3 = dw
        
        return [dvx, dvy, domega, dX, dY, dpsi, dw1, dw2, dw3]

    def get_state(self):
        return {
            'position': { 
                'x':self.state[3],
                'y':self.state[4]
            },                       # X,Y
            'attitude': {
                'yaw': self.state[5]  # psi
            },
            'velocity': {
                'longitudinal': self.state[0],  # vx
                'lateral': self.state[1],       # vy
                'yaw_rate': self.state[2],       # omega
                'lateral_center':self.state[1] - (6.5/2 - self.total_c)*self.state[2] #vy_sprung
            },
            'wheel': {
                'speed': self.state[6:9],       # w1,w2,w3
                'Fx': self.F_longi,             # 各轴纵向力
                'Fy': self.F_lateral,           # 各轴侧向力
                'Fz': self.Fz                   # 各轴垂直载荷
            }
        }

    def update(self, dt, delta, torque):
        t_span = [0, dt]
        new_state = odeint(self.dynamics, self.state, t_span, 
                          args=(delta, torque))[-1]
        self.state = new_state
        return self.get_state()

# 使用示例（带非线性特性可视化）
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    vehicle = ThreeAxleVehiclePacejka()
    
    # 数据记录初始化
    history = {
        'time': [],
        'vx': [], 'vy': [], 'omega': [],
        'X': [], 'Y': [],
        'delta': {'front': [], 'mid': [], 'rear': []},
        'torque': {'front': [], 'mid': [], 'rear': []},
        'slip': {'front': [], 'mid': [], 'rear': []},
        'alpha': {'front': [], 'mid': [], 'rear': []},
        'Fx':{'front':[], 'mid':[], 'rear':[]},
        'Fy':{'front':[], 'mid':[], 'rear':[]}
        
    }
    
    # 仿真参数
    total_time = 25.0  # 仿真时间
    dt = 0.01         # 时间步长
    
    print("开始非线性模型仿真...")
    for t in np.arange(0, total_time, dt):
        # 生成动态输入（增强非线性激励）
        # 转向输入：前轴正弦+谐波，后轴反相
        delta_front = (np.sin(t * 2*np.pi/4) + 0.3*np.sin(t*2*np.pi/1.5)) * np.deg2rad(8)
        steering = [delta_front, 0, -delta_front*0.8]  # 后轴转向比例
        
        # 扭矩输入：带漂移成分的波动
        base_torques = [3000, 2000, 1500]  # 提高基准扭矩
        torque_fluct = [
            100 * np.sin(t * 2*np.pi/3.5) * (1 + 0.1*t),  # 时变振幅
            100 * np.sin(t * 2*np.pi/3.5),
            100 * np.cos(t * 2*np.pi/3.5)
        ]
        torques = [ base+fluct for base, fluct in 
                   zip(base_torques,torque_fluct)]
        
        # 更新状态并记录
        state = vehicle.update(dt, steering, torques)
        
        # 记录扩展数据
        history['time'].append(t)
        history['vx'].append(state['velocity']['longitudinal'])
        history['vy'].append(state['velocity']['lateral'])
        history['omega'].append(state['velocity']['yaw_rate'])
        history['X'].append(state['position']['x'])
        history['Y'].append(state['position']['y'])
        history['delta']['front'].append(np.rad2deg(steering[0]))
        history['delta']['rear'].append(np.rad2deg(steering[2]))
        history['torque']['front'].append(torques[0])
        history['torque']['mid'].append(torques[1])
        history['torque']['rear'].append(torques[2])

        # 记录轮胎特性（需从类内部获取）
        history['slip']['front'].append(vehicle.tire_forces(vehicle.state, steering, torques)[0][0])
        history['alpha']['front'].append(np.rad2deg(vehicle.tire_forces(vehicle.state, steering, torques)[1][0]))

        # 记录轮胎力
        history['Fx']['front'].append(vehicle.Fx[0])
        history['Fx']['mid'].append(vehicle.Fx[1])
        history['Fx']['rear'].append(vehicle.Fx[2])
        history['Fy']['front'].append(vehicle.Fy[0])
        history['Fy']['mid'].append(vehicle.Fy[1])
        history['Fy']['rear'].append(vehicle.Fy[2])

    
    # 创建专业可视化面板
    plt.figure(figsize=(18, 12))
    
    # 1. 输入信号分析
    plt.subplot(3, 3, 1)
    plt.plot(history['time'], history['delta']['front'], label='Front Steer')
    plt.plot(history['time'], history['delta']['rear'], '--', label='Rear Steer')
    plt.title('Steering Angle Input\n(Phase Shifted)')
    plt.xlabel('Time (s)')
    plt.ylabel('Degrees')
    plt.grid(True, alpha=0.3)
    plt.legend(loc='upper right')

    plt.subplot(3, 3, 2)
    for ax in ['front', 'mid', 'rear']:
        plt.plot(history['time'], history['torque'][ax], label=ax.capitalize())
    plt.title('Torque Input with Fluctuations')
    plt.xlabel('Time (s)')
    plt.ylabel('Nm')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 2. 车辆动力学响应
    plt.subplot(3, 3, 4)
    plt.plot(history['time'], history['vx'], label='Longitudinal')
    plt.plot(history['time'], history['vy'], label='Lateral')
    plt.title('Velocity Components\n(Show Nonlinear Coupling)')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    plt.subplot(3, 3, 5)
    plt.plot(history['time'], np.rad2deg(history['omega']), 'g-')
    plt.title('Yaw Rate Response\n(Note Saturation Effects)')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Rate (deg/s)')
    plt.grid(True, alpha=0.3)
    
    # 3. 轮胎特性可视化
    plt.subplot(3, 3, 3)
    plt.scatter(history['alpha']['front'], history['slip']['front'], 
               c=history['time'], cmap='viridis', s=10)
    plt.colorbar(label='Time (s)')
    plt.title('Tire Slip vs Slip Angle\n(Show Hysteresis Loop)')
    plt.xlabel('Slip Angle (deg)')
    plt.ylabel('Longitudinal Slip')
    plt.grid(True, alpha=0.3)
    
    # 4. 运动轨迹分析
    plt.subplot(3, 3, 6)
    plt.plot(history['X'], history['Y'], 'b-', lw=1, label='Path')
    # 动态航向标记
    for i in range(0, len(history['X']), 100):  # 每5秒标记
        psi = np.arctan2(history['Y'][min(i+1,len(history['Y'])-1)]-history['Y'][i],
                        history['X'][min(i+1,len(history['X'])-1)]-history['X'][i])
        plt.arrow(history['X'][i], history['Y'][i], 
                3*np.cos(psi), 3*np.sin(psi),
                head_width=1.5, fc='r', ec='r', alpha=0.7)
    plt.title('Spatial Trajectory\n(With Direction Indicators)')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    
    # 5. 相空间分析
    plt.subplot(3, 3, 7)
    plt.plot(history['vx'], history['vy'], 'm-', lw=0.5)
    plt.title('Velocity Phase Portrait\n(Show Attractor)')
    plt.xlabel('Vx (m/s)')
    plt.ylabel('Vy (m/s)')
    plt.grid(True, alpha=0.3)
    
    # 6. 非线性特性分析
    plt.subplot(3, 3, 8)
    plt.plot(history['time'], np.abs(history['vy']), label='|Vy|')
    plt.plot(history['time'], np.rad2deg(np.abs(history['omega'])), label='|Yaw|')
    plt.title('Nonlinear Metrics Growth')
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude')
    plt.yscale('log')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    plt.subplot(3, 3, 9)
    plt.plot(history['time'], history['Fy']['front'], label='Front Fy')
    plt.plot(history['time'], history['Fy']['rear'], label='Rear Fy')
    plt.title('Tire Force Saturation')
    plt.xlabel('Time (s)')
    plt.ylabel('Lateral Force (N)')
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.tight_layout()
    plt.show()