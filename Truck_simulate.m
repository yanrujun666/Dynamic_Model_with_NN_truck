clc;
clear
%% 整车参数(PAC轮胎)

m_sprung = 4455; % 簧上车身质量kg
m_load = 5000; %货物质量kg
m_unsprung = 350 * 3 ; %簧下质量kg
m = m_sprung + m_load + m_unsprung; % 整车质量kg（簧上与前后簧下质量之和）

% Bf = 2.030; % 前轴轮距m 
% Bm = 2.030; % 中轴轮距m
% Br = 2.030; % 后轴轮距m
% Lf = ; % 质心距离前轴的距离m
% Lm = ; % 质心距离中轴的距离m
% Lr = ; % 质心距离后轴的距离m
% L = ; % 轴距m 
% Iz = ; % 整车z轴转动惯量kg*m^2


r = 0.510; % 车轮滚动半径/m
u = 1; % 路面附着系数
g = 9.8; % 重力加速度

wz0 = 0; % 初始z角速度 rad/s
vx0 = 5; % 初始Vx速度 m/s
vy0 = 0; % 初始Vy速度 m/s
w0 = vx0 / r; % 初始轮速 rad/s

% 保存数据
vehicle_params.m = m;
% vehicle_params.Bf = Bf;
% vehicle_params.Br = Br;
% vehicle_params.Lf = Lf;
% vehicle_params.Lr = Lr;
% vehicle_params.L = L;
% vehicle_params.Iz = Iz;
vehicle_params.r = r;
vehicle_params.u = u;
vehicle_params.g = g;

% 初始条件
initial_conditions.wz0 = wz0;
initial_conditions.vx0 = vx0;
initial_conditions.vy0 = vy0;
initial_conditions.w0 = vx0 / r;

% 获取仿真设置参数
model = 'trucksim_specific_control';
open_system(model);
sim_params.time = get_param(model, 'StopTime');
sim_params.solverType = get_param(model, 'Solver');
sim_params.fixedStep = get_param(model, 'FixedStep');

h = waitbar(0, 'Simulation Progress');

t_motor_const = [50, 150, 250]; %[0, 50, 100]
%t_motor_sine_amp = 50;  %50
%t_motor_sine_fre = 0.5; %0.5
delta_act_sine_amp = [0.025,  0.05,  0.1];  %[0, 0.075,  0.15]
delta_act_sine_fre = [0.25, 0.5 ,0.8];  %[0, 0.5 ,1]



%total_iterations = length(delta_act_sine_amp) * length(delta_act_sine_fre)-2;
count = 0;
% 运行每个工况的仿真
total_iterations = length(t_motor_const) * length(delta_act_sine_amp) * length(delta_act_sine_fre);


for i = 1:length(t_motor_const)
    for j = 1:length(delta_act_sine_amp)         
        for k = 1:length(delta_act_sine_fre)
	        
	        T_motor_constant = t_motor_const(i); 
            % T_motor_sine_amp = t_motor_sine_amp;
            % T_motor_sine_fre = t_motor_sine_fre;
	        Delta_act_sine_amp = delta_act_sine_amp(j);
	        Delta_act_sine_fre = delta_act_sine_fre(k);
	        
            specific_control_params.T_motor_constant = t_motor_const(i); 
            % specific_control_params.T_motor_sine_amp = t_motor_sine_amp;
            % specific_control_params.T_motor_sine_fre = t_motor_sine_fre;
	        specific_control_params.Delta_act_sine_amp = delta_act_sine_amp(j);
	        specific_control_params.Delta_act_sine_fre = delta_act_sine_fre(k);
    
	        % 运行仿真+获取数据
            sim(model);
	        T_motor = T_motor_out.Data;
	        Delta_act = Delta_act_out.Data;
            Vehicle_state_trucksim_33d = Vehicle_state_trucksim_33d_out.Data;
           
            % 隔10个数据取一个(0.01s)
            T_motor = T_motor(1:10:end, :);
            Delta_act = Delta_act(1:10:end, :);	
            Vehicle_state_trucksim_33d = Vehicle_state_trucksim_33d(1:10:end, :);
            
      
                
	        % 构造文件路径
            save_dir = 'D:\\YRJ_Workspace\\Dynamic_Model_with_NN_truck\\data\\trucksim\\specific_control\\data_v_test_3';

            % 如果文件夹不存在，则创建
            if ~exist(save_dir, 'dir')
                mkdir(save_dir);
            end
            % 生成文件名
            save_filename = sprintf('%s\\data%03d.mat', save_dir, count);
            save(save_filename, 'sim_params', 'vehicle_params', 'initial_conditions', 'specific_control_params', 'T_motor', 'Delta_act','Vehicle_state_trucksim_33d');
            waitbar(count / total_iterations, h);
            count = count+1;
        end
    end
end
close(h);



