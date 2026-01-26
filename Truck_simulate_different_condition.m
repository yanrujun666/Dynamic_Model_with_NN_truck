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
model = 'trucksim_different_condition';
open_system(model);
sim_params.time = get_param(model, 'StopTime');
sim_params.solverType = get_param(model, 'Solver');
sim_params.fixedStep = get_param(model, 'FixedStep');

% 关键修改 1: 定义 Signal Editor 模块的块路径
signalEditorBlockPath = [model, '/Signal Editor-opposite_steering']; 

% signalEditorBlockPath = [model, '/Signal Editor-tank_turn'];

% signalEditorBlockPath = [model, '/Signal Editor-front_middle_steering']; 

% signalEditorBlockPath = [model, '/Signal Editor-same_steering_crab_motion']; 

% signalEditorBlockPath = [model, '/Signal Editor-lateral_movement']; 


% 尝试使用不同的方法获取场景信息
try
    % 方法1: 尝试使用 getScenarioNames 函数（如果可用）
    scenarioNames = getScenarioNames(signalEditorBlockPath);
catch
    try
        % 方法2: 尝试使用 get_param 获取 'ActiveScenario' 参数
        % 首先获取当前活动场景
        currentActiveScenario = get_param(signalEditorBlockPath, 'ActiveScenario');

        % 然后尝试获取所有场景
        % 注意：这种方法可能不适用于所有版本，但值得一试
        scenarioInfo = get_param(signalEditorBlockPath, 'Scenarios');
        scenarioNames = arrayfun(@(x) x.Name, scenarioInfo, 'UniformOutput', false);
    catch
        % 方法3: 如果以上方法都失败，手动指定场景名称
        % % 您需要根据您的 Signal Editor 中实际设置的场景名称进行修改
        
        % scenarioNames = {'Scenario_straight_const_5m_s','Scenario_straight_acc_5m_s','Scenario_straight_deacc_5m_s'...
        %     ,'Scenario_straight_const_10m_s','Scenario_straight_acc_10m_s','Scenario_straight_deacc_10m_s'...
        %     ,'Scenario_straight_const_15m_s','Scenario_straight_acc_15m_s','Scenario_straight_deacc_15m_s'}; %直行工况

        % scenarioNames = {'Scenario_straight_const_5m_s','Scenario_straight_acc_5m_s','Scenario_straight_deacc_5m_s'...
        %     ,'Scenario_snake_const_5m_s','Scenario_snake_acc_5m_s','Scenario_snake_deacc_5m_s'}; %直行+蛇形工况



       
        % scenarioNames = {'Scenario_tank_turn_500Nm','Scenario_tank_turn_750Nm',...
        %     'Scenario_tank_turn_1000Nm','Scenario_tank_turn_1250Nm','Scenario_tank_turn_1500Nm'}; %坦克掉头不同转矩输入

        % scenarioNames = {'Scenario_left_turn_acc_5m_s','Scenario_left_turn_const_5m_s','Scenario_left_turn_deacc_5m_s',...
        %     'Scenario_right_turn_acc_5m_s','Scenario_right_turn_const_5m_s','Scenario_right_turn_deacc_5m_s'}; %左转、右转

        % 
        % scenarioNames = {'Scenario_lateral_movement_1','Scenario_lateral_movement_2','Scenario_lateral_movement_3','Scenario_lateral_movement_4',...
        %     'Scenario_lateral_movement_5','Scenario_lateral_movement_6'};

        % scenarioNames = {'Scenario_snake_acc_5m_s_1','Scenario_snake_acc_5m_s_2','Scenario_snake_acc_5m_s_3','Scenario_snake_acc_5m_s_4',...
        %                  'Scenario_snake_acc_5m_s_5','Scenario_snake_acc_5m_s_6','Scenario_snake_acc_5m_s_7','Scenario_snake_acc_5m_s_8',...
        %                  'Scenario_snake_acc_5m_s_9','Scenario_snake_acc_5m_s_10','Scenario_snake_acc_5m_s_11','Scenario_snake_acc_5m_s_12',...
        %                  'Scenario_snake_acc_5m_s_13','Scenario_snake_acc_5m_s_14','Scenario_snake_acc_5m_s_15','Scenario_snake_acc_5m_s_16',...
        %                  'Scenario_snake_const_5m_s_1','Scenario_snake_const_5m_s_2','Scenario_snake_const_5m_s_3','Scenario_snake_const_5m_s_4',...
        %                  'Scenario_snake_const_5m_s_5','Scenario_snake_const_5m_s_6','Scenario_snake_const_5m_s_7','Scenario_snake_const_5m_s_8',...
        %                  'Scenario_snake_const_5m_s_9','Scenario_snake_const_5m_s_10','Scenario_snake_const_5m_s_11','Scenario_snake_const_5m_s_12',...
        %                  'Scenario_snake_const_5m_s_13','Scenario_snake_const_5m_s_14','Scenario_snake_const_5m_s_15','Scenario_snake_const_5m_s_16',...
        %                  'Scenario_snake_deacc_5m_s_1','Scenario_snake_deacc_5m_s_2','Scenario_snake_deacc_5m_s_3','Scenario_snake_deacc_5m_s_4',...
        %                  'Scenario_snake_deacc_5m_s_5','Scenario_snake_deacc_5m_s_6','Scenario_snake_deacc_5m_s_7','Scenario_snake_deacc_5m_s_8',...
        %                  'Scenario_snake_deacc_5m_s_9','Scenario_snake_deacc_5m_s_10','Scenario_snake_deacc_5m_s_11','Scenario_snake_deacc_5m_s_12',...
        %                  'Scenario_snake_deacc_5m_s_13','Scenario_snake_deacc_5m_s_14','Scenario_snake_deacc_5m_s_15','Scenario_snake_deacc_5m_s_16',...
        %                  }; 
        
        scenarioNames = {'Scenario_snake_rollover_5m_s_1','Scenario_snake_rollover_5m_s_2','Scenario_snake_rollover_5m_s_3','Scenario_snake_rollover_5m_s_4'
                         }; 
        % 'Scenario_snake_rollover_10m_s_1','Scenario_snake_rollover_10m_s_2'
      
        warning('无法自动获取场景名称，使用手动指定的场景列表。请确保这些名称与Signal Editor中的场景匹配。');
    end
end

total_iterations = length(scenarioNames); % 总迭代次数等于场景数量

h = waitbar(0, 'Simulation Progress');
count = 0;

% 关键修改 3: 循环遍历每个场景
for i = 1:total_iterations
    currentScenario = scenarioNames{i};
    
    % 在运行仿真前，设置 Signal Editor 模块的 ActiveScenario 参数
    set_param(signalEditorBlockPath, 'ActiveScenario', currentScenario);
    
    % 保存当前场景名到参数中，便于后续记录
    specific_control_params.ActiveScenario = currentScenario;
    % 您之前的 specific_control_params 字段可能不再需要，或者可以从场景名中解析出参数
    % specific_control_params.T_motor_constant = ... 
    % specific_control_params.Delta_act_sine_amp = ...;
    % specific_control_params.Delta_act_sine_fre = ...;

    % 运行仿真
    sim(model);
    
    % 获取数据
    T_motor = T_motor_out.Data;
    Delta_act = Delta_act_out.Data;
    Vehicle_state_trucksim_39d = Vehicle_state_trucksim_39d_out.Data;

    % 数据处理：隔10个数据取一个(0.01s)
    T_motor = T_motor(1:10:end, :);
    Delta_act = Delta_act(1:10:end, :);	
    Vehicle_state_trucksim_39d = Vehicle_state_trucksim_39d(1:10:end, :);
    
    % 构造文件路径
    save_dir = ['D:\\YRJ_Workspace\\Dynamic_Model_with_NN_truck\\data\\trucksim\\different_scenarios\\data_scenarios_for_rollover_v1\\' ...
        'VehicleParams_IzuA_IzulA_RrA_CdA_BcdA'];

    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end
    
    % 生成文件名。可以使用场景名作为文件名，更清晰
    % 注意：场景名可能包含空格或非法字符，需要处理一下
    safeScenarioName = regexprep(currentScenario, '\s', '_');
    save_filename = sprintf('%s\\%s.mat', save_dir, safeScenarioName);
    % 或者沿用您的计数方式： save_filename = sprintf('%s\\data%03d.mat', save_dir, count);
    
    % 保存数据
    save(save_filename, 'sim_params', 'vehicle_params', 'initial_conditions', 'specific_control_params', 'T_motor', 'Delta_act','Vehicle_state_trucksim_39d');
    
    waitbar(i / total_iterations, h, sprintf('Running Scenario: %s (%d/%d)', currentScenario, i, total_iterations));
    count = count + 1;
end
close(h);