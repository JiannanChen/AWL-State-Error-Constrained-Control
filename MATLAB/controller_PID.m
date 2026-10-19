function out = controller_PID(in)
% 本文对比仿真的 PI 控制器（不使用微分项和输入限幅）
%
% 输入：in = [x1; x2; theta_f; gamma; t]
% 输出顺序与本文方法及反步法模型保持一致。

%% 状态和时间
x1 = in(1);
x2 = in(2);
x3 = in(3);
t = in(5);

%% 与论文一致的期望轨迹
x1d = sin(0.1*t);
x2d = cos(0.1*t);

%% PI 增益
% 默认值为离线整定结果。扫参时可在 base workspace 中使用
% global PI_GAINS; PI_GAINS = [Kp_pos, Ki_pos, Kp_theta, Ki_theta]
% 临时覆盖。控制律中不含 D 项。
global PI_GAINS
if isempty(PI_GAINS)
    gains = [1.50, 0.15, 2.00, 0.08];
else
    gains = PI_GAINS;
end
Kp_pos = gains(1);
Ki_pos = gains(2);
Kp_theta = gains(3);
Ki_theta = gains(4);

%% 位置外环 PI
% 控制器块在 Simulink 中以 0.01 s 离散采样运行。使用实际时间差
% 积分，避免连续求解器在一个主步内重复调用函数时重复累加。
persistent int_e1 int_e2 int_e3 prev_e1 prev_e2 prev_e3 last_t

e1_ctrl = x1d - x1;
e2_ctrl = x2d - x2;

if isempty(last_t) || t < last_t
    int_e1 = 0;
    int_e2 = 0;
    int_e3 = 0;
    prev_e1 = e1_ctrl;
    prev_e2 = e2_ctrl;
    prev_e3 = 0;
    last_t = t;
end

dt = t - last_t;
if dt > 0
    int_e1 = int_e1 + 0.5*(e1_ctrl + prev_e1)*dt;
    int_e2 = int_e2 + 0.5*(e2_ctrl + prev_e2)*dt;
end

ux = Kp_pos*e1_ctrl + Ki_pos*int_e1;
uy = Kp_pos*e2_ctrl + Ki_pos*int_e2;

%% 虚拟速度到车辆输入的坐标变换
% 按论文与其他方法的写法，不对 v_f 做限幅，也不设置人为下限。
v_f = sqrt(ux^2 + uy^2);
x3d = atan2(uy, ux);

%% 航向内环 PI
% 航向误差映射到 (-pi, pi]，避免 atan2 分支处出现虚假的 2*pi 误差。
e3_ctrl = atan2(sin(x3d - x3), cos(x3d - x3));
if dt > 0
    int_e3 = int_e3 + 0.5*(e3_ctrl + prev_e3)*dt;
end

% 仅使用比例项和积分项；按照论文对比设定，不做 u_theta 限幅。
u_theta = Kp_theta*e3_ctrl + Ki_theta*int_e3;

if dt > 0
    prev_e1 = e1_ctrl;
    prev_e2 = e2_ctrl;
    prev_e3 = e3_ctrl;
    last_t = t;
else
    % 保存 t = 0 时的航向误差，供第一个采样区间使用。
    prev_e3 = e3_ctrl;
end

%% 统一误差符号：e_i = x_i - x_id
e1 = x1 - x1d;
e2 = x2 - x2d;
e3 = atan2(sin(x3 - x3d), cos(x3 - x3d));

out = [ux; uy; u_theta; v_f; x3d; x1d; x2d; e1; e2; e3];

end
