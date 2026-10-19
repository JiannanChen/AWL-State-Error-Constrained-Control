function out = controller(in)
% 纯反步轨迹跟踪控制器（论文对比方法）
%
% 输入 in:
% in(1) = x1        前桥中心 x 坐标
% in(2) = y1        前桥中心 y 坐标
% in(3) = theta_f   前车体航向角
% in(4) = gamma     铰接角
% in(5) = t         仿真时间
%
% 输出 out:
% out(1)  = ux          x 方向虚拟控制量
% out(2)  = uy          y 方向虚拟控制量
% out(3)  = u_theta     铰接角速度控制量
% out(4)  = v_f         前车体速度控制量
% out(5)  = theta_fd    theta_f 的目标轨迹
% out(6)  = x1d         x1 的目标轨迹
% out(7)  = y1d         y1 的目标轨迹
% out(8)  = e1          x1 跟踪误差
% out(9)  = e2          y1 跟踪误差
% out(10) = e3          theta_f 跟踪误差

%% 控制器所需的变量
x1 = in(1);
x2 = in(2);
x3 = in(3);
gamma = in(4);
t = in(5);

%% 模型参数
l_f = 1.7;
l_r = 1.6;

%% 控制器参数
% 默认值为离线整定结果。扫参时可在 base workspace 中用
% global BS_GAINS; BS_GAINS = [k1, k2, k3]; 临时覆盖。
global BS_GAINS
if isempty(BS_GAINS)
    gains = [0.5, 0.6, 1.2];
else
    gains = BS_GAINS;
end
k1 = gains(1);
k2 = gains(2);
k3 = gains(3);

%% 期望轨迹
x1d = sin(0.1*t);
x2d = cos(0.1*t);

dx1d =  0.1*cos(0.1*t);
dx2d = -0.1*sin(0.1*t);

ddx1d = -0.01*sin(0.1*t);
ddx2d = -0.01*cos(0.1*t);

%% x1 跟踪误差和约束映射
e1 = x1 - x1d;

ux = -k1*e1 + dx1d;

%% y1 跟踪误差和约束映射
e2 = x2 - x2d;

uy = -k2*e2 + dx2d;

%% 由 ux 和 uy 计算 theta_f 目标轨迹和 v_f
x3d = atan2(uy,ux);
v_f = sqrt(ux^2 + uy^2);

%% theta_fd 的导数
% ux、uy 在位置误差收敛阶段也随状态变化，因此 theta_fd 的导数
% 必须由 atan2(uy,ux) 的完整导数计算，不能只使用参考轨迹的切向角速度。
dx1 = v_f*cos(x3);
dx2 = v_f*sin(x3);
dux = -k1*(dx1 - dx1d) + ddx1d;
duy = -k2*(dx2 - dx2d) + ddx2d;
dx3d = (ux*duy - uy*dux)/max(ux^2 + uy^2, 1e-10);

%% theta_f 跟踪误差和约束映射
% 航向角是周期变量，采用 (-pi, pi] 内的最短角误差，避免 atan2
% 在正负 pi 处跳变时产生虚假的 2*pi 控制冲击。
e3 = atan2(sin(x3 - x3d), cos(x3 - x3d));
      

%% theta_f 子系统控制律
u_theta =(l_f * cos(gamma) + l_r) / l_r * (-k3*e3 + dx3d - (v_f * sin(gamma)) / (l_f * cos(gamma) + l_r));

out = [ux;uy;u_theta;v_f;x3d;x1d;x2d;e1;e2;e3];  %%10个输出

end
