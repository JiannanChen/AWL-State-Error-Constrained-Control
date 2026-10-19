function out = plant(in)
% 系统模型文件：铰接式装载机运动学模型
%
% 输入 in:
% in(1) = x1        前桥中心 x 坐标
% in(2) = y1        前桥中心 y 坐标
% in(3) = theta_f   前车体航向角
% in(4) = gamma     铰接角
% in(5) = v_f       前车体速度
% in(6) = u_theta   铰接角速度，即 gamma_dot
%
% 输出 out:
% out(1) = dx1
% out(2) = dy1
% out(3) = dtheta_f
% out(4) = dgamma

%% 控制器所需的变量
x1 = in(1);
x2 = in(2);
x3 = in(3);
gamma = in(4);
v_f = in(5);
u_theta = in(6);

%% 模型参数
l_f = 1.7;
l_r = 1.6;

%% 模型方程
dx1 = v_f*cos(x3);
dx2 = v_f*sin(x3);
dx3 = v_f*sin(gamma)/(l_f*cos(gamma) + l_r) + (l_r*u_theta)/(l_f*cos(gamma) + l_r);
dgamma=u_theta;
out = [dx1;dx2;dx3;dgamma];  %%4个输出

end
