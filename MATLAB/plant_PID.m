function out = plant_PID(in)
%% 控制器所需的变量
x1 = in(1);
x2 = in(2);
x3 = in(3);
gamma = in(4);
vf = in(5);
utheta = in(6);


%% 物理参数
lf=1.7;
lr=1.6;


%% 系统
dx1 = vf*cos(x3);
dx2 = vf*sin(x3);
dx3 = vf*sin(gamma)/(lf*cos(gamma) + lr) + (lr*utheta)/(lf*cos(gamma) + lr);
dgamma=utheta;

%% 输出
out = [dx1;dx2;dx3;dgamma];  %%4个输出