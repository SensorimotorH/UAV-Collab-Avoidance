clear,clc;
global x_zero;
answer=[];
v=[10 10];
x=[-3500 1000];
y=[0 0];
max_t=500*sqrt(3)/10;

t=sym('t');
assume(t>0 & t<max_t);
x_B=sym('x_B');
assume(x_B>=-10000 & x_B<=-1000)
v_B=sym('v_B');
assume(v_B>=10 & v_B<=50);


delta_t=( 500*tan( acos( 500/(abs(x_B)))) + 2*pi*500*(180-60-acosd(500/(abs(x_B))))/360  )/v_B;
delta_s=10*delta_t;
% delta_s=subs(delta_s,x_B,-3500);
% delta_s=subs(delta_s,v_B,10)
theta=atand(v_B*t/500);
s1=1000*sin(pi/180*theta)/sin(pi/180*(180-30-theta))-delta_s
var_s1=symvar(s1)
s2=10*t;
der1=diff(s1,t)
var_der1=symvar(der1)
eq1 = der1 == 10;
tangency_x = solve(eq1, t)
y1_tangency=subs(s1,t,tangency_x)
line_tangency=10*(t-tangency_x)+y1_tangency
x_zero=subs(line_tangency,t,0)
var_x_zero=symvar(x_zero)
fsurf(x_zero,[10,50,-10000,-1000]);
%answer=double(subs(x_zero,[v_B,x_B],[37.67,-2075]))

% %% 模型设置
% % 变量个数
nvars=2;
% 约束条件形式1：（若无取空数组[]）
% lb<= X <= ub
lb=[10,-10000];
ub=[50,-1000];
% 
% % 约束条件形式2：（若无取空数组[]）
% % A*X <= b 
A = [];

b = [];
% 
% % 约束条件形式3：（若无取空数组[]）
% % Aeq*X = beq
Aeq=[];
beq=[];
% 
% %% 求解器设置
% % 最优个体系数paretoFraction
% % 种群大小populationsize
% % 最大进化代数generations
% % 停止代数stallGenLimit
% % 适应度函数偏差TolFun
% % 函数gaplotpareto：绘制Pareto前沿 
options=gaoptimset('paretoFraction',0.3,'populationsize',200,'generations',300,'stallGenLimit',200,'TolFun',1e-10,'PlotFcns',@gaplotpareto);
% 
% %% 主求解
[x,fval]=gamultiobj(@object_fuc,nvars,A,b,Aeq,beq,lb,ub,@condition_fuc,options);
% 
% %% 结果提取
% % 因为gamultiobj是以目标函数分量取极小值为目标，
% % 因此在y=Fun(x)里取相反数的目标函数再取相反数画出原始情况
plot(-1*fval(:,1),-1*fval(:,2)*200,'pr');
xlabel('f_1(x)');
ylabel('f_2(x)');
title('Pareto front');
grid on;
function y=object_fuc(x)
% % y是目标函数向量。有几个目标函数y就有多少个维度（数组y的长度）
% % 因为gamultiobj是以目标函数分量取极小值为目标，
% % 因此有些取极大值的目标函数注意取相反数


y(1) = -1*x(1);
y(2)=-1*x(2)/200;
end
function [c,ceq]=condition_fuc(x)
    global x_zero;
    fit = x_zero^2;
    var_fit=symvar(fit);
    fit=subs(fit,var_fit(1),x(1));
    fit=subs(fit,var_fit(2),x(2));
    c=[];
    ceq=[fit];
end







