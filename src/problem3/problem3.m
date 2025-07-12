clear,clc;
t=sym('t');
x_B=sym('x_B');
assume(x_B<-500);
v_B=sym('v_B');

hypotenuse=sqrt((abs(x_B))^2+(v_B*t).^2-2*abs(x_B)*v_B*t*cos(asin(500/abs(x_B))));
theta=asin(v_B*t*500/abs(x_B)/hypotenuse);
delta_s0=500*sqrt(3)+2*pi*500*(180-60-acosd(500/abs(x_B)))/360;
s1=500*tan(theta)+delta_s0;
s1=subs(s1,v_B,10)
s1=subs(s1,x_B,2092.7)
var=symvar(s1)
%fplot(s1,[0,1000],'r');
der1=diff(s1,t);

s2=10*t;

zero_x=zeros(1,300);
index=1;
%for i=-1000:-10:-3500
for i=-2092.7:-0.01:-2092.8
    der1_temp=subs(der1,x_B,i);
    s1_temp=subs(s1,x_B,i);
    eqn = der1_temp == 10;
    tangency_x = solve(eqn, t);
    y1_tangency=subs(s1_temp,t,tangency_x);
    line_tangency=10*(t-tangency_x)+y1_tangency;
    temp=double(subs(line_tangency,t,0))
    zero_x(index)=temp(1);
    index=index+1;
end
zero_x;
%¼«ÏÞ¾àÀë2092.7