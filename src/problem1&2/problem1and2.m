clear,clc;
figure;
t=sym('t');
v=sym('v');
max_t=sqrt(3500^2-500^2)/10;%超过该时间，B,A无人机不肯能有新�?轮的碰面


hypotenuse=sqrt((3500)^2+(v*t).^2-2*3500*v*t*cos(asin(500/3500)));
theta=asin(v*t*(1/7)/hypotenuse);
s1=500*tan(theta);
s1=subs(s1,v,10);
subplot(3,1,1);
fplot(s1,[0,max_t],'r');
title('B���˻���ӳ��·����ʱ��仯');
%fsurf(s1,[100,max_t,10,30]);
der1=diff(s1,t);
symvar(der1)
delta_s0=500*sqrt(3)+2*pi*500*(180-60-acosd(1/7))/360;
s2=10*t-delta_s0;
subplot(3,1,2);
fplot(s2,[0,max_t],'g');
title('A���˻���·����ʱ��仯');
subplot(3,1,3);
fplot(s1,[0,max_t],'r');
hold on;
fplot(s2,[0,max_t],'g');
title('�������˻��ģ�ӳ�䣩·����ʱ��仯');
