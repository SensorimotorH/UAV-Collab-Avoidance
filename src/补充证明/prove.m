clear,clc;
%证明切点弦比切点弧大
%既证明：500*tan(theta*pi/180)-1000*pi*theta/360>0
theta=sym('theta');
y=500*tan(theta*pi/180)-1000*pi*theta/360;
der=diff(y,theta);
delta=0:0.1:60;
der_value=zeros(1,60*10);
index=1;
for i =0:0.1:60
    der_value(index)=subs(der,theta,i);
    index=index+1;
end
plot(delta,der_value,'r');