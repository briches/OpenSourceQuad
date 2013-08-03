% Quadcopter equations of motion
clear all
clc
close all
 
d=20.5*0.0254/sqrt(2);
l=20.5*0.0254/2;
% masses
mb=0.333;  %battery kg
mw=0.5*mb; %battery wires kg
mm=0.052;  %motor kg
mp=0.000;  %propeller kg
me=0.028;  %esc kg
mf=0.195/4;   %frame (one arm) kg
mbb=0.2;   %breadboard kg
 
m1=mm;
ma=mf;
mi=me;
 
mtot=mb+mw+mm+mp+me+mf+mbb;
 
% Inertia
Ixx=(2*m1+1/2*ma+1/8*mi)*l^2;
Iyy=Ixx+mb*.173^2/12;
Izz=(4*m1+ma+1/4*mi)*l^2;
 
% Pitch and roll inputs
% t=0:.01:5;
% for i=1:length(t)
%     Mr(i)=1*l*sin(pi/4);
%     Mp(i)=1*l*sin(pi/4);
% end
syms Mr Mp x1 x2 x3 x4 %Ixx Iyy Izz
 
% state equations
x1d=x2;
x2d=(Mr-Iyy*x3^2*cos(x1)*sin(x1))/Ixx;
x3d=x4;
x4d=(Mp+(Iyy-Izz)*2*x2*x4*cos(x1)*sin(x1))/(Iyy*cos(x1)^2+Izz*sin(x1)^2);
% Linearizing system via Jacobian Matrix
Asym=jacobian([x1d;x2d;x3d;x4d],[x1,x2,x3,x4]);
Bsym=jacobian([x1d;x2d;x3d;x4d],[Mr,Mp]);
Aic=subs(Asym, [x1 x2 x3 x4 Mr Mp], [0 0 0 0 1*l 1*d]);
Bic=subs(Bsym, [x1 x2 x3 x4 Mr Mp], [0 0 0 0 1*l 1*d]);
 
sA=size(Aic);
sB=size(Bic);
for mA=1:sA(1,1)
    for nA=1:sA(1,2)
        A(mA,nA)=Aic(mA,nA);
    end
end
 
for mB=1:sB(1,1)
    for nB=1:sB(1,2)
        B(mB,nB)=Bic(mB,nB);
    end
end
A;
B;
C=[1 0 0 0;0 0 1 0];
D=zeros(2,2);
 
sys=ss(A,B,C,D);
% controllable and observable?
Co=ctrb(A,B);
rank(Co)
Ob=obsv(A,C);
rank(Ob)
% Transfer function
s=tf('s');
Gr=1/(Ixx*s^2);
Gp=1/(Iyy*s^2);
 
 
% Controller Roll
zeta=0.8;
Ts=0.4;
wn=4/(Ts*zeta);
kd1=2*Ixx*zeta*wn;
a1=Ixx*wn^2/kd1;
b1=0.01;
kp1=kd1*(a1+b1);
ki1=kd1*a1*b1;
C1=(kd1*s^2+kp1*s+ki1)/s;
Gclr=feedback(C1*Gr,1);
% Roll Discrete Time (z-domain)
T=.02;
Gclrz=c2d(Gclr,T);
 
% Controller Pitch
zeta=0.8;
Ts=0.4;
wn=4/(Ts*zeta);
kd2=2*Iyy*zeta*wn;
a2=Iyy*wn^2/kd2;
b2=0.01;
kp2=kd2*(a2+b2);
ki2=kd2*a2*b2;
C2=(kd2*s^2+kp2*s+ki2)/s;
Gclp=feedback(C2*Gp,1);
% Pitch Discrete Time (z-domain)
T=.02;
Gclpz=c2d(Gclp,T);
 
K=[kd1 kp1 ki1;kd2 kp2 ki2];
t=0:0.02:5;
for i=1:length(t)
    u(i)=1;
end
y=lsim(Gr,u,t);
yc1=lsim(Gclr,u,t);
yc2=lsim(Gclp,u,t);
 
% Discrete simulation
yc1z=lsim(Gclrz,u,t);
yc2z=lsim(Gclpz,u,t);
 
% plotting root locus and compensated vs. uncompensated systems
figure(1)
rlocus(C1*Gr/kd1)
figure(2)
% subplot(2,1,1)
plot(t,y,'r')
hold on
plot(t,yc1,'b')
hold on
% subplot(2,1,2)
plot(t,yc2,'g')
hold on
% z-domain plots
plot(t, yc1z, 'k')
hold on
plot(t, yc2z, '-.')
axis([0 5 0 2]);
legend('uncompensated','compensated (roll)', 'compensated (pitch)');