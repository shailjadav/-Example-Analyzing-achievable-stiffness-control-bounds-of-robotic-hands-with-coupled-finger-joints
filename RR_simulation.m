%%  RR Serial chain manipulator with series complaince   [June 20, 2020]
% Implementation of  RR case for
%
% P. Rao, G. C. Thomas, L. Sentis and A. D. Deshpande,
% "Analyzing achievable stiffness control bounds of robotic hands with coupled
%  finger joints," 2017 IEEE International Conference on Robotics and Automation
%  (ICRA), Singapore, 2017, pp. 3447-3452, doi: 10.1109/ICRA.2017.7989393.
%
% Script Author : Shail Jadav
% SysIDEA Lab, IIT Gandhinagar
%Website: https://shailjadav.github.io/
%% Clear variables
clear
close all
clc
%% ODE with Intial condition
ts=100;
tspan=0:1/ts:1000;
[t,x]=ode23('odeIVP2R',tspan,[0,pi/120,0,0,0,0,0,0]);

%% Simulation Parameters
rj=0.015; rm=0.015;
ks=15000;
kd1=5;
kd2=3;
kd3=3;
kd4=3;
R=[rj  0; rj rj];
Rm=[rm  0;0  rm];
Ks=[ks 0 ;0 ks ];
Kd=[kd1 kd2;kd3 kd4];
%%  Stiffness Ellipsoid

k1=kd1;
k2=kd2;
C1= 4*ks*rj^2;
C2= ((2*ks*(rj^2)*k1) - (4*ks*rj^4))/(k1 - (4*ks*rj^2));
C=[k1<C1  k2>C2];
A=Kd;
B=(R'*Ks*R);
eig( A -  B)
[xe,ye]=matellips(A);
figure('units','normalized','outerposition',[0 0 1 1])
[x1,y1]=matellips(B);
plot(x1,y1,'r','lineWidth',5)
hold on
plot(xe,ye,'b','lineWidth',4)
legend('Effective stiffness','Desired stiffness','Interpreter','latex');
xlabel('Joint 2 Stiffness','Interpreter','latex')
ylabel('Joint 1 Stiffness','Interpreter','latex')
set(gca,'FontSize',14)
grid minor
axis equal


%% Plots of timeseries

q1=x(:,1);
q2=x(:,3);
m1=x(:,5);
m2=x(:,7);

% Joint Position
figure('units','normalized','outerposition',[0 0 1 1])
subplot(221)
plot(t,q1,'r','LineWidth',1.5)
title(' Joint 1 Position','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor
subplot(222)
plot(t,q2,'b','LineWidth',1.5)
title(' Joint 2 Position','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor
subplot(223)
plot(t,m1,'r','LineWidth',0.5)
title(' Motor 1 Position','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor
subplot(224)
plot(t,m2,'b','LineWidth',0.5)
title(' Motor 2 Position','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor


% FFT of time series
figure('units','normalized','outerposition',[0 0 1 1])
subplot(221)
[f,Mag,l,p]=findfft(q1,1/ts);
plot(f,Mag,'k','LineWidth',1.5)
title(' Joint 1 FFT','Interpreter','latex')
xlabel('frequency (Hz)','Interpreter','latex')
ylabel('Magnitude ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor

subplot(222)
[f,Mag,l,p]=findfft(q2,1/ts);
plot(f,Mag,'k','LineWidth',1.5)
title(' Joint 2 FFT','Interpreter','latex')
xlabel('frequency (Hz)','Interpreter','latex')
ylabel('Magnitude ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor

subplot(223)
[f,Mag,l,p]=findfft(m1,1/ts);
plot(f,Mag,'k','LineWidth',1.5)
title(' Motor 1 FFT','Interpreter','latex')
xlabel('frequency (Hz)','Interpreter','latex')
ylabel('Magnitude ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor

subplot(224)
[f,Mag,l,p]=findfft(m2,1/ts);
plot(f,Mag,'k','LineWidth',1.5)
title(' Motor 2 FFT','Interpreter','latex')
xlabel('frequency (Hz)','Interpreter','latex')
ylabel('Magnitude ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor

%  Animation of RR serial chain manipulator
figure('units','normalized','outerposition',[0 0 1 1])
c=1;
for i=1:100:length(x)
    theta1=x(i,1);
    dtheta1=x(i,2);
    theta2=x(i,3);
    dtheta2=x(i,4);
    
    l1=1; %Input the l length
    l2=1; %Input the l length
    
    % Homogeneus transformation matrix
    H01 = [cos(theta1) -sin(theta1) 0 l1*cos(theta1);sin(theta1) cos(theta1) 0 l1*sin(theta1);0 0 1 0;0 0 0 1]; %Frame 0 to 1 tranformation
    H12 = [cos(theta2) -sin(theta2) 0 l2*cos(theta2);sin(theta2) cos(theta2) 0 l2*sin(theta2);0 0 1 0;0 0 0 1]; %Frame 1 to 2 tranformation
    
    H02=H01*H12;      %Frame 0 to 2 tranformation
    
    O=[0,0];                   %Joint 1 position
    P1=[H01(1,4) H01(2,4)];    %Joint 2 position
    P2=[H02(1,4) H02(2,4)];    %Joint 3 position
    
    Orn= atan2(H02(2,1),H02(1,1));  %Orientation of end effector
    Orn=(Orn)*(180/pi);
    
    plot(P1(1),P1(2),'ok','LineWidth',5)
    hold on
    plot(P2(1),P2(2),'om','LineWidth',5)
    plot(0,0,'ok','LineWidth',10)
    xlim([-2.5 2.5])
    ylim([-2.5 2.5])
    axis square
    grid minor
    plot([0 P1(1)], [0 P1(2)],'r','LineWidth',5)
    plot([P1(1) P2(1)], [P1(2) P2(2)],'b','LineWidth',5)
    hold off
    title(strcat('Time = ',num2str(t(i,1))),'Interpreter','latex')
    xlabel('X axis (m)','Interpreter','latex')
    ylabel('Y axis (m)','Interpreter','latex')
    set(gca,'FontSize',18)
    drawnow
    F(c) = getframe(gcf) ;
    c=c+1;
end

