%%  R Serial chain manipulator with series complaince   [June 20, 2020]
% Implementation of  R case for
%
% P. Rao, G. C. Thomas, L. Sentis and A. D. Deshpande,
% "Analyzing achievable stiffness control bounds of robotic hands with coupled
%  finger joints," 2017 IEEE International Conference on Robotics and Automation
%  (ICRA), Singapore, 2017, pp. 3447-3452, doi: 10.1109/ICRA.2017.7989393.
%
% Script Author : Shail Jadav
% SysIDEA Lab, IIT Gandhinagar
%Website: https://shailjadav.github.io/

%% Initialization
clear
close all
clc
%% ODE with Intial condition
ts=100;
tspan=0:1/ts:100;

[t,x]=ode45('odeIVPR',tspan,[1,0,0,0]);
pos=x(:,1);
vel=x(:,2);
Mpos=x(:,3);
Mvel=x(:,4);

%% Time series and FFT for Joint Angle
figure('units','normalized','outerposition',[0 0 1 1])
subplot(311)
plot(t,pos,'r','LineWidth',1.5)
title(' Position','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (m) ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor

subplot(312)
plot(t,vel,'b','LineWidth',1.5)
title(' Velocity','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Velocity (m/s) ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor

subplot(313)
[f,Mag,l,p]=findfft(x(:,1),1/ts);
keff=((p*2*pi)^2)*0.0250;
plot(f,Mag,'k','LineWidth',1.5)
title(' FFT','Interpreter','latex')
xlabel('frequency (Hz)','Interpreter','latex')
ylabel('Magnitude ','Interpreter','latex')
legend(strcat('Keff',num2str(keff)))
set(gca,'FontSize',14)
grid minor


%% Time series and FFT for Motor Angle

figure('units','normalized','outerposition',[0 0 1 1])
subplot(311)
plot(t,Mpos,'r','LineWidth',1.5)
title(' Position','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (m) ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor

subplot(312)
plot(t,Mvel,'b','LineWidth',1.5)
title(' Velocity','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Velocity (m/s) ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor

subplot(313)
[f,Mag,l,p]=findfft(x(:,3),1/ts);
keff=((p*2*pi)^2)*0.0250;
plot(f,Mag,'k','LineWidth',1.5)
title(' FFT','Interpreter','latex')
xlabel('frequency (Hz)','Interpreter','latex')
ylabel('Magnitude ','Interpreter','latex')
legend(strcat('Keff',num2str(keff)))
set(gca,'FontSize',14)
grid minor

%% Synchronisation of motor and joint angle

figure('units','normalized','outerposition',[0 0 1 1])
plot(t,Mpos,'r','LineWidth',1.5)
hold on
plot(t,pos,'b','LineWidth',0.5)
title(' Position','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (m) ','Interpreter','latex')
set(gca,'FontSize',14)
grid minor

%% Animation of joint angles
figure('units','normalized','outerposition',[0 0 1 1])
c=1;
for i=1000:1:length(x)
    
    l=2;
    Os2=-5;
    Os1=0;
    
    
    O=[0,0];                   %Joint 1 position
    P1=[Os1 + (l*cos(Mpos(i,1)))  l*sin(Mpos(i,1)) ];    %Joint 2 position
    
    P21=[Os2 + (l*cos(pos(i,1)))  l*sin(pos(i,1)) ];    %Joint 2 position
    
    
    plot(P1(1),P1(2),'ok','LineWidth',5)
    hold on
    CMpos = [-7 -2 4 4];
    CJpos = [-2 -2 4 4];
    rectangle('Position',CMpos,'Curvature',[1 1])
    rectangle('Position',CJpos,'Curvature',[1 1])
    
    
    plot(Os1,0,'ok','LineWidth',10)
    plot([Os1 P1(1)], [0 P1(2)],'r','LineWidth',5)
    plot(P21(1),P21(2),'ok','LineWidth',5)
    plot(Os2,0,'ok','LineWidth',10)
    plot([Os2 P21(1)], [0 P21(2)],'b','LineWidth',5)
    
    
    axis square
    grid minor
    hold off
    title(strcat('Time = ',num2str(t(i,1))),'Interpreter','latex')
    xlabel('X axis (m)','Interpreter','latex')
    ylabel('Y axis (m)','Interpreter','latex')
    set(gca,'FontSize',18)
    xlim([-10 10])
    ylim([-10 10])
    drawnow
    
    
    F(c) = getframe(gcf) ;
    c=c+1;
end
