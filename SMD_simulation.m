%%  SMD  with series complaince   [June 20, 2020]
% Implementation of  smd case for
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
%% ODE
ts=100;
tspan=0:1/ts:100;
[t,x]=ode45('odeR',tspan,[1,0,0,0]);
pos=x(:,1);
vel=x(:,2);

%% Timeseries and FFT plots
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
keff=(p*2*pi)^2;
plot(f,Mag,'k','LineWidth',1.5)
title(' FFT','Interpreter','latex')
xlabel('frequency (Hz)','Interpreter','latex')
ylabel('Magnitude ','Interpreter','latex')
legend(strcat('Keff',num2str(keff)))
set(gca,'FontSize',14)
grid minor
