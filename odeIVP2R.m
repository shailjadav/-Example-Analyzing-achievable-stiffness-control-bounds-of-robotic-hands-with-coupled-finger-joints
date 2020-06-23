function Out= odeIVP2R(t,x)
%%  RR Serial chain manipulator with series complaince   June 20, 2020
% Author : Shail Jadav
% SysIDEA Lab, IIT Gandhinagar
%Website: https://shailjadav.github.io/
%%  System parameters
m2=1; m1=1; l1=1; l2=1; b=0.01; g=0; rj=0.015; rm=0.015; tr=0.001;

ks=15000; % Series complaince 

kd1=5; % desired stiffness
kd2=3;
kd3=3;
kd4=3;

qd=[0;0]; % virtual equilibrium 

 R=[rj  0; rj rj];          %tendon jacobian
 Rm=[rm  0;0  rm];  %motor map
 Ks=[ks 0 ;0 ks ];      
 Kd=[kd1 kd2;kd3 kd4];

q=([x(1);x(3)]);              % States Q1 Q2 Joint angles
thM=([x(5) ; x(7)]);       % States M1 M1 Motor angles


%% System Dynamics

%M matrix
M11=((((m1/3) + m2)*l1^2) +((m2/3)*l2^2) + (m2*l1*l2*cos(x(3))));
M12=(m2*(((l2^2)/3) + (0.5*l1*l2*cos(x(3)))));
M21=M12;
M22=((1/3)*m2*l2*l2);

%H matrix
H1 =((-m2*l1*l2*sin(x(3))*x(2)*x(4)) - (0.5*m2*l1*l2*sin(x(3))*x(4)*x(4)));
H2 = (0.5 * m2* l1*l2*sin(x(3))*x(2)*x(2));

%G matrix
G1=( ((((0.5*m1) + m2)*l1*cos(x(1))) + (0.5*m2*l2*cos(x(1)+x(3))))*g);
G2=0.5*m2*l2*cos(x(1)+x(3))*g;

% B matrix
B1 = b*x(2);
B2 = b*x(4);

M=[M11 M12;M21 M22];
HG = [H1 + G1; H2 + G2];
B=[B1 ; B2];

%% Dynamics Equation
thMD=(pinv(Rm)*((R*q) + inv(Ks)*pinv(R')*Kd*(qd-q)));
dq=inv(M)*((R'*(Ks*((Rm*thM) - (R*q))))  - B );
dthM= (1/tr)*(thMD -thM);


%% State output 
OP=zeros(8,1);

OP(1)=x(2);     %q1
OP(2)=dq(1);  %dq1
OP(3)=x(4);     %q2
OP(4)=dq(2);  %dq2

OP(5)=x(6);             %M1
OP(6)=dthM(1);     %dM1
OP(7)=x(8);             %M2
OP(8)=dthM(2);     %dM2


Out=OP;
end

%%  Check Condition for the stability
% k1=kd1;
% k2=kd2;
% C1= 4*ks*rj^2
% C2= ((2*ks*(rj^2)*k1) - (4*ks*rj^4))/(k1 - (4*ks*rj^2))
% A=[k1<C1  k2>C2]
% eig(Kd - (R'*Ks*R))

