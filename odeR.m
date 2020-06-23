function Out= odeR(t,x)
%%  R Serial chain manipulator with series complaince   June 20, 2020
% Author : Shail Jadav
% SysIDEA Lab, IIT Gandhinagar
%Website: https://shailjadav.github.io/
% System parameters
M=1;
B=0.2;
Kp=5;
Ks=30;
Kd=33;
tr=0.01;
X1d=0;

%KdAS= (((B*Kp*tr)/M) + ((B*Ks*tr)/M) + ((B^2)/M) + (Kp + Ks) + (B/tr))
%% Dynamics equation

P2d=((Kd*(X1d - x(1)) + Kp*x(1))/Ks) + x(1);
dx2=inv(M)*( (Ks*(x(3) - x(1))) - B*x(2) -Kp*x(1));
dx4=inv(tr)*(P2d -x(3));



%% Output States
OP=zeros(4,1);
OP(1)=x(2);
OP(2)=dx2(1);
OP(3)=x(4);
OP(4)=dx4(1);


Out=OP;
end