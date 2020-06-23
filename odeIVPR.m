function Out= odeIVPR(t,x)
%%  R Serial chain manipulator with series complaince   June 20, 2020
% Author : Shail Jadav
% SysIDEA Lab, IIT Gandhinagar
%Website: https://shailjadav.github.io/
% System parameters
Ks=1000;
Kd=6;
tr=0.01;
m=0.3;
l=0.5;
B=0.01;
rj=0.05;
rm=0.05;

I=(m*l*l)/3;
qd=0;

R=[rj ; -rj];
Rm=[rm ; 0];
q=(x(1));
thM=(x(3));

qd=0;
%% Dynamics Equation
thMD=(pinv(Rm)*((R*q) + inv(Ks)*pinv(R')*Kd*(qd-q)));
dq=inv(I)*((R'*(Ks*((Rm*thM) - (R*q))))  - B*x(2) );
dthM= (1/tr)*(thMD -thM);

OP=zeros(4,1);
OP(1)=x(2);
OP(2)=dq(1);
OP(3)=x(4);
OP(4)=dthM(1);
Out=OP;
end

