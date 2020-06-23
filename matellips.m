function [Xo,Yo] = matellips(A)

[evt,evl]=eig(A);
 
th1=subspace([1;0],evt(:,1));
th2=subspace([1;0],evt(:,2));

th= min(th1,th2);

EVV=[evl(1,1); evl(2,2)];

a=abs(min(EVV)); % horizontal radius
b=abs(max(EVV)); % vertical radius
x0=0; % x0,y0 ellipse centre coordinates
y0=0;
t=-pi:0.01:pi;
x=x0+a*cos(t);
y=y0+b*sin(t);
R=[ cos(th) -sin(th) ; sin(th) cos(th)];
P= R*[x;y];
Xo= P(1,:);
Yo= P(2,:);
end
