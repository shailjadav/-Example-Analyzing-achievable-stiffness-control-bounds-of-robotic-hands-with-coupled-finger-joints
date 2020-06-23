function [f,Mag,mx,PF]=findfft(x,ts)
Fs = 1/ts;
T =  ts;
L = length(x);
x=x-mean(x);
Y = fft(x);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
Mag=P1;

[pks,locs] = findpeaks(Mag,f);

Md=[pks';locs];

mx=max(pks);

if(size(Md,2)>1)
    for i=1:1:length(Md)
        if(Md(1,i)==mx)
            PF=Md(2,i);
        end
    end
end

if(size(Md,2)==1)
    PF=Md(2,1);   
end
end