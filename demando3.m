function demand = demando3(k)
%DEMANDO2 Summary of this function goes here
%   Detailed explanation goes here
% k=k-1;
k=k-60;
% demand=0;
max=1400;
min=600;
t1=0;t2=t1+40;t3=t2+90;t4=t3+40;
demand=500.*(k<0)+min.*(k>=0 & k<t1)+(min+(max-min)/(t2-t1)*(k-t1)).*(k>=t1 & k<=t2)+max.*(k>t2 & k<=t3)...
    +(max-(max-min)/(t4-t3)*(k-t3)).*(k>t3 & k<=t4)+min.*(k>t4);
end

