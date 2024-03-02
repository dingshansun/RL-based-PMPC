function demand = demando1(k)
%DEMANDO2 Summary of this function goes here
%   Detailed explanation goes here
% k=k-1;
k=k-60;
% demand=0;
max=1500;
min=500;
demand=750.*(k<0)+(min+(max-min)/(40-0)*k).*(k>=0 & k<=40)+max.*(k>40 & k<130)+(max-(max-min)/(170-130)*(k-130)).*(k>=130 & k<=170)+min.*(k>170);
end

