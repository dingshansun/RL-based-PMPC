function demand = demando0(k)
%DEMANDO1 Summary of this function goes here
%   Detailed explanation goes here
% k=k-1;
k=k-60;
Max=3080;
t1=360;
t2=450;
demand=Max.*(k<=t1)+(Max-(Max-1000)/(t2-t1)*(k-t1)).*(k>t1 & k<=t2)+1000.*(k>t2);
end

