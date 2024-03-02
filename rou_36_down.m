function rou = rou_36_down(k)
    k=k-0;
    rou_min=33.5;rou_max=75;t1=250;t2=t1+36;t3=t2+36;
    rou=rou_min.*(k<=t1)+(rou_min+((rou_max-rou_min)/(t2-t1))*(k-t1)).*(k>t1 & k<=t2)+(rou_max-((rou_max-rou_min)/(t3-t2))*(k-t2)).*(k>t2 & k<=t3)+rou_min.*(k>t3);
end

