function sat_A = sat_rm(A)
    if A>=1
        sat_A=1;
    elseif A<=0
        sat_A=0;
    else
        sat_A=A;
    end
end

