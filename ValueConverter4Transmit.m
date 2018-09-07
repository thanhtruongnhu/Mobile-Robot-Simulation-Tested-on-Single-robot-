function [wl_quo, wl_rem, wl_sign, wr_quo, wr_rem, wr_sign] = ValueConverter4Transmit(wl,wr)


wl_unsign   = round(abs(wl)); wr_unsign = round(abs(wr));
wl_quo = fix(wl_unsign/254); %Transfer the Quotient 
wl_rem = rem(wl_unsign,254); %Transfer the Remainder    
        
wr_quo = fix(wr_unsign/254);
wr_rem = rem(wr_unsign,254);

% (Negative = 200, Positive = 0)
if wl > 0
    wl_sign = 0;
else wl_sign = 200;
end

if wr > 0
    wr_sign = 0;
else wr_sign = 200;
end

end