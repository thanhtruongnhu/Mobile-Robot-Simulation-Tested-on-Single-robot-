function  [wl_mod, wr_mod] = WheelSpeedModify(wl,wr)

% Purpose: Mapping from [0 - 80] rad/s to [30 - 120] pwm
l = abs(wl);
r = abs(wr);

% Left wheel modification
if l ~= 0
    percent = l*100/80;
    wl_mod = 30 + percent*90/100;
else
    wl_mod = 0;
end
if wl < 0
    wl_mod = -wl_mod;
end

% Right wheel modification
if r ~= 0
    percent = r*100/80;
    wr_mod = 30 + percent*90/100;
else
    wr_mod = 0;
end
if wr < 0
    wr_mod = -wr_mod;
end

end