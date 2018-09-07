% Test Robot 2

% Robot(id).w(1)=left wheel. (Rad/s)
% Robot(id).w(2)=right wheel.(Rad/s)
%% Reset all left-over settings
if ~isempty(instrfind)
     fclose(instrfind);
      delete(instrfind);
end

%% Set up COM ports
chn = serial('COM6');
chn.OutputBufferSize = 500;
fopen(chn);
disp('Start Writing')

%% Trasmitting data to robots (Writing data to 9xstream modem)
startMarker_out = 179;
endMarker_out   = 179;
f2 = 0.865; % Calibrate wl parameter:: Cuz Left-wheel motor is faster.
wl = 0; %Khong chay o nguong 40 50 55 (Chay o nguong 20-30 &  >= 60)
wr = 0; %Khong chay o nguong 35 40 50 55 (Chay o nguong 20-30 &  >= 60)

% [wl,wr]=AntiExcessiveRotate(wl,wr);
[wl,wr] = WheelSpeedModify(wl,wr);
[wl_quo, wl_rem, wl_sign, wr_quo, wr_rem, wr_sign] = ...
    ValueConverter4Transmit(f2*wl,wr); 
    
fwrite(chn,[startMarker_out wl_quo wl_rem wl_sign wr_quo wr_rem wr_sign endMarker_out],'async');
fprintf('Number of bytes sent:   %.3f\n', chn.BytesToOutput)
% fprintf('WL1:   %.3f\n', f2*Robot(1).w(1))
% fprintf('WR1:   %.3f\n', Robot(1).w(2))
fprintf('WL1_rounded:   %.3f\n', (wl_quo*254 + wl_rem))
fprintf('WR1_rounded:   %.3f\n', (wr_quo*254 + wr_rem))