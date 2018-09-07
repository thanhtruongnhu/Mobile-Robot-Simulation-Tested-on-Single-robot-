function Transmit2Robot_3(Robot)


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
startMarker_out = 250;
endMarker_out   = 250;
[wl_quo, wl_rem, wl_sign, wr_quo, wr_rem, wr_sign] = ...
    ValueConverter4Transmit(Robot(1).w(1),Robot(1).w(2));

fwrite(chn,[startMarker_out wl_quo wl_rem wl_sign wr_quo wr_rem wr_sign endMarker_out],'async');
fprintf('Number of bytes sent:   %.3f\n', chn.BytesToOutput)
fprintf('WL1:   %.3f\n', Robot(1).w(1))
fprintf('WR1:   %.3f\n', Robot(1).w(2))
fprintf('WL1_rounded:   %.3f\n', (wl_quo*254 + wl_rem))
fprintf('WR1_rounded:   %.3f\n', (wr_quo*254 + wr_rem))