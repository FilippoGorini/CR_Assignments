hudps = dsp.UDPSender('RemoteIPPort',1505);
hudps.RemoteIPAddress = '127.0.0.1';

q = zeros(7,1);

for t = 1:0.1:30
    disp(t);
    hudps([q; q]);   % <-- call object directly
    pause(1);
end
