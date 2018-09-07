function [Robot] = ControlVector_2(Robot,nb,stopflag)

Vs = zeros(1,2);
Vc = zeros(1,2);
gamma = 1; rho = 1;

% The Decisive Parameter (Sigma)
sigma = 0.2; % From 0 to 1 OR 1 to any larger value


for id = 1 : nb
    if stopflag(id) ~= 1
        for k = 1 : nb-1
            if id ~= k
                r_ij = Robot(k).x - Robot(id).x;           
                [Vs_comp,Vc_comp] = SeparateAndCoherent(r_ij);  % Component separate velocity (element velocity)
                Vs = Vs + Vs_comp;  % Vector combination (Separate Velocity)
                Vc = Vc + Vc_comp;  % Vector combination (Coherent Velocity)
            end
        end
                Va = allignVel_2(id,Robot);       % Allignment Velocity

        Robot(id).v = gamma*(sigma*Vc + Va) + rho*Vs; % CONTROL VECTOR

        if norm(Robot(id).v)>1
            Robot(id).v = Robot(id).v/norm(Robot(id).v) ; % Assume that robot velocity is 0.4 m/s
        end    
%         Robot(id).v = Robot(id).v/norm(Robot(id).v) * 0.4; % Assume that robot vel = 0.05 m/s
    else
    Robot(id).v = [0,0]; % Stopflag == 1 => Stop the Robot
    end
end
end
    
    
            