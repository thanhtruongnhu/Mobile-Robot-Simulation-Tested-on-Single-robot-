function [Robot,stopflag] = StopCondition(Robot,stopflag,nb)
global count
%% Redirect & Stop condition: r_ij_stop = 0.01m
for id = 1:nb
    if stopflag(id)~= 1
        if  norm (Robot(id).waypoints(count(id),:) - Robot(id).x) <= 0.05
            if  count(id) == size(Robot(id).waypoints,1)  % Stop condition 
                stopflag(id) = 1;     
            end
            count(id) = count(id) + 1; % Jump to new waypoints
            

        end
    end
end
end

        
        
        
        