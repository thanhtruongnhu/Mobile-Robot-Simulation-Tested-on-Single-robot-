classdef Robot_2
   properties 
      x         % gia tri ban dau
      angle     % gia tri ban dau
      v         % duoc tinh toan dua vao toa do cua 2 waypoint     
      w         % [wl wr] van toc 2 banh: w1 banh trai, w2 banh phai
      waypoints % tap cac waypoints cua tung Robot
   end
        
   methods
        function obj = Robot_2(waypoints,F,nb,nw) %Input 1 ma tran voi 5 thanh phan: x dot, y dot...
%           ,x (initial), y(initial), angle(goc heading ban dau)   
            if nargin ~= 0            
            obj(nb) = Robot_2;
            
                for id = 1:nb            
                    obj(id).x(1)    = F(id,1);
                    obj(id).x(2)    = F(id,2);
                    obj(id).angle   = F(id,3);
                    obj(id).v(1)    = F(id,4);
                    obj(id).v(2)    = F(id,5);
                    obj(id).w(1)    = 0;
                    obj(id).w(2)    = 0;
                    for k = 1:nw
                    obj(id).waypoints(k,:) = waypoints(k, id*2-1 : id*2);
                    end
                end
            end
        end
    end
end