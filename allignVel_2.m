function Va = allignVel_2(id,Robot)
global count
 
 Va = Robot(id).waypoints(count(id),:) - Robot(id).x;     
 
 Va = (Va/norm(Va));   %Assume that the robot always travel with 0.4 m/s velocity
            