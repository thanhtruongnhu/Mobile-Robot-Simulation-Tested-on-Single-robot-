function [filterAngle,angle]= UpdateFilterMatrix(filterAngle,angle)

% Update Matrix
for i = 10:(-1):2 % 20:Oldest data , 1:Newest data
    filterAngle(i) = filterAngle(i-1);
end
filterAngle(1) = angle;    

quad1num = numel(filterAngle(filterAngle>0 & filterAngle< pi/2));
quad4num = numel(filterAngle(filterAngle>(3*pi/2) & filterAngle< 2*pi));

if angle > 3*pi/2 && angle < 2*pi    
    if quad1num >= 3  %&&  quad4num < 10
        idx = find(filterAngle> 0 & filterAngle< pi/2);
        angle = filterAngle(idx(1)); %Taking the latest quadrant I angle
        %     filterAngle(1) = angle;
    end    
elseif angle > 2*pi
    idx = find(filterAngle< 2*pi);
    angle = filterAngle(idx(1)); %Taking the latest 
    filterAngle(1) = angle;    
end
    
    