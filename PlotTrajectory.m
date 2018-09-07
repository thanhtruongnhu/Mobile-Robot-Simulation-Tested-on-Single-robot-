function PlotTrajectory(Robot,nb,colorVec)

for id = 1:nb         
plot(Robot(id).x(1),Robot(id).x(2),'Marker','+','Color',colorVec(id,:)) 
drawnow
end

end