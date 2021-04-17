function plotAnimation(map,pathWaypoints,trace, offload)

if offload == 1
    lineSpec = 'r:'; % red path if RRT
else
    lineSpec = 'g:'; % green path if A*
end

hold off;
show(map);
hold all;
plot(pathWaypoints(:,1), pathWaypoints(:,2),lineSpec,'LineWidth',2);
hold all
plot(trace(:,1), trace(:,2), 'lineWidth',2, 'color', [0.4940 0.1840 0.5560]);
hold all;
   
end

