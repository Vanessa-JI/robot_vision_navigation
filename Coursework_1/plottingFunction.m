function plottingFunction(Method, time, longitude, latitude, vN, vE)
%% Positioning plot 
figure
plot(longitude, latitude, '-m');
title(Method, ' Position');
xlabel('Longitude (°)');
ylabel('Latitude (°)');
grid on;
saveas(gcf,['Results\', Method,'_Pos.png'])

%% Velocity plot
figure
plot(time, vN, '-m', ...
    time, vE, '--b');
legend('Velocity North', 'Velocity East');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title(Method, ' Velocities');
grid on;
saveas(gcf,['Results\', Method,'_Vel.png'])

%% Quiver plot
figure
quiver(longitude,latitude,vE,vN)
grid on;
xlabel('Longitude (°)');
ylabel('Latitude (°)');
title(['Quiver Plot of ', Method ,' Results'])
saveas(gcf,['Results\', Method ,'_Quiver.png'])

%% Positioning geoplot , ensure lat, long format
figure
geoplot(latitude, longitude, '-m');
title(Method, ' GeoPlot Position');
grid on;
saveas(gcf,['Results\', Method,'_Geoplot.png'])

end
