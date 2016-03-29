function updateDisplay(obj)
% Figure setup
if isempty(obj.m_figHandle) || ~isgraphics(obj.m_figHandle)
    obj.m_figHandle = figure;
    obj.m_displayData.axes_positions = subplot(221);
    obj.m_displayData.axes_gridExplored = subplot(222);
    obj.m_displayData.axes_gridSite = subplot(223);
    obj.m_displayData.axes_gridObstacles = subplot(224);
end

%Plot trajectory
axes(obj.m_displayData.axes_positions);
hold off
plot(obj.m_positionsHistory(1:obj.m_currentIndex, 1), obj.m_positionsHistory(1:obj.m_currentIndex, 2) );
hold on;
%Plot de l'orientation actuel
% quiver(obj.m_position(1),obj.m_position(2), ...
%     sin(obj.m_position(3) + pi/2), -cos(obj.m_position(3) + pi/2), 30);
quiver(obj.m_position(1),obj.m_position(2), ...
    cos(obj.m_position(3)), sin(obj.m_position(3)), 30);


%Plot de l'obstacle actuel
if ~isempty(obj.m_currentObstaclePosition)
    quiver(obj.m_position(1),obj.m_position(2), ...
        obj.m_currentObstaclePosition(1) - obj.m_position(1), obj.m_currentObstaclePosition(2) - obj.m_position(2), 1, ...
        'color', 'green');
end

% Plot currently seen sites
for i=1:6
    if ~any(isnan(obj.m_currentSitesPosition(i,:)))
        quiver(obj.m_position(1),obj.m_position(2), ...
            obj.m_currentSitesPosition(i,1) - obj.m_position(1), obj.m_currentSitesPosition(i,2) - obj.m_position(2), 1);
    end
end
xdata = obj.m_grid_X([1 end]);
ydata = obj.m_grid_Y([1 end]);
xlim(xdata);
ylim(ydata);

% Exploration grids
axes(obj.m_displayData.axes_gridExplored);
imagesc(obj.m_grid_explored, 'xdata',xdata,'ydata',ydata); axis equal xy;
caxis([0, 2]);

% Grid site
axes(obj.m_displayData.axes_gridSite);
imagesc(obj.m_grid_score, 'xdata',xdata,'ydata',ydata); axis equal xy;

% Grid obstacles
axes(obj.m_displayData.axes_gridObstacles);
imagesc(obj.m_grid_obstacles, 'xdata',xdata,'ydata',ydata); axis equal xy;
caxis([0, 2]);

end