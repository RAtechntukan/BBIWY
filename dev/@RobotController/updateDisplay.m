function updateDisplay(obj)
%Plot trajectory
axes(obj.m_displayData.axes_positions);
hold off
plot(obj.m_positionsHistory(1:obj.m_currentIndex, 1), obj.m_positionsHistory(1:obj.m_currentIndex, 2) );
hold on;
%Plot de l'orientation actuel
quiver(obj.m_position(1),obj.m_position(2), ...
    sin(obj.m_position(3) + pi/2), -cos(obj.m_position(3) + pi/2), 30);

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

xlim([0 800]);
ylim([0 800]);

% Exploration grids
axes(obj.m_displayData.axes_gridExplored);
imagesc(obj.m_grid_explored); axis equal xy;
caxis([0, 200]);
colormap('jet');
colorbar;

axes(obj.m_displayData.axes_gridSite);
imagesc(obj.m_grid_site); axis equal xy;
colormap('jet');
caxis([0, 200]);
colorbar;


end