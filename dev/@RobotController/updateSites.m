function updateSites(obj,p_distances,p_bearings)
obj.m_currentSitesPosition = nan(6,2);
latency = obj.m_robotParameters.cameraLatencyInCycle;
for i=1:6
    if (p_distances(i) ~= 0 && obj.m_currentIndex > 4)
        % obj.histPos(obj.m_currentIndex-3,:) => compensation de la latence
        Theta = -obj.m_positionsHistory(obj.m_currentIndex-latency, 3) - double(p_bearings(i))*pi/180 + pi/2;
        obsX = obj.m_positionsHistory(obj.m_currentIndex-latency, 1) + double(p_distances(i)) * sin( Theta );
        obsY = obj.m_positionsHistory(obj.m_currentIndex-latency, 2) + double(p_distances(i)) * cos( Theta );
        assert(abs(obsX)<obj.m_grid_X(end,end) && abs(obsY)<obj.m_grid_Y(end,end), 'Position is out of grid bounds');
        
        [~,iSite] = min(abs(obsY-obj.m_grid_Y(:,1)));
        [~,jSite] = min(abs(obsX-obj.m_grid_X(1,:)));
        obj.m_grid_obstacles(iSite,jSite) = 1;
        
        obj.m_currentSitesPosition(i,1)=obsX;
        obj.m_currentSitesPosition(i,2)=obsY;
        
    end
end
end