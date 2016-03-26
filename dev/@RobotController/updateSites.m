function updateSites(obj,p_distances,p_bearings)
obj.m_currentSitesPosition = nan(6,2);
latency = obj.m_robotParameters.cameraLatencyInCycle;
for i=1:6
    if (p_distances(i) ~= 0 && obj.m_currentIndex > 4)
        % obj.histPos(obj.m_currentIndex-3,:) => compensation de la latence
        Theta = -obj.m_positionsHistory(obj.m_currentIndex-latency, 3) - double(p_bearings(i))*pi/180 + pi/2;
        obsX = obj.m_positionsHistory(obj.m_currentIndex-latency, 1) + double(p_distances(i)) * sin( Theta );
        obsY = obj.m_positionsHistory(obj.m_currentIndex-latency, 2) + double(p_distances(i)) * cos( Theta );
        
        % Boundaries
        if (obsX < 1)
            obsX = 1;
        elseif (obsX > 800)
            obsX = 800;
        end
        
        if (obsY < 1)
            obsY = 1;
        elseif (obsX > 800)
            obsY = 800;
        end
        
        obj.m_grid_site( 160-floor(obsY/5), floor(obsX/5) ) = obj.m_grid_site( 160-floor(obsY/5), floor(obsX/5) ) + 10;
        
        obj.m_currentSitesPosition(i,1)=obsX;
        obj.m_currentSitesPosition(i,2)=obsY;
        
    end
end
end