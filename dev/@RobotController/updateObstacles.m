function updateObstacles(obj, p_scannerStatus, p_scannerValues)
value = -1;
if (p_scannerStatus == EPosition.LEFT)
    value = p_scannerValues(EPosition.LEFT);
    angle = -obj.m_robotParameters.obstacleDetectorAngleInDegrees*pi/180;
elseif (p_scannerStatus == EPosition.MIDDLE)
    value = p_scannerValues(EPosition.MIDDLE);
    angle = 0;
elseif (p_scannerStatus == EPosition.RIGHT)
    value = p_scannerValues(EPosition.RIGHT);
    angle = obj.m_robotParameters.obstacleDetectorAngleInDegrees*pi/180;
end

obj.m_currentObstaclePosition = [];
if (value ~= -1)
    Theta = -obj.m_position(3) + angle + pi/2;
    murX = obj.m_position(1) + double(value) * sin( Theta );
    murY = obj.m_position(2) + double(value) * cos( Theta );
    obj.m_currentObstaclePosition = [murX; murY];
    
    [~,iObstacle] = min(abs(murY-obj.m_grid_Y(:,1)));
    [~,jObstacle] = min(abs(murX-obj.m_grid_X(1,:)));
    offset = (size(obj.m_simulationParameters.gaussianKernel, 1)-1) /2;
    if(iObstacle + offset > 180)
        iObstacle
        offset
    elseif(iObstacle - offset < 1)
        iObstacle
        offset
    elseif(jObstacle + offset > 180)
        jObstacle
        offset
    elseif(jObstacle - offset < 1)
        jObstacle
        offset
    end
    obj.m_grid_obstacles(iObstacle-offset:iObstacle+offset,jObstacle-offset:jObstacle+offset) = obj.m_grid_obstacles(iObstacle-offset:iObstacle+offset,jObstacle-offset:jObstacle+offset) + obj.m_simulationParameters.gaussianKernel;
end

end