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
    obj.m_grid_site( 160-floor(murY/5), floor(murX/5) ) = obj.m_grid_site( 160-floor(murY/5), floor(murX/5) ) - 50;
end

end