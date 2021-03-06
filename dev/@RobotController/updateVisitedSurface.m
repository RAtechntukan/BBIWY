function updateVisitedSurface(obj)
latency = obj.m_robotParameters.cameraLatencyInCycle;
if (obj.m_currentIndex > 4)
    % r�duction du champs de d�tection (valeurs empirique)
    lcam = [15 80]; %[21 90];  % largeur near, largeur far
    pcam = [28 90]; %[24 100]; % near - far
    xTrapezRobot = [pcam(1) pcam(2) pcam(2) pcam(1)] ; % trapez x values
    yTrapezRobot = [lcam(1)/2 lcam(2)/2 -lcam(2)/2 -lcam(1)/2] ; % trapez y values
    xyRobotAbsolute = [cos(obj.m_positionsHistory(obj.m_currentIndex-latency, 3)) -sin(obj.m_positionsHistory(obj.m_currentIndex-latency, 3)) ; ...
        sin(obj.m_positionsHistory(obj.m_currentIndex-latency, 3)) cos(obj.m_positionsHistory(obj.m_currentIndex-latency, 3))] * ...
        [xTrapezRobot;yTrapezRobot]; % rotation of robotheta
    
    xRobotAbsolute = xyRobotAbsolute(1,:)'+obj.m_positionsHistory(obj.m_currentIndex-latency, 1);
    yRobotAbsolute = xyRobotAbsolute(2,:)'+obj.m_positionsHistory(obj.m_currentIndex-latency, 2);
    %fill(xRobotAbsolute, yRobotAbsolute, 'r');
    
    
    in = inpolygon(obj.m_grid_X, obj.m_grid_Y, xRobotAbsolute,yRobotAbsolute);
    obj.m_grid_explored( in ) = 1;
end
end