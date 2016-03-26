function updateVisitedSurface(obj)
latency = obj.m_robotParameters.cameraLatencyInCycle;
if (obj.m_currentIndex > 4)
    % réduction du champs de détection (valeurs empirique)
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
    
    [X,Y] = meshgrid(1:160,1:160);
    
    in = inpolygon(X,Y,xRobotAbsolute/5,yRobotAbsolute/5);
    obj.m_grid_explored( 160-Y(in), X(in) ) = 100;
end
end