function o_busCommand = updateOutput(obj)
o_busCommand = [];

%% Exploration mode
if isempty(obj.m_currentObstaclePosition)
    % Find maxima in score grid
    [~,iMaxScore] = max(obj.m_grid_score(:));
    xMaxScore = obj.m_grid_X(iMaxScore);
    yMaxScore = obj.m_grid_Y(iMaxScore);
    
    % Compute command angle
    vectX = xMaxScore - obj.m_position(1);
    vectY = yMaxScore - obj.m_position(2);
    thetaGlobal = atan2(vectY, vectX);
    thetaCommand = thetaGlobal - obj.m_position(3);
    
    % Assign output commands
    o_busCommand.Mode = EMode.SPEED;
    o_busCommand.Angle = single(rad2deg(thetaCommand));
    o_busCommand.Position = single(obj.m_simulationParameters.explorationSpeed);
    
%% Obstacle mode
else 
    % Get obstacle position wrt robot
    obstaclePosition_refRobot = obj.m_currentObstaclePosition(:) - vect(obj.m_position(1:2));
    if obstaclePosition_refRobot(2) >= 0
        thetaCommand = -obj.m_simulationParameters.obstacleAvoidanceAngleInRadian;
    else
        thetaCommand = obj.m_simulationParameters.obstacleAvoidanceAngleInRadian;
    end

    
    % Assign output commands
    o_busCommand.Mode = EMode.SPEED;
    o_busCommand.Angle = single(rad2deg(thetaCommand));
    o_busCommand.Position = single(obj.m_simulationParameters.explorationSpeed);
end

% o_busCommand.Mode = EMode.SPEED;
% o_busCommand.Angle = single(0.1);
% o_busCommand.Position = single(3);
end