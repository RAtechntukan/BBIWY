function o_busCommand = updateOutput(obj)
%% Compute scoremap
distanceMap = sqrt((obj.m_grid_X - obj.m_position(1)).^2 + (obj.m_grid_Y - obj.m_position(2)).^2 );
distanceMap = exp(-distanceMap / obj.m_simulationParameters.expDistanceMap);

angleMap = mod(atan2((obj.m_grid_Y-obj.m_position(2)), (obj.m_grid_X-obj.m_position(1))), 2*pi) - mod(obj.m_position(3), 2*pi);
angleMap = mod(atan2(obj.m_grid_Y-obj.m_position(2), obj.m_grid_X-obj.m_position(1)), 2*pi);
angleMap = abs(mod(angleMap-mod(obj.m_position(3), 2*pi), 2*pi)-pi);
obstacleMap = 1./(obj.m_grid_obstacles+0.01);
explorationMap = conv2(1-obj.m_grid_explored, obj.m_simulationParameters.gaussianKernel, 'same');
obj.m_grid_score = distanceMap .* obstacleMap .* explorationMap .* angleMap;

%obj.m_grid_score(1:91, :) = 0;
%obj.m_grid_score(:, 1:91) = 0;


o_busCommand = [];

%On prend comme centre de la distanceMap la zone visible par la camera
Theta = -obj.m_position(3) + pi/2;
visionX = obj.m_position(1) + 70 * sin( Theta );
visionY = obj.m_position(2) + 70 * cos( Theta );

distanceMap = sqrt((obj.m_grid_X - visionX).^2 + (obj.m_grid_Y - visionY).^2 );
distanceMap = exp(-distanceMap / obj.m_simulationParameters.expDistanceMap);
[~,iRobot] = min(abs(obj.m_position(2)-obj.m_grid_Y(:,1)));
[~,jRobot] = min(abs(obj.m_position(1)-obj.m_grid_X(1,:)));
offset = (size(obj.m_simulationParameters.gaussianKernel, 1)-1) /2;
maxGaussianKernel = max(max(obj.m_simulationParameters.gaussianKernel));
invKernel = ((maxGaussianKernel - obj.m_simulationParameters.gaussianKernel)./maxGaussianKernel);
distanceMap(iRobot-offset:iRobot+offset,jRobot-offset:jRobot+offset) =  invKernel.* distanceMap(iRobot-offset:iRobot+offset,jRobot-offset:jRobot+offset);

%angleMap = mod(atan2((obj.m_grid_Y-obj.m_position(2)), (obj.m_grid_X-obj.m_position(1))), 2*pi) - mod(obj.m_position(3), 2*pi);
angleMap = mod(atan2(obj.m_grid_Y-obj.m_position(2), obj.m_grid_X-obj.m_position(1)), 2*pi);
angleMap = abs(mod(angleMap-mod(obj.m_position(3), 2*pi), 2*pi)-pi);
angleMap = angleMap + 0.1;

%obstacleMap = 1./(obj.m_grid_obstacles+0.01);
obstacleMap = single(obj.m_grid_obstacles < 0.01);

explorationMap = conv2(1-obj.m_grid_explored, obj.m_simulationParameters.gaussianKernel, 'same');
obj.m_grid_score = distanceMap .* obstacleMap .* explorationMap .* angleMap;

obj.m_grid_score = conv2(obj.m_grid_score, obj.m_simulationParameters.gaussianKernel, 'same');
% obj.m_grid_score(:,1:91) = 0; 
% obj.m_grid_score(1:91, :) = 0; 
% obj.m_grid_score(:,140:end) = 0; 
% obj.m_grid_score(140:end, :) = 0; 

%% Exploration mode
%if isempty(obj.m_currentObstaclePosition)
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
    o_busCommand.Angle = single(rad2deg(-thetaCommand));
    %if ( o_busCommand.Angle < 90)
        o_busCommand.Position = single(obj.m_simulationParameters.explorationSpeed);
    %else
    %    o_busCommand.Position = (200-o_busCommand.Angle)/110*single(obj.m_simulationParameters.explorationSpeed);
    %end
    
% %% Obstacle mode
%  else 
%     % Get obstacle position wrt robot
%     obstaclePosition_refRobot = obj.m_currentObstaclePosition(:) - vect(obj.m_position(1:2));
%     if obstaclePosition_refRobot(2) >= 0
%         thetaCommand = -obj.m_simulationParameters.obstacleAvoidanceAngleInRadian;
%     else
%         thetaCommand = obj.m_simulationParameters.obstacleAvoidanceAngleInRadian;
%     end
% 
%     
%     % Assign output commands
%     o_busCommand.Mode = EMode.SPEED;
%     o_busCommand.Angle = single(rad2deg(thetaCommand)+30);
%     o_busCommand.Position = single(obj.m_simulationParameters.explorationSpeed);
% end

% o_busCommand.Mode = EMode.SPEED;
% o_busCommand.Angle = single(0.1);
% o_busCommand.Position = single(3);

end