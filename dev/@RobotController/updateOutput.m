function o_busCommand = updateOutput(obj)
o_busCommand = [];
o_busCommand.Mode = EMode.SPEED;
o_busCommand.Angle = single(0.1);
o_busCommand.Position = single(1);

distanceMap = sqrt((obj.m_grid_X - obj.m_position(1)).^2 + (obj.m_grid_Y - obj.m_position(2)).^2 );
distanceMap = exp(-distanceMap / obj.m_simulationParameters.expDistanceMap);

angleMap = mod(atan2((obj.m_grid_Y-obj.m_position(2)), (obj.m_grid_X-obj.m_position(1))), 2*pi) - mod(obj.m_position(3), 2*pi);
angleMap = mod(atan2(obj.m_grid_Y-obj.m_position(2), obj.m_grid_X-obj.m_position(1)), 2*pi);
angleMap = pi - abs(mod(angleMap-mod(obj.m_position(3), 2*pi), 2*pi)-pi);
obstacleMap = 1./(obj.m_grid_obstacles+0.01);
explorationMap = conv2(1-obj.m_grid_explored, obj.m_simulationParameters.gaussianKernel, 'same');
obj.m_grid_score = distanceMap .* obstacleMap .* explorationMap .* angleMap;

end