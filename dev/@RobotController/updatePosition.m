function updatePosition(obj, p_encodersCountDouble)
% Gestion de l'odometrie
% http://geonobotwiki.free.fr/doku.php?id=robotics:odometrie
encoders = double(p_encodersCountDouble-obj.m_old_encoders) * obj.m_robotParameters.wheelRadius * 2 * pi / obj.m_robotParameters.encodersResolution;
obj.m_old_encoders = p_encodersCountDouble;

dR = ( encoders(1) + encoders(2)) /2;
dTheta = ( encoders(2) - encoders(1) ) / obj.m_robotParameters.entraxe;

R = dR / dTheta;
xO = obj.m_position(1) - R * sin(obj.m_position(3));
yO = obj.m_position(2) + R * cos(obj.m_position(3));

if (encoders(1) == encoders(2))
    obj.m_position(1) = obj.m_position(1) + dR * cos(obj.m_position(3));
    obj.m_position(2) = obj.m_position(2) + dR * sin(obj.m_position(3));
else
    obj.m_position(3) = obj.m_position(3) + dTheta;
    obj.m_position(1) = xO + R * sin(obj.m_position(3));
    obj.m_position(2) = yO - R * cos(obj.m_position(3));
end

assert(abs(obj.m_position(1))<obj.m_grid_X(end,end) && abs(obj.m_position(2))<obj.m_grid_Y(end,end), 'Position is out of grid bounds');
obj.m_positionsHistory(obj.m_currentIndex, :) = obj.m_position;
obj.m_encodersHistory(obj.m_currentIndex, :) = encoders;

end