function image_output = getProjObstacles(obj)
%obj = RobotController.getInstance;
obstaclesRelative = obj.m_histObstacle(1:obj.m_indexObstacle-1, :) - ones(obj.m_indexObstacle-1, 1)*obj.m_position(1:2);
[theta, rho] = cart2pol(obstaclesRelative(:, 1), obstaclesRelative(:, 2));

distanceMax = 800;
ThetaRes = 30;
Rho_res = distanceMax/obj.m_simulationParameters.resolution;
polarObs = zeros(Rho_res, ThetaRes);

%i = Y;
%j = X;
jPol = floor(mod(theta, 2*pi)./(2*pi)*size(polarObs, 2));
iPol = floor(rho./(distanceMax/size(polarObs, 1)));

offset = 4;
for i=1:obj.m_indexObstacle-1
    imin = min(iPol(i)-1, offset);
    imax = min(size(polarObs, 1)-iPol(i), offset);
    jmin = min(jPol(i)-1, offset);
    jmax = min(size(polarObs, 2)-jPol(i), offset);
    
    polarObs(iPol(i)-imin:iPol(i)+imax,jPol(i)-jmin:jPol(i)+jmax) = ...
    polarObs(iPol(i)-imin:iPol(i)+imax,jPol(i)-jmin:jPol(i)+jmax) + obj.m_simulationParameters.gaussianKernel(5-imin:5+imax, 5-jmin:5+jmax);
end

%figure;imagesc(polarObs);axis xy;

polarObs = cumsum(polarObs);

%polarObs = conv2(polarObs, obj.m_simulationParameters.gaussianKernel, 'same');

%figure;imagesc(polarObs);axis xy;

theta_vect = double(0:2*pi/size(polarObs, 2):2*pi*(size(polarObs, 2)-1)/size(polarObs, 2));
rho_vect = double(0:(distanceMax/size(polarObs, 1)):distanceMax-(distanceMax/size(polarObs, 1)));

[mesh_theta, mesh_rho] = meshgrid(theta_vect, rho_vect);
[mesh_pol_x, mesh_pol_y] = pol2cart(mesh_theta, mesh_rho);

x_vect = -distanceMax:obj.m_simulationParameters.resolution:+distanceMax;
y_vect = -distanceMax:obj.m_simulationParameters.resolution:+distanceMax;
[mesh_x, mesh_y] = meshgrid(x_vect, y_vect);

res = griddata(mesh_pol_x(:), mesh_pol_y(:), polarObs(:), mesh_x, mesh_y, 'cubic');

indice = round(obj.m_position(1:2)/obj.m_simulationParameters.resolution+91);

imin = min(indice(2)-1, size(polarObs, 1));
imax = min(181-indice(2), size(polarObs, 1));
jmin = min(indice(1)-1, size(polarObs, 1));
jmax = min(181-indice(1), size(polarObs, 1));

image_output = ones(181, 181);

image_output(indice(2)-imin:indice(2)+imax, indice(1)-jmin:indice(1)+jmax) = res(size(polarObs, 1)+1-imin:size(polarObs, 1)+1+imax,size(polarObs, 1)+1-jmin:size(polarObs, 1)+1+jmax);

image_output(isnan(image_output)) = 0;

%res = reshape(res, size(mesh_x));

%Pas plus rapide...
%F = scatteredInterpolant(mesh_pol_x(:), mesh_pol_y(:), polarObs(:));
%res = F(mesh_x, mesh_y);
%toc;
%figure;imagesc(polarObs);axis xy;
%figure;imagesc(res);axis xy;
end