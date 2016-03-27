classdef RobotController < handle
    %% Properties
    properties
        % Static properties
        m_robotParameters = []; % robot Parameters
        m_simulationParameters = [];
        
        % Updateable properties
        m_currentIndex = 0; % Iteration index
        m_position = [0, 0, 0]; % Robot estimated position (x, y, theta(radian))
        m_positionsHistory = zeros(10000, 3); % History of m_position properties
        m_old_encoders = zeros(2, 1); % Last EncodersCount (double)
        m_currentObstaclePosition = []; % Estimated x,y of current obstacle detected position
        m_currentSitesPosition = []; % Estimated x,y of current sites detected positions
        
        % Grid and layers
        m_grid_X = [];
        m_grid_Y = [];
        m_grid_angle = [];
        m_grid_distance = [];
        m_grid_explored = zeros(160, 160); % Meshgrid for grid exploration
        m_grid_site = ones(160, 160)*100; % Meshgrid for sites
        m_grid_obstacles = [];
        m_grid_score = [];
        
        % Mode
        m_explorationMode = 0;
        
        % Display properties
        m_figHandle = [];
        m_displayData = [];
        
    end
    
    %% Methods
    methods
        %% Constructor
        function obj = RobotController   
            % Robot parameters setup
            obj.m_robotParameters = getRobotParametersStruct;
            obj.m_simulationParameters = getSimulationParametersStruct;
            
            % Initialize grids
            obj.m_simulationParameters.mainMeshSize
            xMax = obj.m_simulationParameters.mainMeshSize(1)/2;
            x_vect = single(-xMax:obj.m_simulationParameters.resolution:xMax);
            yMax = obj.m_simulationParameters.mainMeshSize(2)/2;
            y_vect = single(-yMax:obj.m_simulationParameters.resolution:yMax);
            [obj.m_grid_X, obj.m_grid_Y] = meshgrid(x_vect,y_vect);
            obj.m_grid_angle = single(zeros(size(obj.m_grid_X)));
            obj.m_grid_distance = single(zeros(size(obj.m_grid_X)));
            obj.m_grid_explored = single(zeros(size(obj.m_grid_X)));
            obj.m_grid_site = single(zeros(size(obj.m_grid_X)));
            obj.m_grid_obstacles = single(zeros(size(obj.m_grid_X)));
            obj.m_grid_score = single(zeros(size(obj.m_grid_X)));
            
            % Figure setup
            obj.m_figHandle = figure;
            obj.m_displayData.axes_positions = subplot(221);
            obj.m_displayData.axes_gridExplored = subplot(222);
            obj.m_displayData.axes_gridSite = subplot(223);
            
        end
        
        
        %% Various Methods prototypes
        updatePosition(obj, p_encodersCountDouble);
        updateDisplay(obj);
        o_busCommand = updateOutput(obj);
        updateObstacles(obj, p_scannerStatus, p_scannerValues);
        updateSites(obj,p_distances,p_bearings);
        updateVisitedSurface(obj);
        %
        
        
        %% UpdateStep
        % input : p_busRobot
        %     EncodersCount  int32 2x1
        %     Distance       int16 6x1 unité : cm     latence de 200-300ms
        %     Bearing        int8  6x1 unité : degree latence de 200-300ms
        %     ScannerStatus  Enum       left : -25° & right : 25°
        %     ScannerValues  int8  3x1

        % output : busCommand
        %     Mode           Enum
        %     Angle          real
        %     Position       real
        function o_busCommand = updateStep(obj, p_busRobot)
            % Update index
            obj.m_currentIndex = obj.m_currentIndex + 1;
            
            % Update estimated position data
            obj.updatePosition(double(p_busRobot.EncodersCount));
               
            % Gestion des obstacles (capteur de distance)
            obj.updateObstacles(p_busRobot.ScannerStatus, p_busRobot.ScannerValues);
            
            % Gestion de la camera & de la détection de site
            % Gestion des Sites
            obj.updateSites(p_busRobot.Distance, p_busRobot.Bearing);
 
            % Gestion de la surface visité (et vide de site)
            obj.updateVisitedSurface;
            
            % Update display
%             obj.updateDisplay;
            
            %Gestion active des consignes : désactivé dans simulink
            o_busCommand = obj.updateOutput;
            
        end
        
    end
    
    
    
    methods (Static)
        %% getInstance : to get an instance of the object
        function obj = getInstance
            persistent RobotControllerSingleton
            if isempty(RobotControllerSingleton)
                RobotControllerSingleton = RobotController;
            end
            obj = RobotControllerSingleton;
        end
        

    end
end