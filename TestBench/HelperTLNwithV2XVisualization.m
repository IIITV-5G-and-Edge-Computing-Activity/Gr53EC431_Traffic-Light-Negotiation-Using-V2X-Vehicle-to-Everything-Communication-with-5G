classdef HelperTLNwithV2XVisualization < matlab.System
    % HelperTLNWithV2XVisualization Visualization helper class.
    % This is a helper class which plots the run time simulation result
    % for the Traffic Light Negotiation with V2X example.
    
    % NOTE: The name of this System Object and its functionality may
    % change without notice in a future release,
    % or the System Object itself may be removed.
    
    % Copyright 2021-2023 The MathWorks, Inc.
    
    properties
        % V2X Range
        range = 300;
    end
    properties(Nontunable)
        % Enable Traffic Light Visualization
        TrafficLightDisp (1, 1) logical = true;
        % Enable Visualization
        EnableVisualization (1,1) logical = true;
    end
    
    properties(Access = private)
        %Figure Store the figure handle.
        Figure;

        % Axes handle for traffic light simulation
        AxesHandle;
        % Axes handle for chase plot
        AxesHandleChasePlot;
        % Axes handle for birds eye plot
        AxesHandleBEP;

        % Birds Eye Plot
        BEP
        % Detection Plot
        DetectionPlot
        % Outline Plot
        OutlinePlot
        % Lane Plot
        LaneMarkingPlot
        % Coverage area plot
        CoverageAreaPlot
        % MIO Plot
        MIOPlt

        %Scenario Store scenario variable from the workspace.
        Scenario;
        
        %TrafficLightGraphicObjects Store graphic objects to change marker
        %face color.
        TrafficLightGraphicObjects = [];
    end
    
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj,varargin)
            if obj.EnableVisualization
                %setupImpl Perform one-time calculations.
                figureName = 'Traffic Light Negotiation Using V2X';
                obj.Figure = findobj('Type','Figure','Name',figureName);
    
                % create the figure only if it is not already open.
                if isempty(obj.Figure)
                    obj.Figure = figure('Name',figureName);
                    obj.Figure.NumberTitle = "off";
                    obj.Figure.MenuBar = "none";
                    obj.Figure.ToolBar = "none";
                    obj.Figure.Visible = "on";
                    obj.Figure.Position = [300, 50, 1200, 800];
                end
                
                %Clear figure.
                clf(obj.Figure);
    
                % Get the scenario object from the base workspace.
                obj.Scenario = evalin('base', 'scenario');
                
                % If traffic light display is enabled 
                if obj.TrafficLightDisp
                    % UI panel for chase view
                    p1 = uipanel(obj.Figure,'Units','normalized','Position',[0.5 0.0 0.5 0.25],'Title','Chase View');
                    % UI panel for birds eye view
                    p2 = uipanel(obj.Figure,'Units','normalized','Position',[0.5 0.25 0.5 0.75],'Title','V2V Detections in Birds Eye View');
                    % UI panel for traffic light simulation
                    p3 = uipanel(obj.Figure,'Units','normalized','Position',[0.0 0.0 0.5 1.0],'Title','Traffic Light Simulation');
                    obj.AxesHandleChasePlot = axes(p1);
                    obj.AxesHandleBEP = axes(p2);
                    obj.AxesHandle    = axes(p3);         
        
                    
                    % Plot the scenario.
                    plot(obj.Scenario,'Parent',obj.AxesHandle,'RoadCenters',...
                        'off','Centerline','off');
                    hold(obj.AxesHandle,'on');
                    
                    % Get intersection info from the base workspace.
                    intersectionInfo = evalin('base', 'intersectionInfo');
                    
                    % Update the traffic light positions from the intersection
                    % information.
                    trafficLightPositions = [intersectionInfo.tlSensor1Position(1,1), ...
                        intersectionInfo.tlSensor1Position(1,2); ...
                        intersectionInfo.tlSensor2Position(1,1), ...
                        intersectionInfo.tlSensor2Position(1,2); ...
                        intersectionInfo.tlSensor3Position(1,1), ...
                        intersectionInfo.tlSensor3Position(1,2); ...
                        intersectionInfo.tlSensor4Position(1,1), ...
                        intersectionInfo.tlSensor4Position(1,2), ...
                        ];
                    
                    % preallocate an array to store graphics handles.
                    obj.TrafficLightGraphicObjects = gobjects(4,1);
                    
                    % Plot traffic light sensor positions.
                    for i=1:size(trafficLightPositions,1)
                        obj.TrafficLightGraphicObjects(i) = plot(trafficLightPositions(i,1),trafficLightPositions(i,2), ...
                            'Marker','o','MarkerFaceColor','r','MarkerSize',4,...
                            'Parent',obj.AxesHandle);
                    end
                     
                    % Add legend information.
                    hAxes=get(obj.AxesHandle,'Children');
                    
                    % Assign 'none' for the LineStyle in legend for traffic light
                    % representation.
                    hAxes(2).LineStyle = 'none';
                    
                    % Add legend
                    legend([hAxes(2)], {'State of Traffic light on the Ego lane'},'AutoUpdate','off',"Location","northoutside",'Orientation','horizontal');
                else
                    % UI panel for chase view
                    p1 = uipanel(obj.Figure,'Units','normalized','Position',[0.0 0.0 0.5 1.0],'Title','Chase View');
                    % UI panel for birds eye view
                    p2 = uipanel(obj.Figure,'Units','normalized','Position',[0.5 0.0 0.5 1.0],'Title','V2V Detections in Birds Eye View');
                    obj.AxesHandleChasePlot = axes(p1);
                    obj.AxesHandleBEP = axes(p2);        
                end
    
                chasePlot(obj.Scenario.Actors(1), ...
                    'Parent',obj.AxesHandleChasePlot,'Meshes','on');
                XYlimit = [-100 100];
                if obj.range>XYlimit(2)
                    XYlimit = [-1*obj.range-20 obj.range+20];
                end
                % Create birds eye plot
                obj.BEP = birdsEyePlot('XLim',XYlimit,'YLim',XYlimit, ...
                    'Parent',obj.AxesHandleBEP);
                obj.BEP.Parent.Legend.Location = 'northoutside';
                obj.BEP.Parent.Legend.Orientation = 'horizontal';
    
                % Create detection plot
                obj.DetectionPlot = detectionPlotter(obj.BEP,'DisplayName','Tracks', ...
                    'Marker','s','MarkerFaceColor','none');
                % Create MIO Plot
                obj.MIOPlt = detectionPlotter(obj.BEP,'DisplayName','MIO', ...
                    'Marker','s','MarkerFaceColor','red');
                % Add an outline plotter for ground truth
                obj.OutlinePlot = outlinePlotter(obj.BEP, 'Tag', 'Ground truth');
                % Add road borders to plot
                obj.LaneMarkingPlot = laneMarkingPlotter(obj.BEP,'DisplayName', ...
                    '  Lanes');
                if obj.range>0
                    % Add coverage area plot if range is greater than zero
                    obj.CoverageAreaPlot = coverageAreaPlotter(obj.BEP,'DisplayName', ...
                        '  V2V Range','FaceColor','r');
                    plotCoverageArea(obj.CoverageAreaPlot,[0 0],obj.range,0,360);
                end
            end
        end
        
        %------------------------------------------------------------------
        function stepImpl(obj,trafficLightSensorStates,...
                targetPoses, egoInfo,tracks,mioIndex,posSelector)
            if obj.EnableVisualization
                % stepImpl Implements the main logic that updates the plot with
                % new actor positions and traffic light states for all
                % traffic lights.
                
                % Get the number of actors along with ego vehicle.
                numActors = targetPoses.NumActors + 1;
                
                % Update ego position and yaw.
                obj.Scenario.Actors(1).Position = egoInfo.Position;
                obj.Scenario.Actors(1).Yaw = egoInfo.Yaw;
                
                % Update other vehicle positions in the scenario.
                for n = 2:numActors
                    obj.Scenario.Actors(n).Position = ...
                        targetPoses.Actors(n-1).Position;
                    obj.Scenario.Actors(n).Yaw   = targetPoses.Actors(n-1).Yaw;
                end
    
                % Update the ground truth
                [position, yaw, length, width, originOffset, color] = targetOutlines(obj.Scenario.Actors(1));
                plotOutline(obj.OutlinePlot, position, yaw, length, width,'OriginOffset', originOffset, 'Color', color);
    
                % Update the lanes
                [lmv, lmf] = laneMarkingVertices(obj.Scenario.Actors(1));
                if isvalid(obj.AxesHandleBEP)
                    plotLaneMarking(obj.LaneMarkingPlot,lmv,lmf);
                end
    
                % Get tracks location for MIO and Other target vehicles in
                % vehicle co-ordinate
                [locations,mioPos] = obj.computeDetectionLocation(tracks,mioIndex,egoInfo,posSelector);
                % Update the track location
                plotDetection(obj.DetectionPlot,locations);
                plotDetection(obj.MIOPlt,mioPos);
                
                if ~isempty(obj.AxesHandle)   
                    if (isvalid(obj.AxesHandle))
                        % Update marker face colors based on the traffic light
                        % sensor states. Red state of TL Sensor 1 is represented by
                        % value 0. Yellow state of TL Sensor 1 is represented by
                        % value 1. Green state of TL Sensor 1 is represented by
                        % value 2. Black represents that there are no valid
                        % detections from TL Sensor 1. It is assumed to have
                        % one-to-one correspondence between
                        % TrafficLightGraphicObjects and trafficLightSensorStates.
                        for i = 1:size(obj.TrafficLightGraphicObjects,1)
                            switch(trafficLightSensorStates(i))
                                case 0
                                    % Set the marker color to red.
                                    obj.TrafficLightGraphicObjects(i).MarkerFaceColor = 'r';
                                case 1
                                    % Set the marker color to yellow.
                                    obj.TrafficLightGraphicObjects(i).MarkerFaceColor = 'y';
                                case 2
                                    % Set the marker color to green.
                                    obj.TrafficLightGraphicObjects(i).MarkerFaceColor = 'g';
                                otherwise
                                    % Set the marker color to black.
                                    obj.TrafficLightGraphicObjects(i).MarkerFaceColor = 'k';
                            end
                        end
                        % limits the number of updates to 20 frames per second.
                        drawnow limitrate
                    end
                end
    
                % update existing scenario plot.
                updatePlots(obj.Scenario);
            end
        end
        %----------------------------------------------------------------------------------
        function [locations,mioPos] = computeDetectionLocation(obj,tracks,mioIndex,egoInfo,posSelector)
        % get track position and velocity
            if tracks.NumTracks>0
                % Get track position from confirmed tracks 
                confirmedTracks = tracks.Tracks(1:tracks.NumTracks);
                trackPos = getTrackPositions(confirmedTracks,posSelector);
                % Convert track position to ego coordinates
                egoYaw = egoInfo.Yaw;
                rotmat = [cosd(egoYaw), -sind(egoYaw); sind(egoYaw), cosd(egoYaw)];
                trackPos = (trackPos(1:end,:)-egoInfo.Position(1:2))*rotmat;
                % Get track ID
                trackID  = [confirmedTracks(1:end).TrackID];
                % set the values for the output
                idx = find(trackID == mioIndex);
                mioPos = trackPos(idx,:);
                trackPos(idx,:) = [];
                locations = trackPos;
            else
                % If no tracks set the values to infinite
                locations = [inf,inf];
                mioPos    = [inf,inf];
            end
        end

    end
    
    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog.
            simMode = "Interpreted execution";
        end
        
        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block
            % dialog.
            flag = false;
        end
    end
    
end
