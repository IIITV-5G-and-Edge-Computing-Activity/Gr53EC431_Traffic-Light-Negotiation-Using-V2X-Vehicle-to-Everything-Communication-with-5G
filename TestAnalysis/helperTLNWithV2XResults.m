function helperTLNWithV2XResults(logsout)
% helperTLNWithV2XResults A helper function for plotting
% the results of the Traffic Light Negotiation with V2X demo.
%
% NOTE: This is a helper function for example purposes and may be removed 
% or modified in the future.
%
% The function assumes that the demo outputs the Simulink log, logsout.

% Copyright 2021-2023 The MathWorks, Inc.
%%
numsubplots = 6;
%% Get the data from simulation

% MIO Index 
mioIdx = logsout.getElement('mioIdx');

% Cross Over Alert
cross_over_alert = logsout.getElement('cross_over_alert');

% number of vehicle to vehicle detections
numv2vdet = logsout.getElement('<NumOfBSM>');

% longitudinal velocity of ego car
ego_long_velocity = logsout.getElement('ego_velocity');                                 

% tlSensor1 traffic light state
tlSensor1State = logsout.getElement('tlSensor1State');

% relative distance to mio (tracker)
relative_distance = logsout.getElement('mio_relative_distance');                        

% ego_acceleration
ego_acceleration = logsout.getElement('ego_acceleration');

% ego_yaw
ego_yaw = logsout.getElement('ego_yaw');

% simulation time
tmax = ego_long_velocity.Values.time(end);                                              

%% Plot the spacing control results
hFigure = figure('Name','Spacing Control Performance','position',[100 100 720 600]);

%% Plot the states of TL Sensor 1 traffic light.
%
% Red state of TL Sensor 1 is represented by value 0.
% Yellow state of TL Sensor 1 is represented by value 1.
% Green state of TL Sensor 1 is represented by value 2.
% Black represents that there are no valid states from TL Sensor 1.  
subplot(numsubplots,1,1)
for i=1:size(tlSensor1State.Values.time,1)    
    switch tlSensor1State.Values.Data(i)
        case 0 
            % Set the marker color to red.
            markerColor = 'r';    
        case 1
            % Set the marker color to yellow.
            markerColor = 'y';
        case 2
            % Set the marker color to green.
            markerColor = 'g';
        otherwise
            % Set the marker color to black.
            markerColor = 'k';
    end
    scatter(tlSensor1State.Values.time(i), tlSensor1State.Values.Data(i),markerColor);
    hold on;
end
grid on
xlim([0,tmax])
title('Traffic light state - TL Sensor 1')
xlabel('time (sec)')

subplot(numsubplots,1,2)
xData = numv2vdet.Values.time;
yData = reshape(numv2vdet.Values.Data,[numel(xData) 1]);
stem(xData,yData,'b','LineStyle','-','MarkerSize',3);
xlim([0,tmax])
title('Number of V2V Detections')
xlabel('time (sec)')
ylabel('$num$ $of$ $detections$','Interpreter','latex')

subplot(numsubplots,1,3)
mioData = mioIdx.Values.Data;
crossOverData = cross_over_alert.Values.Data;
mioData = (mioData>0);
yData = mioData+crossOverData;
yData(yData == 1) = -1;
yData(yData == 2) =  1;
xData = mioIdx.Values.Time;
stem(xData,yData,'b','LineStyle','-','MarkerSize',3);
xlim([0,tmax])
title('MIO')
xlabel('time (sec)')
yticks([-1 0 1])
yticklabels({'Lead MIO','No MIO','Cross Over MIO'})


%% relative longitudinal distance
subplot(numsubplots,1,4)
plot(relative_distance.Values.time,relative_distance.Values.Data,'b')
grid on
xlim([0,tmax])
title('Relative longitudinal distance (between ego and MIO/Intersection Point)')
xlabel('time (sec)')
ylabel('$meters$','Interpreter','latex')

%% ego acceleration 
subplot(numsubplots,1,5)
plot(ego_acceleration.Values.time,ego_acceleration.Values.Data,'b')
grid on
xlim([0,tmax])
ylim(ylim + [-2 2])
title('Ego acceleration')
xlabel('time (sec)')
ylabel('$m/s^2$','Interpreter','latex')

%% ego yaw
subplot(numsubplots,1,6)
plot(ego_yaw.Values.time,(ego_yaw.Values.Data),'b')
grid on
xlim([0,tmax])
ylim(ylim + [-2 2])
title('Ego yaw angle')
xlabel('time (sec)')
ylabel('$degree$','Interpreter','latex')

end
