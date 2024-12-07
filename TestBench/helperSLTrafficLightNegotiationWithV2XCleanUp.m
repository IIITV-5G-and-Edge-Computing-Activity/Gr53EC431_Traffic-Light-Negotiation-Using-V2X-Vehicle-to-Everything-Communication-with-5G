% Clean up script for the Traffic Light Negotiation With V2X Example
%
% This script cleans up the example model. It is triggered by the
% CloseFcn callback.
%
% This is a helper script for example purposes and may be removed or
% modified in the future.

% Copyright 2021-2022 The MathWorks, Inc.

clearBuses({ ...
    'BusActors',...
    'BusVehicleToWorldActors',...
    'BusLaneBoundaries',...
    'BusLaneBoundariesLaneBoundaries',...
    'BusVehiclePose',...
    'BusAccelerationSet4Way',...
    'BusBrakeSystemStatus',...
    'BusBSMCoreData',...
    'BusId',...
    'BusIntersectionState',...
    'BusPositionalAccuracy',...
    'BusStates',...
    'BusStateTimeSpeed',...
    'BusBSM',...
    'BusActorsInfo',...
    'BusActorsPose',...
    'BusVehicleSize',...
    'BusDetections',...
    'BusMeasurementParameters',...
    'BusTargetDetections',...
    'BusMultiObjectTrackerTracks',...
    'BusMultiObjectTracker',...
    'BusSPAT'...
});

clear Cf
clear Cr
clear FB_decel
clear Iz
clear LaneSensor
clear LaneSensorBoundaries
clear M
clear N
clear PB1_decel
clear PB2_decel
clear PredictionHorizon
clear ControlHorizon
clear ReferencePathInfo
clear Ts
clear assignThresh
clear default_spacing
clear driver_decel
clear egoVehDyn
clear egoVehicle
clear headwayOffset
clear intersectionInfo
clear lf
clear lr
clear m
clear maxMIOLeadDistance
clear max_ac
clear max_dc
clear max_steer
clear min_ac
clear min_steer
clear numSensors
clear numTracks
clear posSelector
clear referencePathInfo
clear refpathSize
clear scenario
clear tau
clear tau2
clear timeMargin
clear timeToReact
clear time_gap
clear trafficLightStateTriggerThreshold
clear v0_ego
clear v_set
clear velSelector
clear waypointsSize
clear Default_decel
clear TimeFactor
clear stopVelThreshold
clear egoActorID
clear actorsClassID
clear V2XRange
clear regionInfo
clear sceneOrigin
clear channelAttributes
clear approachingIntersectionName
clear approachingSignalName

figs = findobj('Name','Traffic Light Negotiation Using V2X','Type','figure');
if ~isempty(figs)
    close(figs);
end
clear figs

function clearBuses(buses)
matlabshared.tracking.internal.DynamicBusUtilities.removeDefinition(buses);
end