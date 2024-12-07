function [relativeDistance,relativeVelocity,mioTrackIndex,crossOverVehicleAlert] = helperFindMIO(tracks, lane, egoInfo, maxDistance,distanceToIntersection,posSelector,velSelector)
    % helperFindMIO Find's the most Important object and compute the
    % relative distance and relative velocity. The MIO can be lead car or
    % it can be a crossover vehicle

    % NOTE: This is a helper function for example purposes and may be removed or
    % modified in the future.

    % Copyright 2021 The MathWorks, Inc.

    % Initialize outputs and parameters
    relativeDistance = Inf;
    relativeVelocity = Inf;
    mioTrackIndex    = -1;
    crossOverVehicleAlert = false;

    % Initialise track related Information
    coder.varsize('confirmedTracks');
    coder.varsize('posTrack');
    coder.varsize('velTrack');
    confirmedTracks = tracks.Tracks(1);
    posTrack = [0 0];
    velTrack = [0 0];

    % get track position and velocity
    if tracks.NumTracks>0
        confirmedTracks = tracks.Tracks(1:tracks.NumTracks);
        posTrack = getTrackPositions(confirmedTracks,posSelector);
        velTrack = getTrackVelocities(confirmedTracks,velSelector);
    end

    % Initialize MIO Index
    mioIndex    = 0;
    crossOverMIOIndex = 0;

    % Initialize Position for lead and crossover vehicle
    mioPos = inf;
    mioCrossPos = inf;

    % Initialize threshold for relative distance
    minDist        = maxDistance;
    minCrossDist   = maxDistance;
    % Initialize minimum distance to intersection
    minDistToInter = 60; % meters
    % Initialize minimum time gap
    minTgap = 5;

    % Initialize Velocity for lead and crossover vehicle
    mioVel = inf;
    mioCrossVel = inf;

    % Time for path prediction
    tpp = 20;

    % If lane detection is not available, the ego car is assumed to be located
    % in the middle of its lane. The lane width is assumed to be 3.6m, typical
    % highway lane width.
    
    % Lane Info
    laneWidth = 3.6;
    halfLaneWidth = laneWidth/2;
    
    % Compute the left and right lane boundaries
    if lane.Left.Strength > 0.01
        lb = [lane.Left.CurvatureDerivative/6,...
            lane.Left.Curvature/2,...
            lane.Left.HeadingAngle,...
            lane.Left.LateralOffset];
    else
        lb = single([0,0,0,halfLaneWidth]);
    end
    
    if lane.Right.Strength > 0.01
        rb = [lane.Right.CurvatureDerivative/6,...
            lane.Right.Curvature/2,...
            lane.Right.HeadingAngle,...
            lane.Right.LateralOffset];
    else
        rb = single([0,0,0,-halfLaneWidth]);
    end

    
    for i = 1:tracks.NumTracks    
        targetPos = posTrack(i,:);
        targetVel = velTrack(i,:);

        egoPos = [egoInfo.Position(1:2)];
        egoYaw = egoInfo.Yaw;
        egoVelocity = egoInfo.Velocity;
        rotmat = [cosd(egoYaw), -sind(egoYaw); sind(egoYaw), cosd(egoYaw)];
        
        locations = (targetPos - egoPos)*rotmat;
        relativeDistance = locations(1); % Longitudinal position
        latPos = locations(2);           % Lateral position

        % Check Of MIO infront of ego
        if relativeDistance < minDist && relativeDistance > 0 % No point checking otherwise
            latLeftLane  = polyval(lb,relativeDistance); % lateral position of left lane
            latRightLane = polyval(rb,relativeDistance); % lateral position of right lane            
            % Find a new MIO track
            if (latRightLane <= latPos) && (latPos <= latLeftLane)
                minDist  = relativeDistance;
                mioIndex = double(confirmedTracks(i).TrackID);
                mioPos   = relativeDistance;
                mioVel   = double(norm(targetVel))-double(norm([egoVelocity(1),egoVelocity(2)]));
            end
        end
        % Check for crossover MIO vehicle when distance to intersection is 
        % less than threshold 
        if distanceToIntersection>0 && distanceToIntersection<minDistToInter
            % Compute target heading 
            targetHeading = rad2deg(cart2pol(targetVel(2),targetVel(1)));
            targetHeading = rem(targetHeading+360,360);
            % Compute ego heading
            egoHeading    = rem((450-rem(360+egoYaw,360)),360);
            % Check if the vehicle approaches from left or right, excluding
            % the forward and rear vehicles
            leftFlag  = (isAngleBetween(targetHeading,rem(egoHeading+40,360),rem(egoHeading+140,360))) && (latPos>=0);
            rightFlag = (isAngleBetween(targetHeading,rem(egoHeading+220,360),rem(egoHeading+320,360))) && (latPos<0);

            if leftFlag || rightFlag
                % Estimate target path
                targetSpeed   = norm(targetVel);
                targetPath    = vehiclePath(tpp,targetHeading,targetSpeed,[targetPos(1),targetPos(2)]);
                
                % Estimate ego path
                egoSpeed      = double(norm([egoVelocity(1),egoVelocity(2)]));
                if egoSpeed < 1.5
                    egoSpeed = 9;
                end
                egoPath       = vehiclePath(tpp,egoHeading,egoSpeed,[egoPos(1),egoPos(2)]);

                % Check for intersection between ego and target path
                intersection = intersectionPoint(targetPath,egoPath);

                % If intersection exist get the distance and velocity
                if ~isempty(intersection)
                    targetDist = norm(targetPath(1,:) - intersection);
                    egoDist    = norm(egoPath(1,:) - intersection);
                    targetTime = targetDist/targetSpeed;
                    egoTime    = egoDist/egoSpeed;
                    % If ego and target vehicle reach the intersection
                    % point within the minimum time gap and the ego
                    % distance to intersection is less than minimum
                    % crossover distance, consider it to be an MIO and
                    % compute the relative distance and velocity.

                    % For Cross Over Vehicle Condition,
                    % The relative distance is considered to be the
                    % distance between ego to intersection point and the
                    % relative velocity is considered to be the ego's
                    % velocity towards the intersection point.
                    if (abs(egoTime-targetTime)<=minTgap) && egoDist<minCrossDist
                        minCrossDist = egoDist;
                        mioCrossPos  = min(egoDist,distanceToIntersection);
                        mioCrossVel  = -1*double(norm([egoVelocity(1),egoVelocity(2)]));
                        crossOverMIOIndex = double(confirmedTracks(i).TrackID);
                    end
                end
            end
            
        end
    end
    if mioIndex > 0 || crossOverMIOIndex>0
        % Output:If MIO is lead vehicle
        if mioPos < mioCrossPos
            % Longitudinal position of the lead car
            relativeDistance = mioPos;
            % Longitudinal velocity of the lead car
            relativeVelocity = mioVel;
            mioTrackIndex = mioIndex;
        % Output:If MIO is crossover vehicle
        else
            % Distance to intersection point
            relativeDistance = mioCrossPos;
            % Ego velocity towards the intersection point
            relativeVelocity = mioCrossVel;
            mioTrackIndex    = crossOverMIOIndex;
            crossOverVehicleAlert = true;
        end
    else
        % Output: If no MIO
        relativeDistance = inf; 
        relativeVelocity = inf; 
        mioTrackIndex    = -1;
    end
end

% Estimate the vehicle's path based on heading, speed and current position
function path = vehiclePath(tpp,heading,speed,initialPos)
    distTravelled  = speed*tpp;
    xOffset = distTravelled*sind(double(heading));
    yOffset = distTravelled*cosd(double(heading));
    path    = double([initialPos(1) initialPos(2);initialPos(1)+xOffset initialPos(2)+yOffset]);
end

% Compute the Intersection Point between two paths
function intersection = intersectionPoint(targetPath,egoPath)
    intersection = zeros(0,2);
    x1 = double([targetPath(1,1),targetPath(2,1)]);
    y1 = double([targetPath(1,2),targetPath(2,2)]);
    x2 = double([egoPath(1,1),egoPath(2,1)]);
    y2 = double([egoPath(1,2),egoPath(2,2)]);
    
    A = [x1(2)-x1(1)       0       -1   0;      
              0       x2(2)-x2(1)  -1   0;   
         y1(2)-y1(1)       0        0  -1;      
              0       y2(2)-y2(1)   0  -1];      
          
    B = [-x1(1);
         -x2(1);
         -y1(1);
         -y2(1)];
    % Compute solution only if matrix is not singular
    if rank(A)==size(A,2)
        T = (eye(4)/A)*B;
        if ((0<=T(1)) && (T(1)<1)) && ((0<=T(2)) && (T(2)<1))
            intersection = [T(3),T(4)];
        end
    end
end

% Check if an anlge lies between lower bound and upper bound
function flag = isAngleBetween(angle,lb,ub)
    flag=(lb<=angle & angle<=ub)|(((0<=angle & angle<=ub)|lb <= angle) & lb>ub);
end