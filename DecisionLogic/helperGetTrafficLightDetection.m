function trafficLightState = helperGetTrafficLightDetection(spatOut, approachingIntersectionName, approachingSignalName)
    trafficLightState = -1;
    intersectionNames = {spatOut.Intersections(:).Name}; % Extract names as a cell array
    % Check if message from approaching intersection exists
    idx = find(strcmp(intersectionNames, approachingIntersectionName)); % Use strcmp for comparison
    if ~isempty(idx)
        % Check if 'Status' exists and can be processed
        try
            % Handle Status field conversion
            rawStatus = spatOut.Intersections(idx).Status;
            if ischar(rawStatus)
                intersectionSignalStatus = bin2dec(rawStatus); % Convert binary string to decimal
            elseif isnumeric(rawStatus)
                intersectionSignalStatus = rawStatus; % Use directly if numeric
            else
                error("Unsupported 'Status' field format");
            end

            % Check if the signal in the intersection is in a working state
            if any(intersectionSignalStatus == [0, 1, 5, 6])
                % Find the signal state for the approaching signal
                signalNames = {spatOut.Intersections(idx).States(:).MovementName}; % Convert to cell array
                sigIdx = find(strcmp(signalNames, approachingSignalName)); % Compare strings
                if ~isempty(sigIdx)
                    % Extract EventState
                    eventState = spatOut.Intersections(idx).States(sigIdx).StateTimeSpeed.EventState;
                    switch eventState
                        case MovementPhaseState.PermissiveMovementAllowed
                            trafficLightState = 2;
                        case MovementPhaseState.PermissiveClearance
                            trafficLightState = 1;
                        case MovementPhaseState.StopAndRemain
                            trafficLightState = 0;
                        otherwise
                            trafficLightState = -1;
                    end
                end
            end
        catch ME
            % Display error for debugging purposes
            disp("Error processing intersection signal status:");
            disp(ME.message);
        end
    end
end
