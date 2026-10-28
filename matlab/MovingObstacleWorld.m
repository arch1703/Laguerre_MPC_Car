classdef MovingObstacleWorld < handle
    properties (Access=private)
        Track
        Origins
        Speeds
        Time = 0
        Map
    end
    methods
        function obj = MovingObstacleWorld(track)
            obj.Track = track;
            obj.Origins = track.length*[0.20;0.50;0.80];
            obj.Speeds = [0.18;0.18;0.18];
            obj.Map = obj.snapshot(0);
            obj.Map.X(:) = NaN;
            obj.Map.Y(:) = NaN;
            obj.Map.last_seen = NaN(3,1);
        end

        function advance(obj,time)
            validateattributes(time,{'numeric'},{'scalar','finite','>=',obj.Time});
            obj.Time = time;
        end

        function truth = snapshot(obj,time)
            if nargin<2, time = obj.Time; end
            validateattributes(time,{'numeric'},{'scalar','finite','nonnegative'});
            s = mod(obj.Origins+obj.Speeds*time,obj.Track.length);
            grid = [obj.Track.s;obj.Track.length];
            X = interp1(grid,[obj.Track.X(:);obj.Track.X(1)],s);
            Y = interp1(grid,[obj.Track.Y(:);obj.Track.Y(1)],s);
            truth = table((1:3)',X,Y,0.12*ones(3,1),zeros(3,1), ...
                'VariableNames',{'id','X','Y','R','spawn_time'});
        end

        function [known,history] = observe(obj,pose,time)
            validateattributes(pose,{'numeric'},{'vector','numel',4,'real','finite'});
            if time~=obj.Time
                error('mpc:WorldClock','Advance the world before observing.');
            end
            truth = obj.snapshot();
            sensed = hypot(truth.X-pose(1),truth.Y-pose(2))<=2.5;
            obj.Map.X(sensed) = truth.X(sensed);
            obj.Map.Y(sensed) = truth.Y(sensed);
            obj.Map.last_seen(sensed) = time;
            detected = isfinite(obj.Map.last_seen);
            known = obj.Map(detected,:);
            history = truth;
            history.t = repmat(time,3,1);
            history.sensed = sensed;
            history.known = detected;
            history.known_X = obj.Map.X;
            history.known_Y = obj.Map.Y;
            history.last_seen = obj.Map.last_seen;
        end

        function value = metadata(~)
            value = struct('mode','live_moving_world', ...
                'parameters',struct('sensor_range',2.5,'sensor_fov',2*pi), ...
                'motion','three_centerline_objects_0.18_mps_counterclockwise', ...
                'obstacle_prediction','none_latest_measured_position_held_over_horizon', ...
                'memory','last_measured_position_held_when_out_of_range', ...
                'spawn_policy','all_present_at_start_independent_of_vehicle');
        end

        function speed = speed_bound(obj)
            speed = max(abs(obj.Speeds));
        end
    end
end
