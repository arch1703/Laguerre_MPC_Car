classdef CrossingObstacleWorld < handle
    properties (Access=private)
        Centers
        Normals
        Time = 0
        Config
    end
    methods
        function obj = CrossingObstacleWorld(track,config)
            if nargin<2, config = struct(); end
            obj.Config = boundary_experiment_parameters(config);
            stations = track.length*[0.18;0.48;0.70;0.86];
            grid = [track.s;track.length];
            obj.Centers = interp1(grid,[track.X,track.Y;track.X(1),track.Y(1)],stations);
            headings = interp1(grid,[track.psi;track.psi(1)+2*pi],stations);
            obj.Normals = [-sin(headings),cos(headings)];
        end

        function advance(obj,time)
            validateattributes(time,{'numeric'},{'scalar','finite','>=',obj.Time});
            obj.Time = time;
        end

        function truth = snapshot(obj,time)
            if nargin<2, time = obj.Time; end
            validateattributes(time,{'numeric'},{'scalar','finite','nonnegative'});
            phase = obj.Config.crossing_direction*obj.Config.crossing_scale*2*pi*time/28+ ...
                obj.Config.crossing_phase;
            lateral = [1.3*sin(phase-pi/2);-0.10;1.3*sin(phase+pi/2);0.10];
            points = obj.Centers+lateral.*obj.Normals;
            truth = table((1:4)',points(:,1),points(:,2),0.10*obj.Config.radius_scale*ones(4,1), ...
                [0;0;0;10],[Inf;Inf;Inf;26], ...
                'VariableNames',{'id','X','Y','R','spawn_time','removal_time'});
            truth = truth(truth.spawn_time<=time & truth.removal_time>time,:);
        end

        function [observed,history] = observe(obj,pose,time,range)
            if time~=obj.Time, error('mpc:WorldClock','Advance before sensing.'); end
            truth = obj.snapshot();
            sensed = hypot(truth.X-pose(1),truth.Y-pose(2))<=range;
            observed = truth(sensed,{'id','X','Y','R'});
            observed.last_seen = repmat(time,height(observed),1);
            observed.speed_bound = obj.speed_bound()*double(ismember(observed.id,[1,3]));
            history = truth;
            history.t = repmat(time,height(truth),1);
            history.sensed = sensed; history.known = sensed;
            history.known_X = truth.X; history.known_Y = truth.Y;
            history.known_X(~sensed) = NaN; history.known_Y(~sensed) = NaN;
            history.last_seen = NaN(height(truth),1);
            history.last_seen(sensed) = time;
            history.speed_bound = obj.speed_bound()*double(ismember(truth.id,[1,3]));
        end

        function speed = speed_bound(obj)
            speed = 1.3*2*pi/28*obj.Config.crossing_scale;
        end

        function value = metadata(obj)
            value = struct('mode','live_moving_world', ...
                'parameters',struct('sensor_range',3.5,'sensor_fov',2*pi), ...
                'track_sensor','local_paired_boundary_lines', ...
                'motion','two_transverse_crossers_one_static_one_temporary_static', ...
                'obstacle_prediction','isotropic_speed_bound_no_directional_forecast', ...
                'obstacle_speed_bound',obj.speed_bound(), ...
                'memory','current_range_measurements_only', ...
                'spawn_policy','independent_world_no_vehicle_or_controller_input');
        end
    end
end
