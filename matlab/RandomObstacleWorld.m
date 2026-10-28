classdef RandomObstacleWorld < handle
    properties (Access=private)
        Track
        Config
        Stream
        Seed
        NextSpawn
        Time = 0
        Obstacles
        ExpiryTimes = zeros(0,1)
    end
    methods
        function obj = RandomObstacleWorld(track,config,seed)
            if nargin < 3 || isempty(seed)
                entropy = java.security.SecureRandom;
                seed = double(entropy.nextInt(intmax('int32')));
            end
            validateattributes(seed,{'numeric'}, ...
                {'scalar','integer','finite','>=',0,'<=',2^32-1});
            obj.Track = track;
            obj.Config = random_world_parameters(config);
            obj.Seed = seed;
            obj.Stream = RandStream('mt19937ar','Seed',seed);
            obj.NextSpawn = obj.wait_time();
            obj.Obstacles = table(zeros(0,1),zeros(0,1),zeros(0,1), ...
                zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1), ...
                'VariableNames',{'id','X','Y','R','spawn_time', ...
                'requested_spawn_time','detection_time','removal_time','removal_detection_time'});
        end

        function [spawned,removed] = advance(obj,time)
            validateattributes(time,{'numeric'},{'scalar','finite','>=',obj.Time});
            obj.Time = time;
            expired = isnan(obj.Obstacles.removal_time) & obj.ExpiryTimes<=time;
            obj.Obstacles.removal_time(expired) = time;
            removed = obj.Obstacles(expired,:);
            first = height(obj.Obstacles)+1;
            while obj.NextSpawn<=time && height(obj.Obstacles)<obj.Config.max_obstacles
                s = rand(obj.Stream)*obj.Track.length;
                grid = [obj.Track.s;obj.Track.length];
                x = interp1(grid,[obj.Track.X(:);obj.Track.X(1)],s);
                y = interp1(grid,[obj.Track.Y(:);obj.Track.Y(1)],s);
                heading = interp1(grid,unwrap([obj.Track.psi(:);obj.Track.psi(1)]),s);
                lateral = (2*rand(obj.Stream)-1)*obj.Config.lateral_spread;
                radius = obj.Config.radius_min+rand(obj.Stream)* ...
                    (obj.Config.radius_max-obj.Config.radius_min);
                row = {height(obj.Obstacles)+1,x+lateral*sin(heading), ...
                    y-lateral*cos(heading),radius,time,obj.NextSpawn,NaN,NaN,NaN};
                obj.Obstacles(end+1,:) = row;
                lifetime = obj.Config.lifetime_min;
                if isfinite(lifetime)
                    lifetime = lifetime+rand(obj.Stream)* ...
                        (obj.Config.lifetime_max-obj.Config.lifetime_min);
                end
                obj.ExpiryTimes(end+1,1) = time+lifetime;
                obj.NextSpawn = obj.NextSpawn+obj.wait_time();
            end
            spawned = obj.Obstacles(first:end,:);
        end

        function [known,discovered,cleared] = observe(obj,pose,time)
            validateattributes(pose,{'numeric'},{'vector','numel',4,'real','finite'});
            if time~=obj.Time
                error('mpc:WorldClock','Advance the world to the observation time first.');
            end
            dx = obj.Obstacles.X-pose(1);
            dy = obj.Obstacles.Y-pose(2);
            bearing = atan2(sin(atan2(dy,dx)-pose(3)),cos(atan2(dy,dx)-pose(3)));
            sensed = hypot(dx,dy)<=obj.Config.sensor_range & ...
                abs(bearing)<=obj.Config.sensor_fov/2;
            present = isnan(obj.Obstacles.removal_time);
            fresh = sensed & present & isnan(obj.Obstacles.detection_time);
            obj.Obstacles.detection_time(fresh) = time;
            absent = sensed & ~present & isfinite(obj.Obstacles.detection_time) & ...
                isnan(obj.Obstacles.removal_detection_time);
            obj.Obstacles.removal_detection_time(absent) = time;
            % Do not leak actual out-of-range removal times into the remembered map.
            observed = {'id','X','Y','R','spawn_time','detection_time','removal_detection_time'};
            discovered = obj.Obstacles(fresh,observed);
            cleared = obj.Obstacles(absent,observed);
            known = obj.Obstacles(isfinite(obj.Obstacles.detection_time) & ...
                isnan(obj.Obstacles.removal_detection_time),observed);
        end

        function truth = snapshot(obj)
            % Only already-created objects exist in this export; there is no future schedule.
            truth = obj.Obstacles;
        end

        function value = metadata(obj)
            value = struct('mode','independent_random_world','seed',obj.Seed, ...
                'parameters',obj.Config,'placement','uniform_track_arc_and_lateral_band', ...
                'arrivals','exponential_waits_sampled_without_car_or_controller_input', ...
                'sensor','ideal_range_and_fov_with_memory_no_occlusion', ...
                'removal_policy','private_uniform_lifetimes_absence_requires_sensor_confirmation', ...
                'spawn_policy','no_feasibility_filter_no_deferral');
        end
    end
    methods (Access=private)
        function wait = wait_time(obj)
            wait = -obj.Config.mean_interval*log(max(rand(obj.Stream),realmin));
        end
    end
end
