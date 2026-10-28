classdef BoundarySensor < handle
    properties (Access=private)
        Config
        Stream
        Frames = {}
        LastProcessed = 0
        CachedLines
        LineTime = 0
        Map
    end
    methods
        function obj = BoundarySensor(config)
            obj.Config = boundary_experiment_parameters(config);
            obj.Stream = RandStream('mt19937ar','Seed',obj.Config.seed);
            obj.Map = table(zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1), ...
                zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1), ...
                'VariableNames',{'id','X','Y','R','last_seen','speed_bound', ...
                'position_bound','miss_count'});
        end

        function [corridor,measured,lines,history,state] = observe(obj,track,world,pose,time)
            c = obj.Config;
            % Fixed draw count per tick: exogenous perturbations do not depend on control.
            draws = 2*rand(obj.Stream,20,1)-1;
            [~,raw_lines] = sense_track_lines(track,pose,3.5);
            columns = {'left_X','left_Y','right_X','right_Y'};
            for j = 1:4
                raw_lines.(columns{j}) = raw_lines.(columns{j})+c.line_noise*draws(j);
            end
            [observed,history] = world.observe(pose,time,3.5);
            detected = (draws(4+observed.id)+1)/2>=c.miss_probability;
            observed = observed(detected,:);
            for j = 1:height(observed)
                id = observed.id(j);
                observed.X(j) = observed.X(j)+c.obstacle_noise*draws(8+id);
                observed.Y(j) = observed.Y(j)+c.obstacle_noise*draws(12+id);
            end
            if c.uncertain_classification
                observed.speed_bound(:) = world.speed_bound();
            end
            observed.position_bound = sqrt(2)*c.obstacle_noise*ones(height(observed),1);
            frame = struct('time',time,'pose',pose,'lines',raw_lines,'objects',observed, ...
                'lines_present',~(time>=c.line_loss_start && time<c.line_loss_end));
            obj.Frames{end+1} = frame;
            % The initial sample bootstraps the sensor before delayed updates arrive.
            index = max(1,numel(obj.Frames)-c.latency_steps);
            if index>obj.LastProcessed
                delivery = obj.Frames{index};
                if delivery.lines_present || isempty(obj.CachedLines)
                    obj.CachedLines = delivery.lines;
                    obj.LineTime = delivery.time;
                end
                obj.update_map(delivery);
                obj.LastProcessed = index;
            end
            lines = obj.CachedLines;
            lines.capture_time = repmat(obj.LineTime,height(lines),1);
            lines.t = repmat(time,height(lines),1);
            left = [lines.left_X,lines.left_Y]; right = [lines.right_X,lines.right_Y];
            center = (left+right)/2;
            inward = (right-left)./vecnorm(right-left,2,2);
            margin = sqrt(2)*c.line_noise+c.model_margin;
            left = left+margin*inward; right = right-margin*inward;
            corridor = path_geometry(center,false);
            corridor.left = left; corridor.right = right;
            corridor.half_width = vecnorm(left-right,2,2)/2;
            measured = obj.Map(:,1:7);
            delivery = obj.Frames{index};
            outside = ~ismember(measured.id,delivery.objects.id) & c.miss_probability==0;
            measured.outside_visible = outside;
            ages = time-measured.last_seen;
            [progress,~] = project_path(corridor,pose(1:2));
            [s,distance] = project_path(corridor,[measured.X,measured.Y]);
            relevant = ~outside & s>=progress-0.3 & distance<= ...
                max(corridor.half_width)+measured.R+measured.speed_bound.*ages;
            state = struct('line_age',time-obj.LineTime, ...
                'obstacle_age',max([0;ages(relevant)]), ...
                'line_stale',time-obj.LineTime>c.max_line_age, ...
                'obstacle_stale',any(ages(relevant)>c.max_obstacle_age), ...
                'delivery_time',delivery.time,'range_center',delivery.pose(1:2), ...
                'range_age',time-delivery.time,'range',3.5, ...
                'unseen_speed_bound',world.speed_bound(), ...
                'unseen_radius_bound',0.10*c.radius_scale, ...
                'complete_detection',c.miss_probability==0);
            for j = 1:height(history)
                map_index = find(measured.id==history.id(j),1);
                history.known(j) = ~isempty(map_index);
                history.known_X(j) = NaN; history.known_Y(j) = NaN;
                history.last_seen(j) = NaN;
                if ~isempty(map_index)
                    history.known_X(j) = measured.X(map_index);
                    history.known_Y(j) = measured.Y(map_index);
                    history.last_seen(j) = measured.last_seen(map_index);
                end
            end
        end
    end
    methods (Access=private)
        function update_map(obj,frame)
            seen = frame.objects;
            for j = 1:height(obj.Map)
                if ismember(obj.Map.id(j),seen.id), continue; end
                age = frame.time-obj.Map.last_seen(j);
                radius = obj.Map.position_bound(j)+obj.Map.speed_bound(j)*age;
                contained = hypot(obj.Map.X(j)-frame.pose(1), ...
                    obj.Map.Y(j)-frame.pose(2))+radius<3.5;
                if contained, obj.Map.miss_count(j) = obj.Map.miss_count(j)+1;
                else, obj.Map.miss_count(j) = 0;
                end
            end
            obj.Map(obj.Map.miss_count>=obj.Config.absence_confirmations,:) = [];
            for j = 1:height(seen)
                row = [seen(j,:),table(0,'VariableNames',{'miss_count'})];
                index = find(obj.Map.id==seen.id(j),1);
                if isempty(index), obj.Map = [obj.Map;row];
                else, obj.Map(index,:) = row;
                end
            end
            obj.Map = sortrows(obj.Map,'id');
        end
    end
end
