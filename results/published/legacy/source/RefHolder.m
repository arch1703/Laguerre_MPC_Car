classdef RefHolder < handle
    % Thin handle class so anonymous functions capture the ref by reference.
    % Updating holder.ref between nlmpcmove calls is seen by the closure
    % without changing the function handle (which would reset warm-start).
    properties
        ref
        previous
        obstacles
        track
    end
end
