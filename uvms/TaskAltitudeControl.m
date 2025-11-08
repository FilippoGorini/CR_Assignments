classdef TaskAltitudeControl < Task   
    properties
        h
        % h_star is the goal for the equivalent equality task
        h_star = 1.5;
        h_full_activation = 1
    end


    methods
        function updateReference(obj, robot)
            obj.h = robot.altitude;
            % Get reference rate by multiplying gain (0.2) by error between
            % h_star and h
            obj.xdotbar = 0.2 * (obj.h_star - obj.h);
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(1,7) 0 0 1 0 0 0];
        end
        
        function updateActivation(obj, robot)
            obj.A = DecreasingBellShapedFunction(obj.h_full_activation, obj.h_star, 0, 1, obj.h);
        end
    end
end