classdef TaskHorizontalAtt < Task   
    properties
        n
        theta
        % theta_star is the goal for the equivalent equality task
        theta_star = 0.1
        theta_full_activation = 0.2
        gain = 0.2
    end


    methods
        function updateReference(obj, robot)
            % In this task our control variable x is the misalignment
            % vector norm (theta) between k_v and k_w so it is a scalar
       
            % Extract rotation matrix of vehicle wrt world
            wRv = robot.wTv(1:3, 1:3);
            % Get axis angle representation equivalent (n*theta = rho)
            [obj.n, obj.theta] = RotToAngleAxis(wRv);
            % Get reference rate by multiplying gain (0.2) by error between
            % theta_star and theta
            obj.xdotbar = obj.gain * (obj.theta_star - obj.theta);
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = obj.n' * [zeros(3,10), eye(3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = IncreasingBellShapedFunction(obj.theta_star, obj.theta_full_activation, 0, 1, obj.theta);
        end
    end
end