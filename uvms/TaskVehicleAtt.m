classdef TaskVehicleAtt < Task   
    properties
        n
        theta
        theta_star = 0
        gain = 0.2
    end


    methods
        function updateReference(obj, robot)
            [w_ang_err, ~] = CartError(robot.wTv, robot.wTgv);
            obj.theta = norm(w_ang_err);
            obj.n = w_ang_err / obj.theta;
            % lambda = 0.2 gain
            obj.xdotbar = obj.gain * (obj.theta_star - obj.theta);
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = obj.n' * (robot.wTv(1:3,1:3) * [zeros(3,10), eye(3)]);
        end
        
        function updateActivation(obj, robot)
            obj.A = 1;
        end
    end
end