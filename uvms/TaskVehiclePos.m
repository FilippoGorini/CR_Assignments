classdef TaskVehiclePos < Task   
    properties

    end


    methods
        function updateReference(obj, robot)
            % In this task our control variable x is the cartesian distance
            % only (no orientation), so it is a 3x1 vector, therefore xdot
            % is 3x1 as well.
            [~, v_lin] = CartError(robot.wTgv , robot.wTv);
            % lambda = 0.2 gain
            obj.xdotbar = - 0.2 * v_lin;  % we should also consider the velocity of the goal if it was moving (feedforward term)
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7), -eye(3), zeros(3,3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end