classdef TaskVehiclePos < Task   
    properties

    end


    methods
        function updateReference(obj, robot)
            [v_ang, v_lin] = CartError(robot.wTgv , robot.wTv);
            obj.xdotbar = 0.2 * [v_ang; v_lin];
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7), -eye(3), zeros(3,3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
            % obj.A = [zeros(3,7), eye(3), zeros(3,3)];
            % obj.A = eye(6);
        end
    end
end