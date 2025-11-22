classdef TaskVehicleOrient < Task   
    properties
        n
        rho         % rho is misalignment between vehicle and vehicle goal, projected on <w>
        rho_star = zeros(3,1)
        gain = 0.4
    end


    methods
        function updateReference(obj, robot)
            % Compute misalignment vector projected on <w>
            [obj.rho, ~] = CartError(robot.wTv, robot.wTgv);
            % Compute reference for xdot bar
            obj.xdotbar = obj.gain * (obj.rho_star - obj.rho);
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end

        function updateJacobian(obj, robot)
            obj.J = robot.wTv(1:3,1:3) * [zeros(3,10), eye(3)];
        end
        
        function updateActivation(obj, robot, task_status, time_elapsed, transition_time)
            % Update the task's inherent activation
            obj.A = eye(3);
            % Update the activation depending on the task status
            if task_status == "FADING_IN"
                obj.A = IncreasingBellShapedFunction(0, transition_time, 0, 1, time_elapsed) * obj.A;
            elseif task_status == "FADING_OUT"
                obj.A = DecreasingBellShapedFunction(0, transition_time, 0, 1, time_elapsed) * obj.A;
            end
        end
    end
end