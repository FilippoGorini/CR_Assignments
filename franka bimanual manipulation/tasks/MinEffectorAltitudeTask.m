classdef MinEffectorAltitudeTask < Task
    properties
        h_min = 0.15;       % Minimum allowed altitude from obstacle
        buffer = 0.05;      % Buffer zone size         
        h_star;             % Target altitude when task activates 
        h_current;          % Current altitude relative to obstacle     
        
        h_obstacle = 0.55;  % Height of the obstacle (table) from ground (World Z=0)
        h_obstacle_start = 0.55;     
        h_obstacle_target = 0.55;               

        transition_duration = 0.1;  % for the obstacle altitude change (0.55 -> 0)
        time_since_change = 0;    % Timer 

        gain = 0.1;
    end
    
    methods
        function obj = MinEffectorAltitudeTask(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
            obj.h_star = obj.h_min + obj.buffer;
        end
        
        % Method to change obstacle height at runtime
        function setObstacleHeight(obj, new_height)
            obj.h_obstacle_start = obj.h_obstacle;
            obj.h_obstacle_target = new_height;
            % obj.h_obstacle = new_height;
            obj.time_since_change = 0;
        end
        
        function updateReference(obj, robot_system)

            % Logic to manage a smooth transition from 0.55 to 0 obstacle
            if obj.time_since_change < obj.transition_duration
                if obj.h_obstacle_start > obj.h_obstacle_target
                    obj.h_obstacle = DecreasingBellShapedFunction(0, ...
                                                                  obj.transition_duration, ...
                                                                  obj.h_obstacle_start, ...
                                                                  obj.h_obstacle_target, ...
                                                                  obj.time_since_change);
                    obj.time_since_change = obj.time_since_change + robot_system.dt;
                else
                    obj.h_obstacle = IncreasingBellShapedFunction(0, ...
                                                                  obj.transition_duration, ...
                                                                  obj.h_obstacle_start, ...
                                                                  obj.h_obstacle_target, ...
                                                                  obj.time_since_change);
                    obj.time_since_change = obj.time_since_change + robot_system.dt;
                end
            else
                obj.h_obstacle = obj.h_obstacle_target;
            end
            

            % 1. Select the correct arm
            if(obj.ID == 'L')
                robot = robot_system.left_arm;
            elseif(obj.ID == 'R')
                robot = robot_system.right_arm;    
            end
            
            % 2. Compute and Store Current Relative Altitude
            obj.h_current = robot.wTe(3,4) - obj.h_obstacle;
            
            % 3. Calculate Velocity Reference        
            obj.xdotbar = obj.gain * (obj.h_star - obj.h_current);

            % 4. Saturate velocity for safety
            obj.xdotbar = Saturate(obj.xdotbar, 0.2); 
        end
                      
        function updateJacobian(obj, robot_system)
            % 1. Select the correct arm
            if(obj.ID == 'L')
                robot = robot_system.left_arm;
            elseif(obj.ID == 'R')
                robot = robot_system.right_arm;    
            end
            
            % 2. Extract Linear Z component from EE Jacobian
            % wJe is 6x7 (AngX, AngY, AngZ, LinX, LinY, LinZ)
            % Linear Z velocity is the 6th row
            J_altitude = robot.wJe(6, :); 
            
            % 3. Construct Full Jacobian (1x14) based on robot ID
            if obj.ID == 'L'
                % [ J_L_z (1x7), Zeros (1x7) ]
                obj.J = [J_altitude, zeros(1, 7)];
            elseif obj.ID == 'R'
                % [ Zeros (1x7), J_R_z (1x7) ]
                obj.J = [zeros(1, 7), J_altitude];
            end
        end
        
        function updateActivation(obj, robot_system)     
            obj.A = DecreasingBellShapedFunction(obj.h_min, obj.h_min + obj.buffer, 0, 1, obj.h_current);
        end
    end
end






% classdef MinEffectorAltitudeTask < Task
%     properties
%         h_min = 0.15;   % Minimum allowed altitude (15 cm)
%         buffer = 0.05;  % Buffer zone to smooth activation (starts activating at h_min + buffer)
%         gain = 10.0;     % Strong gain to push back quickly if violated
%         h_table = 0.55;
%         h_star = 0.15 + 0.05;
%     end
% 
%     methods
%         function obj = MinEffectorAltitudeTask(robot_ID, taskID)
%             % Initialize with robot ID (L/R) and task name
%             obj.ID = robot_ID;
%             obj.task_name = taskID;
%         end
% 
%         function updateReference(obj, robot_system)
%             % 1. Select the correct arm
%             if(obj.ID == 'L')
%                 robot = robot_system.left_arm;
%             elseif(obj.ID == 'R')
%                 robot = robot_system.right_arm;    
%             end
% 
%             % 2. Get current altitude (Z component of translation)
%             current_h = robot.wTe(3,4) - obj.h_table;
% 
%             % 3. Calculate error (Target - Current)
%             % Since this is a minimum altitude, if we are below h_min, 
%             % we want to go TO h_min.
%             % We set the target to h_min.            
%             obj.xdotbar = obj.gain * (obj.h_star - current_h);
% 
%             % 4. Saturate velocity for safety
%             % If we are very low, we want a positive velocity (upwards).
%             obj.xdotbar = Saturate(obj.xdotbar, 0.2); 
%         end
% 
%         function updateJacobian(obj, robot_system)
%             % 1. Select the correct arm
%             if(obj.ID == 'L')
%                 robot = robot_system.left_arm;
%             elseif(obj.ID == 'R')
%                 robot = robot_system.right_arm;    
%             end
% 
%             % 2. Extract Linear Z component from EE Jacobian
%             % Usually wJe is 6x7 (AngX, AngY, AngZ, LinX, LinY, LinZ).
%             % Linear Z velocity is the 6th row.
%             J_altitude = robot.wJe(6, :); 
% 
%             % 3. Construct Full Jacobian (1x14)
%             if obj.ID == 'L'
%                 % [ J_L (1x7), Zeros (1x7) ]
%                 obj.J = [J_altitude, zeros(1, 7)];
%             elseif obj.ID == 'R'
%                 % [ Zeros (1x7), J_R (1x7) ]
%                 obj.J = [zeros(1, 7), J_altitude];
%             end
%         end
% 
%         function updateActivation(obj, robot_system)
%             % 1. Select the correct arm for altitude check
%              if(obj.ID == 'L')
%                 robot = robot_system.left_arm;
%             elseif(obj.ID == 'R')
%                 robot = robot_system.right_arm;    
%             end
% 
%             current_h = robot.wTe(3,4) - obj.h_table;
% 
%             % 2. Compute Activation based on Inequality
%             % We use a Decreasing Bell Shaped Function (or similar).
%             % - If h < h_min: Activation = 1 (Full safety active)
%             % - If h > h_min + buffer: Activation = 0 (Task inactive)
%             % - In between: Smooth transition
% 
%             % Syntax: DecreasingBellShapedFunction(lower_bound, upper_bound, min_val, max_val, current_val)
%             % Note: Assuming h_star (where activation starts dropping) is h_min
% 
%             obj.A = DecreasingBellShapedFunction(obj.h_min, obj.h_min + obj.buffer, 0, 1, current_h);
%         end
%     end
% end
