classdef ActionManager < handle
    properties
        dt = 0.05
        task_set = {}           % Contains all the possible tasks
        actions = {}            % cell array of actions (each action = stack of tasks)
        currentAction = 1       % index of currently active action
        previousAction = 1      % index of the previously active action
        timeInCurrentAction = 0 % Time elapsed since setCurrentAction was called
        transitionDuration = 5  % 2 second transition time between actions
    end

    methods

        function addTaskSet(obj, taskSet)
            obj.task_set = taskSet;
        end

        function addAction(obj, taskStack)
            % taskStack: cell array of tasks that define an action
            obj.actions{end+1} = taskStack;
        end

        function [qdot] = computeICAT(obj, robot)
        % function [v_nu, qdot] = computeICAT(obj, robot)
            % Get current action
            current_tasks = obj.actions{obj.currentAction};
            previous_tasks = obj.actions{obj.previousAction};

            % Perform ICAT (task-priority inverse kinematics)
            ydotbar = zeros(14,1);
            Qp = eye(14);
            % ydotbar = zeros(13,1);
            % Qp = eye(13);

            for i = 1:length(obj.task_set)  % Iterate on ALL of the possible tasks

                % Extract task
                task = obj.task_set{i};

                % Check if task is contained in current and/or previous action
                inCurrent = false;
                for k = 1:length(current_tasks)
                    if task == current_tasks{k}, inCurrent = true; break; end
                end
                inPrevious = false;
                for k = 1:length(previous_tasks)
                    if task == previous_tasks{k}, inPrevious = true; break; end
                end
                
                % Determine task status
                trans_act = 0;
                if ~inPrevious && inCurrent
                    trans_act = IncreasingBellShapedFunction(0, ...
                                                            obj.transitionDuration, ...
                                                            0, ...
                                                            1, ...
                                                            obj.timeInCurrentAction);
                elseif inPrevious && ~inCurrent
                    trans_act = DecreasingBellShapedFunction(0, ...
                                                            obj.transitionDuration, ...
                                                            0, ...
                                                            1, ...
                                                            obj.timeInCurrentAction);
                elseif inPrevious && inCurrent
                    trans_act = 1;
                end
                
                % Update tasks
                task.updateReference(robot);
                task.updateJacobian(robot);
                task.updateActivation(robot);
                
                % If inactive, we can skip computation to save some time
                if trans_act == 0
                    continue; 
                end
                
                % Perform ICAT Step
                [Qp, ydotbar] = iCAT_task(trans_act*task.A, task.J, ...
                                           Qp, ydotbar, task.xdotbar, ...
                                           1e-4, 0.01, 10);
            end

            % 3. Last task: residual damping
            % [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);
            [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);

            % 4. Split velocities for vehicle and arm
            % qdot = ydotbar(1:7);
            % v_nu = ydotbar(8:13); % projected on the vehicle frame
            qdot = ydotbar;

            % 5. Increment time elapsed since last action switch
            obj.timeInCurrentAction = obj.timeInCurrentAction + obj.dt;
        end

        function setCurrentAction(obj, actionIndex)
            % Switch to a different action
            if actionIndex >= 1 && actionIndex <= length(obj.actions)
                obj.previousAction = obj.currentAction;
                obj.currentAction = actionIndex;
                obj.timeInCurrentAction = 0;
            else
                error('Action index out of range');
            end
        end
    end
end