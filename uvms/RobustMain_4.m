% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
clc; clear; close all;

% Simulation parameters
dt       = 0.05;            % Prima era 0.005
endTime  = 200;
trans_duration = 5;         % 5 s transition duration between actions
% Initialize robot model and simulator
robotModel = UvmsModel(); 
% Set initial conditions
robotModel.eta = [48.5 11.5 -33 0 0 pi/2]';
% Create sim object 
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% Define tasks 
task_vehicle_pos = TaskVehiclePos();
task_vehicle_orient = TaskVehicleOrient();
task_horizontal_att = TaskHorizontalAtt();
task_altitude_min = TaskAltitudeControl(1);
task_altitude_min.set_h_min(1);
task_altitude_landing = TaskAltitudeControl(0);
task_altitude_landing.set_h_star(0);

% Define actions and add to ActionManager
action_safe_navigation = Action("SafeNavigation", ...
    {task_altitude_min, task_vehicle_pos, task_vehicle_orient, task_horizontal_att});

action_landing = Action("Landing", ...
    {task_altitude_landing, task_vehicle_pos, task_horizontal_att});

% The priorities now are defined by the order in the global_task_set, not
% in the single actions anymore
global_task_set = {task_altitude_min, ...       
                            task_altitude_landing, ...
                            task_vehicle_pos, ...
                            task_vehicle_orient, ...
                            task_horizontal_att};

% Define ActionManager and add global_task_set and actions
actionManager = ActionManager(dt, 13, trans_duration);
actionManager.addTaskSet(global_task_set);
actionManager.addAction(action_safe_navigation); 
actionManager.addAction(action_landing); 
actionManager.setCurrentAction("SafeNavigation");         % Set initial action


% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [50 -12.5 -33]';  
w_vehicle_goal_orientation = [0, 0, 0];

% Define error thresholds for safe navigation and landing flag
ang_error_threshold = 0.02;
lin_error_threshold = 0.05;
is_landing = false;

% Set goals in the robot model
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, global_task_set);

% Main simulation loop
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);
    
    % If we completed the safe_navigation action, switch to the landing action
    [vehicle_ang_error, vehicle_lin_error] = CartError(robotModel.wTgv , robotModel.wTv);
    if norm(vehicle_ang_error) < ang_error_threshold && ...
       norm(vehicle_lin_error) < lin_error_threshold && ...
       is_landing == false 
        
        actionManager.setCurrentAction("Landing");
        is_landing = true; 
    end

    % 2. Compute control commands for current action
    % In the first few loops the altitude is not yet received from
    % unity so this leads to an empty activation, so we skip the ICAT
    % for the first 0.5 seconds
    if step * dt > 1
        [v_nu, q_dot] = actionManager.computeICAT(robotModel);
    else
        v_nu = zeros(6,1);
        q_dot = zeros(7,1);
    end

    % 3. Step the simulator (integrate velocities)
    sim.step(v_nu, q_dot);

    % 4. Send updated state to Unity
    unity.send(robotModel);

    % 5. Logging
    logger.update(sim.time, sim.loopCounter);

    % 6. Optional debug prints
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
        % if actionManager.actions(action)
        if is_landing == false
            disp('Current action: Safe Navigation')
            fprintf('Vehicle position error = %.8f m\n', norm(vehicle_lin_error));
            fprintf('Vehicle orientation error = %.8f rad\n', norm(vehicle_ang_error));
        else
            disp('Current action: Landing')
        end
    end

    % 7. Optional real-time slowdown
    % SlowdownToRealtime(dt);
end

% Display plots
logger.plotAll();

% Clean up Unity interface
delete(unity);