function main()
    %Add path
    addpath('./simulation_scripts');
    addpath('./tools')
    addpath('./icat')
    addpath('./tasks')
    close all; 
    clear; 
    clc; 

    %Simulation Parameters
    dt = 0.005;
    end_time = 20;

    % Initialize Franka Emika Panda Model
    model = load("panda.mat");

    %Simulation Setup
    real_robot = false;

    %Initiliaze panda_arm() Class, specifying the base offset w.r.t World Frame
    arm1 = panda_arm(model, eye(4));
    %TO DO: TRANSFORMATION MATRIX FROM WORLD FRAME TO RIGHT ARM BASE FRAME
    wTb2 = [-1  0   0   1.06;
            0   -1  0   -0.01;
            0   0   1   0;
            0   0   0   1];
    arm2 = panda_arm(model, wTb2);
 
    %Initialize Bimanual Simulator Class
    bm_sim = bimanual_sim(dt, arm1, arm2, end_time);
    
    %Define Object Shape and origin Frame

    obj_length = 0.12;

    w_obj_pos = [0.5 0 0.59]';

    w_obj_ori = rotation(0, 0, 0);

    %Set goal frames for left and right arm, based on object frame
    
    %TO DO: Set arm goal frame based on object frame.
    offset = (obj_length/2) - 0.01; % offset with a margin to not take the obj exactly at the end
    linear_offset = [offset 0 0]';
    
    arm1.setGoal(w_obj_pos, w_obj_ori, -linear_offset, rotation(pi, -pi/6, 0));
    
    arm2.setGoal(w_obj_pos, w_obj_ori, +linear_offset, rotation(pi, pi/6, 0));



    %Define Object goal frame (Cooperative Motion)
    wTog = [rotation(0, 0, 0) [0.65, -0.35, 0.28]'; 0 0 0 1];
    arm1.set_obj_goal(wTog)
    arm2.set_obj_goal(wTog)

    %Define Tasks, input values(Robot type(L,R,BM), Task Name)
    left_tool_task = ToolTask("L", "LT");
    right_tool_task = ToolTask("R", "RT");

    % ----- task joint marti prova
    left_joint_task = JointLimitsTask("L", "LJ");
    right_joint_task = JointLimitsTask("R", "RJ");
    % -----

    % Task minimum altitude
    left_min_ee_alt_task = MinEffectorAltitudeTask("L", "LMA");
    right_min_ee_alt_task = MinEffectorAltitudeTask("R", "RMA");



    %Actions for each phase: go to phase, coop_motion phase, end_motion phase
    action_go_to = Action("ReachObject", {left_tool_task, right_tool_task, ...
                                          left_joint_task, right_joint_task ...
                                          left_min_ee_alt_task, right_min_ee_alt_task});

    % order define priority { HIGHEST , ... , lowest}
    % global_list = {left_tool_task, right_tool_task};
    global_list = {left_joint_task, right_joint_task, ...
                   left_min_ee_alt_task, right_min_ee_alt_task, ...
                   left_tool_task, right_tool_task};

    %Load Action Manager Class and load actions
    actionManager = ActionManager(dt, 14, 3);
    actionManager.addTaskSet(global_list);
    actionManager.addAction(action_go_to);

    %Initiliaze robot interface
    robot_udp = UDP_interface(real_robot);

    %Initialize logger
    % logger = SimulationLogger(ceil(end_time/dt)+1, bm_sim, global_list);

    %Main simulation Loop
    for t = 0:dt:end_time
        % 1. Receive UDP packets - DO NOT EDIT
        [ql, qr] = robot_udp.udp_receive(t);
        if real_robot == true %Only in real setup, assign current robot configuration as initial configuratio
            bm_sim.left_arm.q = ql;
            bm_sim.right_arm.q = qr;
        end

        % 2. Update Full kinematics of the bimanual system
        bm_sim.update_full_kinematics();

        % 3. Compute control commands for current action
        [q_dot] = actionManager.computeICAT(bm_sim);

        % 4. Step the simulator (integrate velocities)
        bm_sim.sim(q_dot);

        % 5. Send updated state to Pybullet
        robot_udp.send(t, bm_sim)

        % 6. Logging
        % logger.update(bm_sim.time, bm_sim.loopCounter)
        bm_sim.time

        % 7. Optional real-time slowdown
        % SlowdownToRealtime(dt);
    end

    %Display joint position and velocity, Display for a given action, a number
    %of tasks
    action = 1;
    tasks = [1];
    % logger.plotAll(action, tasks);
end
