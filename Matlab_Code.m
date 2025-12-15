% Import robot model
robot = importrobot("Cobot\myCobot_Pro_600_2.urdf");
robot.DataFormat = 'row';
% show(robot);

% % Run Python script and get coordinates
% robot_coords = pyrunfile("Code.py", "Return_List");
    
% % Convert Python numpy array to MATLAB array
% robot_coords = double(robot_coords);

robot_coords = readmatrix('robot_trajectory.csv');
disp(robot_coords);

% Get number of waypoints
num_waypoints = size(robot_coords, 1);

% Initialize joint angles array
joint_angles = zeros(num_waypoints, 6);  % Assuming 6 joints
digital_joint_angles = zeros(num_waypoints, 6);  % Assuming 6 joints

% Setup inverse kinematics solver
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25, 0.25, 0.25, 1, 1, 1];
initialguess = robot.homeConfiguration;

% Process each waypoint
disp("Solving Inverse Kinematics for each waypoint:");
for i = 1:num_waypoints
    % Get current waypoint (convert from mm to m if necessary)
    waypoint = robot_coords(i, :) / 1000;  % Convert mm to m if needed
    
    % Create transformation matrix
    endEffectorPose = trvec2tform(waypoint);
    
    % Solve IK
    [configsol, solInfo] = ik('Link_6', endEffectorPose, weights, initialguess);
    
    if strcmp(solInfo.Status, 'success')
        % Convert to degrees and apply adjustments
        joint_angles_degrees = rad2deg(configsol);
        digital_joint_angles_deg = rad2deg(configsol);
        joint_angles_degrees(1) = joint_angles_degrees(1) + 180;
        joint_angles_degrees(2) = joint_angles_degrees(2) - 90;
        joint_angles_degrees(4) = joint_angles_degrees(4) - 90;
        joint_angles_degrees(5) = joint_angles_degrees(5) * (-1);
        
        % Store result
        joint_angles(i, :) = joint_angles_degrees;
        
        % Update initial guess for next iteration
        initialguess = configsol;
    else
        warning('No solution found for waypoint %d', i);
        joint_angles(i, :) = nan(1, 6);
    end
end

% Save joint angles
csvwrite('joint_angles.csv', joint_angles);


% Display joint angles
disp('Joint angles for each waypoint (degrees):');
disp(joint_angles);

% Execute additional Python script
% pyrunfile("TCP_Code.py");