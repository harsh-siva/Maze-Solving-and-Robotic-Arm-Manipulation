% Import robot model
robot = importrobot("Cobot\myCobot_Pro_600_2.urdf");
robot.DataFormat = 'row';

robot_coords = readmatrix('robot_trajectory.csv');
disp(robot_coords);

% Get number of waypoints
num_waypoints = size(robot_coords, 1);

% Initialize joint angles array
digital_joint_angles = zeros(num_waypoints, 6);  % Assuming 6 joints

% Setup inverse kinematics solver
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25, 0.25, 0.25, 1, 1, 1];
initialguess = robot.homeConfiguration;

% Define target end-effector orientation (Euler angles in degrees)
targetEulerAngles = [-180; 0; 180];  % Yaw, Pitch, Roll in degrees

% Convert orientation to a rotation matrix (ZYX Euler angles)
targetOrientation = eul2rotm(deg2rad(targetEulerAngles'), 'ZYX');

% Process each waypoint
disp("Solving Inverse Kinematics for each waypoint:");
for i = 1:num_waypoints
    % Get current waypoint (convert from mm to m if necessary)
    waypoint = robot_coords(i, :) / 1000;  % Convert mm to m if needed
    
    % Define target pose with position and orientation
    targetPosition = waypoint; % Update target position for current step
    targetPose = trvec2tform(targetPosition); % Create translation matrix
    targetPose(1:3, 1:3) = targetOrientation; % Apply orientation (rotation matrix)
    
    % Solve IK
    [configsol, solInfo] = ik('Link_6', targetPose, weights, initialguess);
    
    if strcmp(solInfo.Status, 'success')
        % Convert to degrees and apply adjustments
        digital_joint_angles_deg = rad2deg(configsol);
        digital_joint_angles_deg(1) = digital_joint_angles_deg(1);
        digital_joint_angles_deg(2) = digital_joint_angles_deg(2);
        digital_joint_angles_deg(3) = digital_joint_angles_deg(3);
        digital_joint_angles_deg(4) = digital_joint_angles_deg(4);
        digital_joint_angles_deg(5) = digital_joint_angles_deg(5);
        
        % Store result
        digital_joint_angles(i, :) = digital_joint_angles_deg;
        
        % Update initial guess for next iteration
        initialguess = configsol;
    else
        warning('No solution found for waypoint %d', i);
        digital_joint_angles(i, :) = nan(1, 6);
    end
end

% Visualize the path in the digital twin
figure;
show(robot, robot.homeConfiguration, 'Frames', 'off');
axis([-1 1 -1 1 -0.5 1]); % Adjust as needed to fit the robot
view(3); % Set 3D view
hold on;

% Convert robot_coords to path format for plotting
path = robot_coords' / 1000; % Convert to meters and transpose for plotting
plot3(path(1, :), path(2, :), path(3, :), 'r-', 'LineWidth', 2); % Path in space

% Animate robot movement through all points
for i = 1:num_waypoints

        for j = 1:length(configsol)
            configsol(j) = deg2rad(digital_joint_angles(i, j));
        end
        
        % Show robot at current configuration
        show(robot, configsol, 'Frames', 'off', 'PreservePlot', false);
        pause(0.1); % Pause for animation
    % end
end

title('Path Planning and Digital Twin Visualization');
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

% Display joint angles
disp('Joint angles for each waypoint (degrees):');
disp(digital_joint_angles);