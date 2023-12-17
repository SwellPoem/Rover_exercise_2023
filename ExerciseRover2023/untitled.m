
% PD controller parameters
Kp = 0.9; % Proportional gain
Kd = 0.05; % Derivative gain

% Initialize the rover pose
currentPose = [42.38, 11.59, deg2rad(90)]; % Initial pose

% Initialize simulation time and arrays
dt = 0.1; % Time step
simulationTime = 0:dt:10; % Adjust the end time as needed

% Arrays to store simulation results
x = zeros(size(simulationTime));
y = zeros(size(simulationTime));
theta = zeros(size(simulationTime));

% Define the target pose
targetPose = [33.07, 19.01, deg2rad(180)]; % X1, Y1, θ1

for i = 1:length(simulationTime)
    % Calculate PD control
    errorPose = [targetPose(1:2) - currentPose(1:2), atan2(sin(targetPose(3) - currentPose(3)), cos(targetPose(3) - currentPose(3)))];
    
    v = Kp * errorPose(1:2);
    omega = Kp * errorPose(3) + Kd * (targetPose(3) - currentPose(3));

    % Update rover pose
    currentPose = updatePose(currentPose, v, omega, dt);

    % Store results
    x(i) = currentPose(1);
    y(i) = currentPose(2);
    theta(i) = currentPose(3);
end


% Plot trajectory on the map
figure();
plot(x, y, 'r', 'LineWidth', 2);
hold on;
plot(targetPose(1), targetPose(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
title('Rover Navigation');
xlabel('X-axis (km)');
ylabel('Y-axis (km)');
legend('Rover Trajectory', 'Target Pose');

function newPose = updatePose(currentPose, v, omega, dt)
    % Update the pose using the bicycle model
    x = currentPose(1);
    y = currentPose(2);
    theta = currentPose(3);

    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;

    newPose = [x, y, theta];
end