%% Clear cache
clear;clc;close all

%% Initilization

% Animation params
colpos = 1/255*[247 148 30;
                0 166 81;
                237 28 36;
                0 174 239];

% Simulation params
dt = 0.1;
r = 0.036;
d = 0.149;
simParams = [dt r d];
w_max = 30;

tf = 20;
time = 0:dt:tf;

% Robot params
robot1 = dualWheelRobot(simParams,1,colpos(1,:));
robot2 = dualWheelRobot(simParams,2,colpos(2,:));
robot3 = dualWheelRobot(simParams,3,colpos(3,:));
robot4 = dualWheelRobot(simParams,4,colpos(4,:));

robot1_IC = robot1.randomIC;
robot2_IC = robot2.randomIC;
robot3_IC = robot3.randomIC;
robot4_IC = robot4.randomIC;

robot1_q = zeros(length(time),3);
robot1_q(1,:) = robot1_IC;
robot2_q = zeros(length(time),3);
robot2_q(1,:) = robot2_IC;
robot3_q = zeros(length(time),3);
robot3_q(1,:) = robot3_IC;
robot4_q = zeros(length(time),3);
robot4_q(1,:) = robot4_IC;

N = 10;
initMPC();

%% Position control demo
x_d = sin(0.35*time)*2.5;
y_d = cos(0.35*time)*2.5;
theta_d = atan2(y_d, x_d) - pi/2;
theta_d = unwrap(theta_d);
q_d = [x_d; y_d; theta_d]';

figure(2)

for i = 2:length(time)
    [robot1_wL_k,robot1_wR_k] = robot1.MPCController(args, solver, N, q_d(i,:),robot1_q(i-1,:));
    [robot2_wL_k,robot2_wR_k] = robot2.MPCController(args, solver, N, q_d(i,:),robot2_q(i-1,:));
    [robot3_wL_k,robot3_wR_k] = robot3.MPCController(args, solver, N, q_d(i,:),robot3_q(i-1,:));
    [robot4_wL_k,robot4_wR_k] = robot4.MPCController(args, solver, N, q_d(i,:),robot4_q(i-1,:));

    plot(q_d(:,1),q_d(:,2),'k');
    hold on

    robot1_q(i,:) = robot1.discreteModel(robot1_q(i-1,:),robot1_wL_k,robot1_wR_k);
    robot1.plotCurrentPos(robot1_q(i,:))

    robot2_q(i,:) = robot2.discreteModel(robot2_q(i-1,:),robot2_wL_k,robot2_wR_k);
    robot2.plotCurrentPos(robot2_q(i,:))

    robot3_q(i,:) = robot3.discreteModel(robot3_q(i-1,:),robot3_wL_k,robot3_wR_k);
    robot3.plotCurrentPos(robot3_q(i,:))

    robot4_q(i,:) = robot4.discreteModel(robot4_q(i-1,:),robot4_wL_k,robot4_wR_k);
    robot4.plotCurrentPos(robot4_q(i,:))

    plot([0 0], [-10 10], 'k--')
    plot([-10 10], [0 0], 'k--')

    box on
    drawnow
    hold off
    % exportgraphics(gcf,'testAnimated.gif','Append',true);
end

% %% Heading control demo
% theta_d = sin(time)*0+pi;
% 
% figure(1)
% 
% for i = 2:length(time)
% 
%     [robot1_wL_k,robot1_wR_k] = robot1.headingController(theta_d(i),robot1_q(i-1,:));
%     [robot2_wL_k,robot2_wR_k] = robot2.headingController(theta_d(i),robot2_q(i-1,:));
%     [robot3_wL_k,robot3_wR_k] = robot3.headingController(theta_d(i),robot3_q(i-1,:));
%     [robot4_wL_k,robot4_wR_k] = robot4.headingController(theta_d(i),robot4_q(i-1,:));
% 
%     robot1_q(i,:) = robot1.discreteModel(robot1_q(i-1,:),robot1_wL_k,robot1_wR_k);
%     robot1.plotCurrentPos(robot1_q(i,:))
%     hold on
%     robot2_q(i,:) = robot2.discreteModel(robot2_q(i-1,:),robot2_wL_k,robot2_wR_k);
%     robot2.plotCurrentPos(robot2_q(i,:))
% 
%     robot3_q(i,:) = robot3.discreteModel(robot3_q(i-1,:),robot3_wL_k,robot3_wR_k);
%     robot3.plotCurrentPos(robot3_q(i,:))
% 
%     robot4_q(i,:) = robot4.discreteModel(robot4_q(i-1,:),robot4_wL_k,robot4_wR_k);
%     robot4.plotCurrentPos(robot4_q(i,:))
% 
%     drawnow
%     hold off
%     exportgraphics(gcf,'testAnimated.gif','Append',true);
% end
