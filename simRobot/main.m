%% Clear cache
clear;clc;close all

%% Initilization

% Animation params
colpos = 1/255*[247 148 30;
                0 166 81;
                237 28 36;
                0 174 239;
                243 255 20;
                243 20 255];

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
robot5 = dualWheelRobot(simParams,5,colpos(5,:));
robot6 = dualWheelRobot(simParams,6,colpos(6,:));

robot1_IC = [0,0,0];%robot1.randomIC;
robot2_IC = [-10,10,0];%robot2.randomIC;
robot3_IC = [0,0,0];%robot3.randomIC;
robot4_IC = [-10,10,0];%robot4.randomIC;
robot5_IC = [0,0,0];
robot6_IC = [-10,10,0];

robot1_q = zeros(length(time),3);
robot1_q(1,:) = robot1_IC;
robot2_q = zeros(length(time),3);
robot2_q(1,:) = robot2_IC;
robot3_q = zeros(length(time),3);
robot3_q(1,:) = robot3_IC;
robot4_q = zeros(length(time),3);
robot4_q(1,:) = robot4_IC;
robot5_q = zeros(length(time),3);
robot5_q(1,:) = robot5_IC;
robot6_q = zeros(length(time),3);
robot6_q(1,:) = robot6_IC;

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
    figure(2)
    [robot1_wL_k,robot1_wR_k] = robot1.positionController(q_d(i,:),robot1_q(i-1,:), 1);
    [robot2_wL_k,robot2_wR_k] = robot2.positionController(q_d(i,:),robot2_q(i-1,:), 1);
    [robot3_wL_k,robot3_wR_k] = robot3.MPCController(args, solver, N, q_d(i,:),robot3_q(i-1,:));
    [robot4_wL_k,robot4_wR_k] = robot4.MPCController(args, solver, N, q_d(i,:),robot4_q(i-1,:));
    [robot5_wL_k,robot5_wR_k] = robot5.positionController(q_d(i,:),robot5_q(i-1,:), 5);
    [robot6_wL_k,robot6_wR_k] = robot6.positionController(q_d(i,:),robot6_q(i-1,:), 5);

    plot(q_d(:,1),q_d(:,2),'k',q_d(i,1),q_d(i,2),'o');
    hold on

    robot1_q(i,:) = robot1.discreteModel(robot1_q(i-1,:),robot1_wL_k,robot1_wR_k);
    robot1.plotCurrentPos(robot1_q(i,:))

    robot2_q(i,:) = robot2.discreteModel(robot2_q(i-1,:),robot2_wL_k,robot2_wR_k);
    robot2.plotCurrentPos(robot2_q(i,:))

    robot3_q(i,:) = robot3.discreteModel(robot3_q(i-1,:),robot3_wL_k,robot3_wR_k);
    robot3.plotCurrentPos(robot3_q(i,:))

    robot4_q(i,:) = robot4.discreteModel(robot4_q(i-1,:),robot4_wL_k,robot4_wR_k);
    robot4.plotCurrentPos(robot4_q(i,:))

    robot5_q(i,:) = robot5.discreteModel(robot5_q(i-1,:),robot5_wL_k,robot5_wR_k);
    robot5.plotCurrentPos(robot5_q(i,:))

    robot6_q(i,:) = robot6.discreteModel(robot6_q(i-1,:),robot6_wL_k,robot6_wR_k);
    robot6.plotCurrentPos(robot6_q(i,:))

    plot([0 0], [-10 10], 'k--')
    plot([-10 10], [0 0], 'k--')
    
    box on
    drawnow
    hold off

    
    % exportgraphics(gcf,'testAnimated.gif','Append',true);
end

figure(3)
axis([0 time(length(time)) 0 5])
robot1.plotDistToRef(time, i, robot1_q, q_d)
robot2.plotDistToRef(time, i, robot2_q, q_d)
robot3.plotDistToRef(time, i, robot3_q, q_d)
robot4.plotDistToRef(time, i, robot4_q, q_d)
robot5.plotDistToRef(time, i, robot5_q, q_d)
robot6.plotDistToRef(time, i, robot6_q, q_d)
axis([0 time(length(time)) 0 5])
legend('Position Controller k_v = 1, Origin', 'Position Controller k_v = 1, Corner', 'MPC, Origin', 'MPC, Corner', 'Position Controller k_v = 5, Origin', 'Position Controller k_v = 5, Corner')
title('Distance to Reference Over Time')
xlabel('Time (s)')
ylabel('Distance')
hold off
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
