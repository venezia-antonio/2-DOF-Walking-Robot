clear all
close all
clc
%% Input Parameter (Change k in order to vary end-effector trajectory)
% Conversion factor from centimetres to metres
conv = 0.01; 
% Incremental step
h = 0.1;  
% Lenght of crank (link L1) in [m]
l = 5*conv;
k = 2;
% Length of links of mechanism
L1 = l;
L2 = 5*l;
L3 = 2.5*l;
L4 = 9*l;
L5 = 18*l;
L6 = 6*l;
L7 = (2/3)*L4;
L8 = 4.2*l;
L9 = L4;
L10 = L8/2;
L11 = L5;
% Length of virtual links for closed loop equations
L0 = k*L1;
L12 = 4*l;
L13 = 12*l;
% Length of foot
frontFootLength = 1.5*l;
backFootLength = 0.5*l;
% Actuator - Costant angular velocity
Theta1 = [0:h:2*pi+0.2];
Theta1_ = Theta1 + pi;
%% Denavit Hartenberg - Forward Kinematic
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7

%% First closed chain
% Transformation matrix Joint 1 to Joint 0 (workspace)
T10 =  DH(L1,0,0,theta1);
% Transformation matrix Joint 2 to Joint 1
T21 = DH(L2/2,0,0,theta2);
% Joint 2
T20 = T10*T21;
% Transformation matrix Joint 3 to Joint 2
T32 = DH(L3,0,0,theta3);
% Joint 3 (first branch)
T30_1p = T10*T21*T32;
% Transformation matrix Joint 3 to Joint 0
T30_1s = DH(L0,0,0,0);

%% Second closed chain
% Transformation matrix Joint 4 to Joint 2
T42 = DH(L2/2,0,0,0);
% Joint 4
T40 = T10*T21*T42;
% Transformation matrix Joint 5 to Joint 4
T54 = DH((1/3)*L4,0,0,theta4);
% Joint 5
T50 = T40*T54;
% Transformation matrix Joint 8 to Joint 5
T85 = DH(L6,0,0,theta5);
% Joint 8 (first branch)
T80_2p = T50*T85;
% Transfromation matrix Joint 3 to Joint 8
T83 = DH(L13,0,0,3*(pi/2));
% Joint 8 (second branch)
T80_2s = T30_1s*T83;

%% Third closed chain
% Transformation matrix Joint 6 to Joint 5
T65 = DH((2/3)*L4,0,0,0);
% Joint 6
T60 = T50*T65;
% Transformation matrix Joint 7 to Joint 6
T76 = DH((1/3)*L5,0,0,theta6);
% Joint 7
T70 = T60*T76;
% Joint 8 (first branch)
T80_3p = T80_2p;
% Transformation matrix Joint 7 to Joint 8
T87 = DH(L7,0,0,theta7);
% Joint 8 (second branch)
T80_3s = T70*T87;
% Transformation matrix Joint 13 to Joint 7
T13_7 = DH((2/3)*L5,0,0,0);
% Joint 13
T13_0 = T70*T13_7;

% Define options of fminsearch method
options = optimset('TolFun',1e-14,'TolX',1e-14,'UseParallel',true);

% Closure equation for first closed loop (Hoeken mechanism)
eq1 = T30_1p(1:3,end) == T30_1s(1:3,end);
eq1 = lhs(eq1)-rhs(eq1);

% Solution of first closure equation using numerical solver
for i = 1:length(Theta1)
    
    tmp = subs(eq1,theta1,Theta1(i));
    eq = tmp(1)^2 + tmp(2)^2;
    eq1H = matlabFunction(eq);
    eq1Mod = @(x)eq1H(x(1),x(2));
    if i==1
        x0 = [3*pi/2,pi/2];
        x = fminsearch(eq1Mod,x0,options);  % x = [theta2 theta3]
        angle2(i) = x(1);
        angle3(i) = x(2);
    else
        x0 = [angle2(i-1),angle3(i-1)];
        x = fminsearch(eq1Mod,x0,options);  % x = [theta2 theta3]
        angle2(i) = x(1);
        angle3(i) = x(2);
    end
end

% Closure equation for second closed loop (pantograph)
eq2 = T80_2p(1:3,end) == T80_2s(1:3,end);
eq2 = lhs(eq2)-rhs(eq2);

% Solution of second closure equation using numerical solver
for i = 1:length(Theta1)
    
    tmp = subs(eq2,{theta1,theta2,theta3},{Theta1(i) angle2(i) angle3(i)});
    eq = tmp(1)^2 + tmp(2)^2;  
    eq1H = matlabFunction(eq);
    eq1Mod = @(x)eq1H(x(1),x(2));
    if i==1
        x0 = [pi/4,3*pi/2];
        x = fminsearch(eq1Mod,x0);  % x = [theta4 theta5]
        angle4(i) = x(1);
        angle5(i) = x(2);
    else
        x0 = [angle4(i-1),angle5(i-1)];
        x = fminsearch(eq1Mod,x0,options);  % x = [theta4 theta5]
        angle4(i) = x(1);
        angle5(i) = x(2);
    end
end

% Closure equation for third closed loop (pantograph)
eq3 = T80_3p(1:3,end) == T80_3s(1:3,end);
eq3 = lhs(eq3)-rhs(eq3);

% Solution of third closure equation using numerical solver
for i = 1:length(Theta1)
    
    tmp = subs(eq3,{theta1,theta2,theta3,theta4,theta5},{Theta1(i),angle2(i),angle3(i),angle4(i),angle5(i)});
    eq = tmp(1)^2 + tmp(2)^2;  
    eq1H = matlabFunction(eq);
    eq1Mod = @(x)eq1H(x(1),x(2));
    if i==1
        x0 = [3*pi/2,3*pi/2];
        x = fminsearch(eq1Mod,x0);  % x = [theta6 theta7]
        angle6(i) = x(1);
        angle7(i) = x(2);
    else
        x0 = [angle6(i-1),angle7(i-1)];
        x = fminsearch(eq1Mod,x0,options);  % x = [theta6 theta7]
        angle6(i) = x(1);
        angle7(i) = x(2);
    end
end


% Joint vector position
J0 = [0 0 0]';
Joint1 = T10(1:3,end);
Joint2 = T20(1:3,end);
Joint3 = T30_1p(1:3,end);
Joint4 = T40(1:3,end);
Joint5 = T50(1:3,end);
Joint6 = T60(1:3,end);
Joint7 = T70(1:3,end);
Joint8 = T80_2p(1:3,end);
Joint13 = T13_0(1:3,end);


% Compute joint trajectory for the first leg
for i = 1:length(Theta1)
    J1(:,i) = double(subs(Joint1,{theta1 theta2 theta3},{Theta1(i) angle2(i) angle3(i)}));
    J2(:,i) = double(subs(Joint2,{theta1 theta2 theta3},{Theta1(i) angle2(i) angle3(i)}));
    J3(:,i) = double(subs(Joint3,{theta1 theta2 theta3},{Theta1(i) angle2(i) angle3(i)}));
    J4(:,i) = double(subs(Joint4,{theta1 theta2 theta3},{Theta1(i) angle2(i) angle3(i)}));
    J5(:,i) = double(subs(Joint5,{theta1 theta2 theta3 theta4 theta5},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i)}));
    J6(:,i) = double(subs(Joint6,{theta1 theta2 theta3 theta4 theta5},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i)}));
    J7(:,i) = double(subs(Joint7,{theta1 theta2 theta3 theta4 theta5 theta6 theta7},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i)}));
    J8(:,i) = double(subs(Joint8,{theta1 theta2 theta3 theta4 theta5 theta6 theta7},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i)}));
    J13(:,i) = double(subs(Joint13,{theta1 theta2 theta3 theta4 theta5 theta6 theta7},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i)}));
end


%% Load data from single leg mechanism
% This section is used in order to not run the previous code
load('DifferentialKinematics.mat');

%% Geometric Jacobian of open chain obtained cutting the closed chain in joint 2,5,7.

% Jacobian for joint 1
Ja1 = Jacobian(eye(4),T13_0,'R');   %da1
% Jacobian for joint 4
Ja2 = Jacobian(T10,T13_0,'R');      %da2
% Jacobian for joint 6
Ja3 = Jacobian(T40,T13_0,'R');      %da4
% Jacobian for joint 7
Ja4 = Jacobian(T60,T13_0,'R');      %da5

% 6x4 jacobian matrix
Ja = [Ja1 Ja2 Ja3 Ja4];

%% Compute derivative data from angles
% Compute derivative of joint variables for the open chain obtained
% virtually cutting unactuated joints.
% It's considered an angular velocity of motor crank equal to 0.1 rad/s
for i = 1:length(Theta1)-1
    da1(i) = ((Theta1(i+1))-(Theta1(i)));
    da2(i) = ((angle2(i+1))-(angle2(i)));
    da4(i) = ((angle4(i+1))-(angle4(i)));
    da5(i) = ((angle5(i+1))-(angle5(i)));
end

%Initialization of vector of linear and angular velocity of end effector
v =zeros(6,1);

%% Define profile of linear velocity of end effector

for i = 1:length(Theta1)-1
    Jacobiano = double(subs(Ja,{theta1,theta2,theta3,theta4,theta5,theta6,theta7},{Theta1(i),angle2(i),angle3(i),angle4(i),angle5(i),angle6(i),angle7(i)}));
    v(:,i) = Jacobiano*[da1(i);da2(i);da4(i);da5(i)];
end
%% Define profile of linear acceleration of end effector

for i = 1:length(Theta1)-2
    linAccX(i) = (v(1,i+1)-v(1,i));
    linAccY(i) = (v(2,i+1)-v(2,i));
    %angAccZ(i) = (v(6,i+1)-v(6,i));
end

%% Plot linear velocity and acceleration
figure()
sgtitle('Differential kinematics','Color','red','FontSize',20)
subplot(1,2,1)
title('Linear velocity of end effector');
hold on
grid on
plot(h*(1:length(v)),v(1,:),'r');
plot(h*(1:length(v)),v(2,:),'g');
%plot(h*(1:length(v)),v(6,:),'b');
xlabel('Time [s]')
ylabel('Linear velocity [m/s]')
legend('v_x','v_y','Location','north');
subplot(1,2,2)
title('Linear acceleration of end effector');
hold on
grid on
plot(h*(1:length(linAccX)),linAccX,'r');
plot(h*(1:length(linAccY)),linAccY,'g');
%plot(h*(1:length(angAccZ)),angAccZ,'b');
xlabel('Time [s]')
ylabel('Linear acceleration [m/s^2]')
legend('a_x','a_y','Location','north');
hold off

%% Movie
figure('units','normalized','outerposition',[0 0 1 1])
view(2)
set(gca,'nextplot','replacechildren');
video = VideoWriter('DifferentialKinematics.avi');
open(video);
hold on
grid on
for i = 1:length(Theta1)-2
    clf
    hold on
    grid on
    hold on
    grid on
    sgtitle('Differential kinematics','Color','red')
    subplot(1,3,1)
    grid on
    hold on
    xlim([-0.5 1.2]);ylim([-1.6 0.5]);
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Single Leg mechanism');
    hold on
    h5 = plot(J4(1,:),J4(2,:),'g','LineWidth',1.5);
    h12 = plot([J0(1,1),J1(1,i)],[J0(2,1),J1(2,i)],'r','LineWidth',2);
    h13 = plot([J1(1,i),J2(1,i)],[J1(2,i),J2(2,i)],'r','LineWidth',2);
    h14 = plot([J2(1,i),J3(1,i)],[J2(2,i),J3(2,i)],'r','LineWidth',2);
    h15 = plot([J2(1,i),J4(1,i)],[J2(2,i),J4(2,i)],'r','LineWidth',2);
    h16 = plot([J4(1,i),J5(1,i)],[J4(2,i),J5(2,i)],'b','LineWidth',2);
    h17 = plot([J5(1,i),J6(1,i)],[J5(2,i),J6(2,i)],'b','LineWidth',2);
    h18 = plot([J5(1,i),J8(1,i)],[J5(2,i),J8(2,i)],'b','LineWidth',2);
    h19 = plot([J6(1,i),J7(1,i)],[J6(2,i),J7(2,i)],'b','LineWidth',2);
    h20 = plot([J7(1,i),J8(1,i)],[J7(2,i),J8(2,i)],'b','LineWidth',2);
    h21 = plot([J7(1,i),J13(1,i)],[J7(2,i),J13(2,i)],'b','LineWidth',2);
    % Single leg
    h1 = plot(J0(1,:),J0(2,:),'blackv','MarkerSize',10);
    text(J0(1,1)-0.1,J0(2,1),'0');
    h2 = plot(J1(1,i),J1(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J1(1,i)-0.01,J1(2,i)+0.03,'1');
    h3 = plot(J2(1,i),J2(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J2(1,i)+0.02,J2(2,i),'2');
    h4 = plot(J3(1,i),J3(2,i),'blackv','MarkerSize',10);
    text(J3(1,1)+0.05,J3(2,1),'3')
    h6 = plot(J4(1,i),J4(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J4(1,i)-0.05,J4(2,i),'4');
    h7 = plot(J5(1,i),J5(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J5(1,i)-5,J5(2,i),'5');
    h8 = plot(J6(1,i),J6(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J6(1,i)+0.02,J6(2,i),'6');
    h9 = plot(J7(1,i),J7(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J7(1,i)+0.02,J7(2,i),'7');
    h10 = plot(J8(1,i),J8(2,i),'black^','MarkerSize',10);
    text(J8(1,i)-0.06,J8(2,i)+0.01,'8');
    h11 = plot(J13(1,i),J13(2,i));
    text(J13(1,i)-0.01,J13(2,i)-0.02,'13');
    h22 = plot(J13(1,:),J13(2,:),'black:','LineWidth',1.5);
    h23 = quiver(J13(1,i),J13(2,i),5*v(1,i),5*v(2,i),'color','magenta','Linewidth',1.5,'MaxHeadSize',15,'AutoScale','off');
    legend([h1 h9 h15 h16 h5 h23 h22],{'Hinged Joint','Revolute Joint','Hoeken mechanism','Pantograph mechanism','Hoeken''s figure','Vector of linear velocity','Gait pattern'})
    subplot(1,3,2)
    grid on
    hold on
    xlim([0 7]);ylim([-0.02 0.06]);
    xlabel('Time [s]')
    ylabel('Linear velocity [m/s]')
    title('Linear velocity of end effector')
    h24 = plot(h*(1:i),v(1,1:i),'r');
    h25 = plot(h*(1:i),v(2,1:i),'g');
    legend([h24 h25],{'v_x','v_y'});
    subplot(1,3,3)
    grid on
    hold on
    xlim([0 7]);ylim([-0.02 0.02]);
    xlabel('Time [s]')
    ylabel('Linear acceleration  [m/s^2]')
    title('Linear acceleration of end effector');
    h26 = plot(h*(1:i),linAccX(1,1:i),'r');
    h27 = plot(h*(1:i),linAccY(1,1:i),'g');
    legend([h26 h27],{'a_x','a_y'});
    hold off
    frame = getframe(gcf);
    writeVideo(video,frame);
end
close(video);

%% Denavit - Hartenberg Transformation Matrix

function dh = DH(link,a_i,d_i,theta_i)
    dh = [cos(theta_i) -sin(theta_i)*cos(a_i)   sin(theta_i)*sin(a_i)     link*cos(theta_i);
          sin(theta_i)  cos(theta_i)*cos(a_i)   -cos(theta_i)*sin(a_i)     link*sin(theta_i);
               0         sin(a_i)                   cos(a_i)                        d_i;
               0             0                          0                           1       ];
end

