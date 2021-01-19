clear all
close all
clc
%% Input Parameter (Change k in order to vary end-effector trajectory)
% Conversion factor from centimetres to metres
conv = 0.01;
%Incremental step 
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
Theta1 = [0:h:2*pi];
Theta1_ = Theta1 + pi;
%% Denavit Hartenberg - Forward Kinematic
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
% For example T30_1s means transformation matrix from Joint 3 to Joint 0
% for the 1 closed loop (p), if is considered second branch (s)
%% First closed chain
% Transformation matrix Joint 1 to Joint 0 (workspace)
T10 =  DH(L1,0,0,theta1);
% Transformation matrix Joint 2 to Joint 1
T21 = DH(L2/2,0,0,theta2);
% Joint 2
T20 = T10*T21;
% Transformation matrix Joint 3 to Joint 2
T32 = DH(L3,0,0,theta3);
% Joint 3
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


% Closure equation for first closed loop (Hoeken mechanism)
eq1 = T30_1p(1:3,end) == T30_1s(1:3,end);

% Solution of first closure equation (Hoeken mechanism)
[Theta2,Theta3] = solve(eq1 ,[theta2 theta3]);

% Subs solutions of first closure equation for first leg
angle2 = double(subs(Theta2(2),theta1,Theta1));
angle3 = double(subs(Theta3(2),theta1,Theta1));

% Subs solutions of first closure equation for second leg
angle2_ = double(subs(Theta2(2),theta1,Theta1_));
angle3_ = double(subs(Theta3(2),theta1,Theta1_));


% Closure equation for second closed loop (pantograph)
eq2 = T80_2p(1:3,end) == T80_2s(1:3,end);

% Solution of second closure equation (pantograph)
[Theta4,Theta5] = solve(eq2,[theta4 theta5]);

% Subs solutions of second closure equation for first leg
angle4 = double(subs(Theta4(1),{theta1 theta2,theta3},{Theta1 angle2 angle3}));
angle5 = double(subs(Theta5(1),{theta1 theta2 theta3},{Theta1 angle2 angle3}));

% Subs solutions of second closure equation for second leg
angle4_ = double(subs(Theta4(1),{theta1 theta2,theta3},{Theta1_ angle2_ angle3_}));
angle5_ = double(subs(Theta5(1),{theta1 theta2 theta3},{Theta1_ angle2_ angle3_}));


% Third closure equation (pantograph)
eq3 = T80_3p(1:3,end) == T80_3s(1:3,end);

% Solution of third closure equation (pantograph)
[Theta6,Theta7] = solve(eq3,[theta6 theta7]);

% Subs solutions of third closure equations for the first leg
angle6 = double(subs(Theta6,{theta1 theta2 theta3 theta4 theta5},{Theta1 angle2 angle3 angle4 angle5}));
angle7 = double(subs(Theta7,{theta1 theta2 theta3 theta4 theta5},{Theta1 angle2 angle3 angle4 angle5}));

% Subs solutions of third closure equations for the second leg
angle6_ = double(subs(Theta6,{theta1 theta2 theta3 theta4 theta5},{Theta1_ angle2_ angle3_ angle4_ angle5_}));
angle7_ = double(subs(Theta7,{theta1 theta2 theta3 theta4 theta5},{Theta1_ angle2_ angle3_ angle4_ angle5_}));


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
    J1(:,i) = double(subs(Joint1,{theta1 theta2(1) theta3(1)},{Theta1(i) angle2(i) angle3(i)}));
    J2(:,i) = double(subs(Joint2,{theta1 theta2(1) theta3(1)},{Theta1(i) angle2(i) angle3(i)}));
    J3(:,i) = double(subs(Joint3,{theta1 theta2(1) theta3(1)},{Theta1(i) angle2(i) angle3(i)}));
    J4(:,i) = double(subs(Joint4,{theta1 theta2(1) theta3(1)},{Theta1(i) angle2(i) angle3(i)}));
    J5(:,i) = double(subs(Joint5,{theta1 theta2(1) theta3(1) theta4(1) theta5(1)},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i)}));
    J6(:,i) = double(subs(Joint6,{theta1 theta2(1) theta3(1) theta4(1) theta5(1)},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i)}));
    J7(:,i) = double(subs(Joint7,{theta1 theta2(1) theta3(1) theta4(1) theta5(1) theta6 theta7},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i)}));
    J8(:,i) = double(subs(Joint8,{theta1 theta2(1) theta3(1) theta4(1) theta5(1) theta6 theta7},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i)}));
    J13(:,i) = double(subs(Joint13,{theta1 theta2(1) theta3(1) theta4(1) theta5(1) theta6 theta7},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i)}));
end

%% Movie
figure('units','normalized','outerposition',[0 0 1 1])
view(2)
set(gca,'nextplot','replacechildren');
v = VideoWriter('SingleLegNoFoot.avi');
open(v);
hold on
grid on
for i = 1:length(Theta1)
    clf
    hold on
    grid on
    hold on
    grid on
    axis equal
    xlim([-0.5 1.2]);ylim([-1.6 0.5]);xlabel('X [m]');ylabel('Y [m]')
    title('1 DOF Leg mechanism');
    set(gca,'FontSize',14)
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
    hold on
    %Only one leg
    h1 = plot(J0(1,:),J0(2,:),'blackv','MarkerSize',10);
    text(J0(1,1)+0.02,J0(2,1),'0');
    h2 = plot(J1(1,i),J1(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J1(1,i)-0.01,J1(2,i)+0.03,'1');
    h3 = plot(J2(1,i),J2(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J2(1,i)+0.02,J2(2,i),'2');
    h4 = plot(J3(1,i),J3(2,i),'blackv','MarkerSize',10);
    text(J3(1,1)+0.02,J3(2,1),'3');
    h5 = plot(J4(1,:),J4(2,:),'g','LineWidth',1.5);
    h6 = plot(J4(1,i),J4(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J4(1,i)+0.02,J4(2,i)+0.02,'4');
    h7 = plot(J5(1,i),J5(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J5(1,i)+0.02,J5(2,i),'5');
    h8 = plot(J6(1,i),J6(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J6(1,i)+0.02,J6(2,i),'6');
    h9 = plot(J7(1,i),J7(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J7(1,i)+0.02,J7(2,i),'7');
    h10 = plot(J8(1,i),J8(2,i),'black^','MarkerSize',10);
    text(J8(1,i)+0.02,J8(2,i)+0.01,'8');
    h11 = plot(J13(1,i),J13(2,i));
    text(J13(1,i),J13(2,i)-0.02,'13');
    h22 = plot(J13(1,:),J13(2,:),'black:','LineWidth',1.5);
    legend([h1 h9 h15 h16 h5 h22],{'Hinged Joint','Revolute Joint','Hoeken mechanism','Pantograph mechanism','Hoeken''s figure','Gait pattern'})
    hold off
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);

%% Denavit - Hartenberg Transformation Matrix

function dh = DH(link,a_i,d_i,theta_i)
    dh = [cos(theta_i) -sin(theta_i)*cos(a_i)   sin(theta_i)*sin(a_i)     link*cos(theta_i);
          sin(theta_i)  cos(theta_i)*cos(a_i)   -cos(theta_i)*sin(a_i)     link*sin(theta_i);
               0         sin(a_i)                   cos(a_i)                        d_i;
               0             0                          0                           1       ];
end
