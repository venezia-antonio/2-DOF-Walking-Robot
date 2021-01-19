%% Prova single leg stab foot
clear all
close all
clc
%% Input Parameter ( Change k in order to vary end-effector trajectory )
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
Theta1 = [0:h:2*pi];
Theta1_ = Theta1 + pi;

%% Denavit Hartenberg - Forward Kinematic
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
% For example T30_1s means transformation matrix from Joint 3 to Joint 0
% for the 1 closed loop (p), if second branch is considered (s)
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
T83 = DH(L13,0,0,3*sym(pi/2));
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

%% Closed loop for parallel extension mechanism
syms theta9 theta10 theta11 theta12 theta13 x

% Transformation matrix Joint 10 to Joint 4
T10_4 = DH(L8,0,0,theta10);
% Joint 10 (first branch)
T10_0p = T40*T10_4;
% Transformation matrix Joint 10~ to Joint 0
T10tilde_0 = DH(L12,0,0,3*sym(pi/2));
% Transformation matrix Joint 10^ to Joint 10~
T10hat_10tilde = DH(x,0,0,3*sym(pi/2));
% Joint 10 (second branch)
T10_0s = T10tilde_0*T10hat_10tilde;
% Transformation matrix Joint 9 to Joint 4
T94 = DH(L8/2,0,0,theta10);
%Joint 9 
T90 = T40*T94;
% Transformation matrix Joint 11 to Joint 9
T11_9 = DH(L9,0,0,theta9);
% Joint 11 (first branch)
T11_0p = T90*T11_9;
% Transformation matrix Joint 11 to Joint 6
T11_6 = DH(L10,0,0,theta11);
% Joint 11 (second branch)
T11_0s = T60*T11_6;
% Transformation matrix Joint 12 to Joint 11
T12_11 = DH(L11,0,0,theta12);
% Joint 12
T12_0 = T60*T11_6*T12_11;
% Transformation matrix Joint 13 to Joint 12
T13_12 = DH(L10,0,0,theta13);
% Joint 13 (second branch)
T13_0s = T12_0*T13_12;
% Transformation matrix FrontFoot to Joint 13
TFF_13 = DH(frontFootLength,0,0,theta13);
% Front foot
TFF_0 = T13_0*TFF_13;
% Transformation matrix BackFoot to Joint 13
TBF_FF = DH(backFootLength+frontFootLength+L10,0,0,sym(pi));
% Back foot
TBF_0 = TFF_0*TBF_FF;

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

% Closure equation for third closed loop (pantograph)
eq3 = T80_3p(1:3,end) == T80_3s(1:3,end);

% Solution of third closure equation (pantograph)
[Theta6,Theta7] = solve(eq3,[theta6 theta7]);

% Subs solutions of third closure equation for first leg
angle6 = double(subs(Theta6,{theta1 theta2 theta3 theta4 theta5},{Theta1 angle2 angle3 angle4 angle5}));
angle7 = double(subs(Theta7,{theta1 theta2 theta3 theta4 theta5},{Theta1 angle2 angle3 angle4 angle5}));

% Subs solutions of third closure equation for second leg
angle6_ = double(subs(Theta6,{theta1 theta2 theta3 theta4 theta5},{Theta1_ angle2_ angle3_ angle4_ angle5_}));
angle7_ = double(subs(Theta7,{theta1 theta2 theta3 theta4 theta5},{Theta1_ angle2_ angle3_ angle4_ angle5_}));

% Closure equation for first closed loop (parallel extension)
eq4 = T10_0p(1:3,end) == T10_0s(1:3,end);

% Solution of first closure equation (parallel extension)
[Theta10,X] = solve(eq4,[theta10,x]);

% Subs solutions of first closure equation (parallel extension) for first leg
angle10 = real(double(subs(Theta10(1),{theta1,theta2},{Theta1,angle2})));
displacement = real(double(subs(X(1),{theta1,theta2},{Theta1,angle2})));

% Subs solutions of first closure equation (parallel extension) for second leg
angle10_ = real(double(subs(Theta10(1),{theta1,theta2},{Theta1_,angle2_})));
displacement_ = real(double(subs(X(2),{theta1,theta2},{Theta1_,angle2_})));

% Closure equation for second closed loop (parallel extension)
eq5 = T11_0p(1:3,end) == T11_0s(1:3,end);

% Solution of second closure equation (parallel extension)
[Theta9,Theta11] = solve(eq5,[theta9,theta11]);

% Subs solutions of second closure equation (parallel extension) for first leg
angle9 = real(double(subs(Theta9(2),{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta10},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle10})));
angle11 = real(double(subs(Theta11(2),{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta10},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle10})));

% Subs solutions of second closure equation (parallel extension) for second leg
angle9_ = real(double(subs(Theta9(2),{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta10},{Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle10_})));
angle11_ = real(double(subs(Theta11(2),{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta10},{Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle10_})));

% Closure equation for third closed loop (parallel extension)
eq6 = T13_0(1:3,end) == T13_0s(1:3,end);

% Solution of third closure equation (parallel extension)
[Theta12,Theta13] = solve(eq6,[theta12,theta13]);

% Subs solutions of third closure equation (parallel extension) for first leg
angle12 = real(double(subs(Theta12(1),{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11})));
angle13 = real(double(subs(Theta13(1),{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11})));

% Subs solutions of third closure equation (parallel extension) for second leg
angle12_ = real(double(subs(Theta12(1),{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11},{Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_})));
angle13_ = real(double(subs(Theta13(1),{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11},{Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_})));

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
Joint9 = T90(1:3,end);
Joint10 = T10_0p(1:3,end);
Joint11 = T11_0p(1:3,end);
Joint12 = T12_0(1:3,end);
Joint13 = T13_0(1:3,end);
FrontFoot = TFF_0(1:3,end);
BackFoot = TBF_0(1:3,end);

%Compute joint trajectory for the first leg
for i = 1:length(Theta1)
    J1(:,i) = double(subs(Joint1,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J2(:,i) = double(subs(Joint2,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J3(:,i) = double(subs(Joint3,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J4(:,i) = double(subs(Joint4,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J5(:,i) = double(subs(Joint5,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J6(:,i) = double(subs(Joint6,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J7(:,i) = double(subs(Joint7,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J8(:,i) = double(subs(Joint8,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J13(:,i) = double(subs(Joint13,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J9(:,i) = double(subs(Joint9,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J10(:,i) = double(subs(Joint10,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J11(:,i) = double(subs(Joint11,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    J12(:,i) = double(subs(Joint12,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    FF(:,i) = double(subs(FrontFoot,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
    BF(:,i) = double(subs(BackFoot,{theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta9 theta10 theta11 theta12 theta13},{Theta1(i) angle2(i) angle3(i) angle4(i) angle5(i) angle6(i) angle7(i) angle9(i) angle10(i) angle11(i) angle12(i) angle13(i)}));
end

%Define length of slide block
sliding = max(J10(1,:))-min(J10(1,:));


%% Load data of two leg mechanisms referred to base frame 0
% This section is useful in order to not run previous code line 
% It contains all data of single leg with parallel extension respect to base frame 0
load('TwoLegStabilizerData.mat');

%% Movie
figure('units','normalized','outerposition',[0 0 1 1])
view(2)
set(gca,'nextplot','replacechildren');
v = VideoWriter('SingleLegStabFoot.avi');
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
    title('Single leg mechanism with parallel extension');
    %Leg mechanism with parallel extension
    set(gca,'FontSize',14)
    %Vertices of hip 
    A = [min(J10(1,:))-0.03,max(J10(2,:))-0.03];
    B = [min(J10(1,:))-0.03,max(J10(2,:))+0.03];
    C = [0,max(J10(2,:))+0.03];
    D = [min(J0(1,:)),max(J0(2,:))];
    E = [min(J3(1,:)),max(J3(2,:))];
    F = [min(J8(1,:)),max(J8(2,:))];
    G = [min(J0(1,:)),max(J8(2,:))];
    Xvertices = [A(1) B(1) C(1) D(1) E(1) F(1) G(1)];
    Yvertices = [A(2) B(2) C(2) D(2) E(2) F(2) G(2)];
    h1 = patch(Xvertices,Yvertices,[1 0.83 0],'facealpha',0.7);
    h2 = plot([J0(1,1),J1(1,i)],[J0(2,1),J1(2,i)],'b','LineWidth',2);
    h3 = plot([J1(1,i),J2(1,i)],[J1(2,i),J2(2,i)],'b','LineWidth',2);
    h4 = plot([J2(1,i),J3(1,i)],[J2(2,i),J3(2,i)],'b','LineWidth',2);
    h5 = plot([J2(1,i),J4(1,i)],[J2(2,i),J4(2,i)],'b','LineWidth',2);
    h6 = plot([J4(1,i),J5(1,i)],[J4(2,i),J5(2,i)],'b','LineWidth',2);
    h7 = plot([J4(1,i),J9(1,i)],[J4(2,i),J9(2,i)],'r','LineWidth',2);
    h8 = plot([J4(1,i),J10(1,i)],[J4(2,i),J10(2,i)],'r','LineWidth',2);
    h9 = plot([J5(1,i),J6(1,i)],[J5(2,i),J6(2,i)],'b','LineWidth',2);
    h10 = plot([J5(1,i),J8(1,i)],[J5(2,i),J8(2,i)],'b','LineWidth',2);
    h11 = plot([J6(1,i),J7(1,i)],[J6(2,i),J7(2,i)],'b','LineWidth',2);
    h12 = plot([J7(1,i),J8(1,i)],[J7(2,i),J8(2,i)],'b','LineWidth',2);
    h13 = plot([J7(1,i),J13(1,i)],[J7(2,i),J13(2,i)],'b','LineWidth',2);
    h14 = plot([J9(1,i),J11(1,i)],[J9(2,i),J11(2,i)],'r','LineWidth',2);
    h15 = plot([J11(1,i),J6(1,i)],[J11(2,i),J6(2,i)],'r','LineWidth',2);
    h16 = plot([J11(1,i),J12(1,i)],[J11(2,i),J12(2,i)],'r','LineWidth',2);
    h17 = plot([J13(1,i),BF(1,i)],[J13(2,i),BF(2,i)],'b','LineWidth',2);
    h18 = plot([J13(1,i),FF(1,i)],[J13(2,i),FF(2,i)],'b','LineWidth',2);
    hold on
    h19 = plot(J0(1,:),J0(2,:),'blackv','MarkerSize',10);
    text(J0(1,1)+0.02,J0(2,1),'0');
    h20 = plot(J1(1,i),J1(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J1(1,i)-0.01,J1(2,i)+0.03,'1');
    h21 = plot(J2(1,i),J2(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J2(1,i)+0.02,J2(2,i),'2');
    h22 = plot(J3(1,i),J3(2,i),'blackv','MarkerSize',10);
    text(J3(1,1)+0.02,J3(2,1),'3');
    %h20 = plot(J4(1,:),J4(2,:),'color',[0, 0.5, 0],'LineWidth',1.5);
    h23 = plot(J4(1,i),J4(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J4(1,i)+0.02,J4(2,i)+0.02,'4');
    h24 = plot(J5(1,i),J5(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J5(1,i)+0.02,J5(2,i),'5');
    h25 = plot(J6(1,i),J6(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J6(1,i)+0.02,J6(2,i),'6');
    h26 = plot(J7(1,i),J7(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J7(1,i)+0.02,J7(2,i),'7');
    h27 = plot(J8(1,i),J8(2,i),'black^','MarkerSize',10);
    text(J8(1,i)+0.02,J8(2,i)+0.01,'8');
    %h26 = plot(J13(1,:),J13(2,:),'black','LineWidth',1.5);
    h28 = plot(J13(1,i),J13(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J13(1,i)-0.02,J13(2,i)-0.02,'13');
    h29 = plot(J9(1,i),J9(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J9(1,i)-0.02,J9(2,i)+0.04,'9');
    h30 = plot(J10(1,i),J10(2,i),'s','MarkerSize',15,'MarkerEdgeColor','black','MarkerFaceColor',[0.8 0.33 0]);
    h31 = plot(J10(1,i),J10(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J10(1,i)-0.01,J10(2,i)+0.04,'10');
    h32 = plot(J10(1,:),J10(2,:)+0.02,'black','LineWidth',1.5);
    h33 = plot(J10(1,:),J10(2,:)-0.02,'black','LineWidth',1.5);
    h34 = plot(J11(1,i),J11(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J11(1,i)-0.07,J11(2,i),'11');
    %h30 = plot(J12(1,:),J12(2,:),'black-.','LineWidth',1.5);
    h35 = plot(J12(1,i),J12(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J12(1,i)-0.03,J12(2,i)-0.03,'12');
    h36 = plot(FF(1,:),FF(2,:),'black:','LineWidth',1.5);
    text(FF(1,i),FF(2,i)+0.03,'FF');
    legend([h19 h20 h30 h32 h1 h2 h7 h36],{'Hinged Joint','Revolute Joint','Prismatic Joint','Slide block','Hip','Leg mechanism','Parallel Extension','Gait pattern of end effector'})
    hold off
    
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);

%% Denavit - Hartenberg Matrix

function dh = DH(link,a_i,d_i,theta_i)
    dh = [cos(theta_i) -sin(theta_i)*cos(a_i)   sin(theta_i)*sin(a_i)     link*cos(theta_i);
          sin(theta_i)  cos(theta_i)*cos(a_i)   -cos(theta_i)*sin(a_i)     link*sin(theta_i);
               0         sin(a_i)                   cos(a_i)                        d_i;
               0             0                          0                           1       ];
end
