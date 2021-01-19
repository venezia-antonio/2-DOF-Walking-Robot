clear all
close all
clc
%% Load data from SingleLegStabilizerData.mat
% The file .mat contains all data of two legs referred to base frame
% In this case it will be considered both parallel extensions and feet
load('TwoLegStabilizerData.mat');

%% Compute angle between world frame and base frame
% The rotation angle between base frame 0 and frame attached to end
% effector (in this case the end effector is toe foot) is computed from the
% rotation matrix

alfa = zeros(1,length(Theta1));
for i=1:length(Theta1)
    R = TFF_0(1:3,1:3);
    RFF_0 = double(subs(R,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13},{Theta1(1,i),angle2(1,i),angle3(1,i),angle4(1,i),angle5(1,i),angle6(1,i),angle7(1,i),angle9(1,i),angle10(1,i),angle11(1,i),angle12(1,i),angle13(1,i)}));
    alfa(i) = AngoloDaTraccia(RFF_0);
end
%% Standing leg (without underscore)
syms ALFA

% Compute matrices for standing leg
T0W = DH(0,0,0,ALFA)*inv(TFF_0);
T1W = T0W*T10;
T2W = T1W*T21;
T3W = T2W*T32;
T4W = T2W*T42;
T5W = T4W*T54;
T6W = T5W*T65;
T7W = T6W*T76;
T8W = T5W*T85;
T9W = T4W*T94;
T10W = T4W*T10_4;
T11W = T9W*T11_9;
T12W = T6W*T11_6*T12_11;
T13W = T0W*T13_0;
TBFW = T0W*TBF_0;

% Compute position vectors of standing leg's joint
P0W = T0W(1:3,end);
P1W = T1W(1:3,end);
P2W = T2W(1:3,end);
P3W = T3W(1:3,end);
P4W = T4W(1:3,end);
P5W = T5W(1:3,end);
P6W = T6W(1:3,end);
P7W = T7W(1:3,end);
P8W = T8W(1:3,end);
P9W = T9W(1:3,end);
P10W = T10W(1:3,end);
P11W = T11W(1:3,end);
P12W = T12W(1:3,end);
P13W = T13W(1:3,end);
PBFW = TBFW(1:3,end);

% Subsitution of joint variables
J0 = double(subs(P0W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J1 = double(subs(P1W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J2 = double(subs(P2W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J3 = double(subs(P3W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J4 = double(subs(P4W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J5 = double(subs(P5W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J6 = double(subs(P6W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J7 = double(subs(P7W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J8 = double(subs(P8W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J9 = double(subs(P9W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J10 = double(subs(P10W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J11 = double(subs(P11W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J12 = double(subs(P12W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
J13 = double(subs(P13W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
BFW = double(subs(PBFW,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,alfa}));
JW = [0 0 0]';

%% Swing leg (with underscore)
syms theta1_ theta2_ theta3_ theta4_ theta5_ theta6_ theta7_ theta9_ theta10_ theta11_ theta12_ theta13_  

%Transformation matrices for swing leg 
T10_ =  DH(L1,0,0,theta1_);
T21_ = DH(L2/2,0,0,theta2_);
T32_ = DH(L3,0,0,theta3_);
T42_ = DH(L2/2,0,0,0);
T54_ = DH((1/3)*L4,0,0,theta4_);
T85_ = DH(L6,0,0,theta5_);
T65_ = DH((2/3)*L4,0,0,0);
T76_ = DH((1/3)*L5,0,0,theta6_);
T13_7_ = DH((2/3)*L5,0,0,0);
T10_4_ = DH(L8,0,0,theta10_);
T94_ = DH(L8/2,0,0,theta10_);
T11_9_ = DH(L9,0,0,theta9_);
T11_6_ = DH(L10,0,0,theta11_);
T12_11_ = DH(L11,0,0,theta12_);
T13_12_ = DH(L10,0,0,theta13_);
TFF_13_ = DH(frontFootLength,0,0,theta13_);
TBF_FF_ = DH(backFootLength+frontFootLength+L10,0,0,sym(pi));


% Compute matrices for swing leg
T1W_ = T0W*T10_;
T2W_ = T1W_*T21_;
T3W_ = T2W_*T32_;
T4W_ = T2W_*T42_;
T5W_ = T4W_*T54_;
T6W_ = T5W_*T65_;
T7W_ = T6W_*T76_;
T8W_ = T5W_*T85_;
T9W_ = T4W_*T94_;
T10W_ = T4W_*T10_4_;
T11W_ = T9W_*T11_9_;
T12W_ = T6W_*T11_6_*T12_11_;
T13W_ = T7W_*T13_7_;
TFFW_ = T13W_*TFF_13_;
TBFW_ = TFFW_*TBF_FF_;


% Compute position vectors of swing leg's joint
P0W_ = T0W(1:3,end);
P1W_ = T1W_(1:3,end);
P2W_ = T2W_(1:3,end);
P3W_ = T3W_(1:3,end);
P4W_ = T4W_(1:3,end);
P5W_ = T5W_(1:3,end);
P6W_ = T6W_(1:3,end);
P7W_ = T7W_(1:3,end);
P8W_ = T8W_(1:3,end);
P9W_ = T9W_(1:3,end);
P10W_ = T10W_(1:3,end);
P11W_ = T11W_(1:3,end);
P12W_ = T12W_(1:3,end);
P13W_ = T13W_(1:3,end);
PBFW_ = TBFW_(1:3,end);
PFFW_ = TFFW_(1:3,end);

% Subsitution of joint variables
J0_ = double(subs(P0W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J1_ = double(subs(P1W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J2_ = double(subs(P2W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J3_ = double(subs(P3W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J4_ = double(subs(P4W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J5_ = double(subs(P5W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J6_ = double(subs(P6W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J7_ = double(subs(P7W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J8_ = double(subs(P8W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J9_ = double(subs(P9W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J10_ = double(subs(P10W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J11_ = double(subs(P11W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J12_ = double(subs(P12W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

J13_ = double(subs(P13W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

BFW_ = double(subs(PBFW_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

FFW_ = double(subs(PFFW_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta9,theta10,theta11,theta12,theta13,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,theta9_,theta10_,theta11_,theta12_,theta13_,ALFA},...
    {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,angle9,angle10,angle11,angle12,angle13,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,angle9_,angle10_,angle11_,angle12_,angle13_,alfa}));

%% Compute geometric centroid

for i=1:length(Theta1)
    com(1,i) = (J3(1,i)+J8(1,i)+J0(1,i))/3;
    com(2,i) = (J3(2,i)+J8(2,i)+J0(2,i))/3;
end

%% Load swing and standing leg data
%This section is useful in order to not run the previous code
load('BodyPostureStabFoot.mat')

%% Movie
figure('units','normalized','outerposition',[0 0 1 1])
view(2)
set(gca,'nextplot','replacechildren');
v = VideoWriter('BodyPostureStabFoot.avi');
open(v);
hold on
grid on
for i = 17:49
    clf
    hold on
    grid on
    hold on
    grid on
    axis equal
    xlim([-0.7 1.2]);ylim([-0.1 1.5]);xlabel('X [m]');ylabel('Y [m]')
    title('Two-Legged Walking Robot with parallel extensions');
    set(gca,'FontSize',14)
   
    %J4 figure(standing leg)
    %h1 = plot(J4(1,17:49),J4(2,17:49),'color',[0, 0.5, 0],'LineWidth',1.5);
    % Standing leg
    h2 = plot([J0(1,i),J1(1,i)],[J0(2,i),J1(2,i)],'b','LineWidth',2);
    h3 = plot([J1(1,i),J2(1,i)],[J1(2,i),J2(2,i)],'b','LineWidth',2);
    h4 = plot([J2(1,i),J3(1,i)],[J2(2,i),J3(2,i)],'b','LineWidth',2);
    h5 = plot([J2(1,i),J4(1,i)],[J2(2,i),J4(2,i)],'b','LineWidth',2);
    h6 = plot([J4(1,i),J5(1,i)],[J4(2,i),J5(2,i)],'b','LineWidth',2);
    h7 = plot([J4(1,i),J9(1,i)],[J4(2,i),J9(2,i)],'b','LineWidth',2);
    h8 = plot([J4(1,i),J10(1,i)],[J4(2,i),J10(2,i)],'b','LineWidth',2);
    h9 = plot([J5(1,i),J6(1,i)],[J5(2,i),J6(2,i)],'b','LineWidth',2);
    h10 = plot([J5(1,i),J8(1,i)],[J5(2,i),J8(2,i)],'b','LineWidth',2);
    h11 = plot([J6(1,i),J7(1,i)],[J6(2,i),J7(2,i)],'b','LineWidth',2);
    h12 = plot([J7(1,i),J8(1,i)],[J7(2,i),J8(2,i)],'b','LineWidth',2);
    h13 = plot([J7(1,i),J13(1,i)],[J7(2,i),J13(2,i)],'b','LineWidth',2);
    h14 = plot([J9(1,i),J11(1,i)],[J9(2,i),J11(2,i)],'b','LineWidth',2);
    h15 = plot([J11(1,i),J6(1,i)],[J11(2,i),J6(2,i)],'b','LineWidth',2);
    h16 = plot([J11(1,i),J12(1,i)],[J11(2,i),J12(2,i)],'b','LineWidth',2);
    h17 = plot([J13(1,i),BFW(1,i)],[J13(2,i),BFW(2,i)],'b','LineWidth',2);
    h18 = plot([J13(1,i),JW(1,1)],[J13(2,i),JW(2,1)],'b','LineWidth',2);
    hold on
    h19 = plot(J0(1,i),J0(2,i),'blackv','MarkerSize',10);
    text(J0(1,i)-0.06,J0(2,i),'0');
    h20 = plot(J1(1,i),J1(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J1(1,i)-0.02,J1(2,i)+0.03,'1');
    h21 = plot(J2(1,i),J2(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J2(1,i)+0.02,J2(2,i),'2');
    h22 = plot(J3(1,i),J3(2,i),'blackv','MarkerSize',10);
    text(J3(1,i)+0.02,J3(2,i),'3')
    h23 = plot(J4(1,i),J4(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J4(1,i)+0.01,J4(2,i)+0.01,'4');
    h24 = plot(J5(1,i),J5(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J5(1,i)+0.02,J5(2,i)+0.01,'5');
    h25 = plot(J6(1,i),J6(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J6(1,i)+0.02,J6(2,i),'6');
    h26 = plot(J7(1,i),J7(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J7(1,i)+0.02,J7(2,i),'7');
    h27 = plot(J8(1,i),J8(2,i),'black^','MarkerSize',10);
    text(J8(1,i)-0.06,J8(2,i)+0.03,'8');
    h28 = plot(J13(1,i),J13(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J13(1,i)-0.03,J13(2,i)-0.03,'13');
    h29 = plot(J9(1,i),J9(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J9(1,i)-0.02,J9(2,i)+0.03,'9');
    h30 = plot(J10(1,i),J10(2,i),'s','MarkerSize',15,'MarkerEdgeColor','black','MarkerFaceColor',[0.8 0.33 0]);
    h31 = plot(J10(1,i),J10(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J10(1,i)-0.02,J10(2,i)+0.04,'10');
    h32 = plot([J0(1,i)-sliding J0(1,i)],[J0(2,i)-L12+0.02 J10(2,i)+0.02],'black','LineWidth',1.5);
    h33 = plot([J0(1,i)-sliding J0(1,i)],[J0(2,i)-L12-0.02 J10(2,i)-0.02],'black','LineWidth',1.5);
    h34 = plot(J11(1,i),J11(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J11(1,i),J11(2,i)+0.02,'11');
    h35 = plot(J12(1,i),J12(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J12(1,i)-0.03,J12(2,i)-0.03,'12');
    
    %Vertices of hip
    A = [J0(1,i),J0(2,i)];
    B = [A(1)+L0,A(2)];
    C = [A(1)+L0,A(2)-L13];
    D = [A(1),A(2)-L13];
    E = [A(1)-0.2575,A(2)-L12-0.03];
    F = [A(1)-0.2575,A(2)-L12+0.03];
    G = [A(1),A(2)-L12+0.03];
    Xvertices = [A(1) B(1) C(1) D(1) E(1) F(1) G(1)];
    Yvertices = [A(2) B(2) C(2) D(2) E(2) F(2) G(2)];
    % Hip 
    h36 = patch(Xvertices,Yvertices,[1 0.83 0],'facealpha',0.7);
    % Slide block
    h37 = plot([J0_(1,i)-sliding J0_(1,i)],[J0_(2,i)-L12+0.02 J10_(2,i)+0.02],'black','LineWidth',1.5);
    h38 = plot([J0_(1,i)-sliding J0_(1,i)],[J0_(2,i)-L12-0.02 J10_(2,i)-0.02],'black','LineWidth',1.5);
    %J4_ figure(swing leg)
    %h39 = plot(J4_(1,17:49),J4_(2,17:49),'green','LineWidth',1.5);
    
    %Swing Leg
    h40 = plot([J0_(1,i),J1_(1,i)],[J0_(2,i),J1_(2,i)],'r','LineWidth',2);
    h41 = plot([J1_(1,i),J2_(1,i)],[J1_(2,i),J2_(2,i)],'r','LineWidth',2);
    h42 = plot([J2_(1,i),J3_(1,i)],[J2_(2,i),J3_(2,i)],'r','LineWidth',2);
    h43 = plot([J2_(1,i),J4_(1,i)],[J2_(2,i),J4_(2,i)],'r','LineWidth',2);
    h44 = plot([J4_(1,i),J5_(1,i)],[J4_(2,i),J5_(2,i)],'r','LineWidth',2);
    h45 = plot([J4_(1,i),J9_(1,i)],[J4_(2,i),J9_(2,i)],'r','LineWidth',2);
    h46 = plot([J4_(1,i),J10_(1,i)],[J4_(2,i),J10_(2,i)],'r','LineWidth',2);
    h47 = plot([J5_(1,i),J6_(1,i)],[J5_(2,i),J6_(2,i)],'r','LineWidth',2);
    h48 = plot([J5_(1,i),J8_(1,i)],[J5_(2,i),J8_(2,i)],'r','LineWidth',2);
    h49 = plot([J6_(1,i),J7_(1,i)],[J6_(2,i),J7_(2,i)],'r','LineWidth',2);
    h50 = plot([J7_(1,i),J8_(1,i)],[J7_(2,i),J8_(2,i)],'r','LineWidth',2);
    h51 = plot([J7_(1,i),J13_(1,i)],[J7_(2,i),J13_(2,i)],'r','LineWidth',2);
    h52 = plot([J9_(1,i),J11_(1,i)],[J9_(2,i),J11_(2,i)],'r','LineWidth',2);
    h53 = plot([J11_(1,i),J6_(1,i)],[J11_(2,i),J6_(2,i)],'r','LineWidth',2);
    h54 = plot([J11_(1,i),J12_(1,i)],[J11_(2,i),J12_(2,i)],'r','LineWidth',2);
    h55 = plot([J13_(1,i),BFW_(1,i)],[J13_(2,i),BFW_(2,i)],'r','LineWidth',2);
    h56 = plot([J13_(1,i),FFW_(1,i)],[J13_(2,i),FFW_(2,i)],'r','LineWidth',2);
    hold on
    
    h57 = plot(J0_(1,i),J0_(2,i),'blackv','MarkerSize',10);
    text(J0_(1,i)-0.06,J0_(2,i),'0''');
    h58 = plot(J1_(1,i),J1_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J1_(1,i)-0.02,J1_(2,i)+0.03,'1''');
    h59 = plot(J2_(1,i),J2_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J2_(1,i)+0.02,J2_(2,i),'2''');
    h60 = plot(J3_(1,i),J3_(2,i),'blackv','MarkerSize',10);
    text(J3_(1,i)+0.02,J3_(2,i),'3''')
    h61 = plot(J4_(1,i),J4_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J4_(1,i)+0.01,J4_(2,i)+0.01,'4''');
    h62 = plot(J5_(1,i),J5_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J5_(1,i)+0.02,J5_(2,i)+0.01,'5''');
    h63 = plot(J6_(1,i),J6_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J6_(1,i)+0.02,J6_(2,i),'6''');
    h64 = plot(J7_(1,i),J7_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J7_(1,i)+0.02,J7_(2,i),'7''');
    h65 = plot(J8_(1,i),J8_(2,i),'black^','MarkerSize',10);
    text(J8_(1,i)-0.06,J8_(2,i)+0.03,'8''');
    h66 = plot(J13_(1,i),J13_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J13_(1,i)-0.03,J13_(2,i)-0.03,'13''');
    h67 = plot(J9_(1,i),J9_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J9_(1,i)-0.02,J9_(2,i)+0.04,'9''');
    h68 = plot(J10_(1,i),J10_(2,i),'s','MarkerSize',15,'MarkerEdgeColor','black','MarkerFaceColor',[0.8 0.33 0]);
    h69 = plot(J10_(1,i),J10_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J10_(1,i)-0.02,J10_(2,i)+0.04,'10''');
    h70 = plot(J11_(1,i),J11_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J11_(1,i),J11_(2,i)+0.02,'11''');
    h71 = plot(J12_(1,i),J12_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J12_(1,i)-0.03,J12_(2,i)-0.03,'12''');
    h72 = plot(FFW_(1,17:49),FFW_(2,17:49),'black:','LineWidth',1.5);
    text(JW(1,1),JW(2,1)-0.02,'FF');
    text(FFW_(1,i),FFW_(2,i)-0.02,'FF''');
    h73 = plot(com(1,i),com(2,i),'o','Color',[0 0.5 0],'Linewidth',1.5);
    h74 = plot(com(1,17:49),com(2,17:49),'Color',[0 0.5 0],'Linewidth',1.5);
    hold off
    legend([h57 h58 h68 h37 h36 h73 h3 h40 h74 h72],{'Hinged Joint','Revolute Joint','Prismatic Joint','Slide block','Hip','COM of hip','Standing leg','Swing leg','Hip trajectory','Gait pattern of swing foot'})
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

%% Compute angle from rotation matrix

function [alfa]=AngoloDaTraccia(Mat)
          Asse =[0,0,1]';
          syms theta
          alfa=acos((trace(Mat)-1)/2);
          eq = Asse == 1/(2*sin(theta))*[Mat(3,2)-Mat(2,3);Mat(1,3)-Mat(3,1);Mat(2,1)-Mat(1,2)];
          Theta = solve(eq,theta);
          if sin(Theta)<0
              alfa = 2*pi-alfa;
          end
end
