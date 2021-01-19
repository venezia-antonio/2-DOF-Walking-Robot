clear all
close all
clc
%% Load data about Forward Kinematics of single leg
% This section allows to load all data of two legs referred to base frame 0
load('SingleLegData.mat');

%% Compute angle between world frame and base frame 
% The rotation angle between base frame 0 and frame attached to end
% effector is computed from rotation matrix
alfa = zeros(1,length(Theta1));
for i=1:length(Theta1)
    R = T13_0(1:3,1:3);
    R13_0 = double(subs(R,{theta1,theta2,theta3,theta4,theta5,theta6,theta7},{Theta1(1,i),angle2(1,i),angle3(1,i),angle4(1,i),angle5(1,i),angle6(1,i),angle7(1,i)}));
    alfa(i) = AngoloDaTraccia(R13_0);
end

%% Standing leg (without underscore)
syms ALFA

% Compute matrices for standing leg
T0W = DH(0,0,0,ALFA)*inv(T13_0);
T1W = T0W*T10;
T2W = T1W*T21;
T3W = T2W*T32;
T4W = T2W*T42;
T5W = T4W*T54;
T6W = T5W*T65;
T7W = T6W*T76;
T8W = T5W*T85;
T13W = T7W*T13_7;


% Compute joints' position vectors of standing leg
P0W = T0W(1:3,end);
P1W = T1W(1:3,end);
P2W = T2W(1:3,end);
P3W = T3W(1:3,end);
P4W = T4W(1:3,end);
P5W = T5W(1:3,end);
P6W = T6W(1:3,end);
P7W = T7W(1:3,end);
P8W = T8W(1:3,end);
P13W = T13W(1:3,end);


% Subsitution of joint variables
J0 = double(subs(P0W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,alfa}));
J1 = double(subs(P1W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,alfa}));
J2 = double(subs(P2W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,alfa}));
J3 = double(subs(P3W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,alfa}));
J4 = double(subs(P4W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,alfa}));
J5 = double(subs(P5W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,alfa}));
J6 = double(subs(P6W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,alfa}));
J7 = double(subs(P7W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,alfa}));
J8 = double(subs(P8W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,alfa}));
J13 = double(subs(P13W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,ALFA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,alfa}));
JW = [0 0 0]';

%% Swing leg (with underscore)
% Transformation matrices for swing leg
syms theta1_ theta2_ theta3_ theta4_ theta5_ theta6_ theta7_

T10_ =  DH(L1,0,0,theta1_);
T21_ = DH(L2/2,0,0,theta2_);
T32_ = DH(L3,0,0,theta3_);
T42_ = DH(L2/2,0,0,0);
T54_ = DH((1/3)*L4,0,0,theta4_);
T85_ = DH(L6,0,0,theta5_);
T65_ = DH((2/3)*L4,0,0,0);
T76_ = DH((1/3)*L5,0,0,theta6_);
T87_ = DH(L7,0,0,theta7_);
T13_7_ = DH((2/3)*L5,0,0,0);

% Compute matrices for swing leg referred to world frame
T1W_ = T0W*T10_;
T2W_ = T1W_*T21_;
T3W_ = T2W_*T32_;
T4W_ = T2W_*T42_;
T5W_ = T4W_*T54_;
T6W_ = T5W_*T65_;
T7W_ = T6W_*T76_;
T8W_ = T5W_*T85_;
T13W_ = T7W_*T13_7_;


%Compute joints' position vectors of swing leg
P0W_ = T0W(1:3,end);
P1W_ = T1W_(1:3,end);
P2W_ = T2W_(1:3,end);
P3W_ = T3W_(1:3,end);
P4W_ = T4W_(1:3,end);
P5W_ = T5W_(1:3,end);
P6W_ = T6W_(1:3,end);
P7W_ = T7W_(1:3,end);
P8W_ = T8W_(1:3,end);
P13W_ = T13W_(1:3,end);


% Subsitution of joint variables
J0_ = double(subs(P0W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,ALFA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,alfa}));

J1_ = double(subs(P1W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,ALFA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,alfa}));
    
J2_ = double(subs(P2W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,ALFA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,alfa}));
    
J3_ = double(subs(P3W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,ALFA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,alfa}));
    
J4_ = double(subs(P4W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,ALFA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,alfa}));
    
J5_ = double(subs(P5W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,ALFA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,alfa}));
    
J6_ = double(subs(P6W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,ALFA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,alfa}));
    
J7_ = double(subs(P7W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,ALFA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,alfa}));
    
J8_ = double(subs(P8W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,ALFA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,alfa}));
    
J13_ = double(subs(P13W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,ALFA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,alfa}));

%% Compute geometric centroid of the hip
for i=1:length(Theta1)
    com(1,i) = (J3(1,i)+J8(1,i)+J0(1,i))/3;
    com(2,i) = (J3(2,i)+J8(2,i)+J0(2,i))/3;
end
    
%% Load standing and swing leg data
%This section is useful in order to not run the previous code
load('BodyPostureNoStabNoFoot.mat');

%% Movie
figure('units','normalized','outerposition',[0 0 1 1])
view(2)
set(gca,'nextplot','replacechildren');
v = VideoWriter('BodyPostureNoStabNoFoot.avi');
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
    title('Two-Legged Walking Robot with punctiform foot');
    % Vertices of hip 
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
    h1 = patch(Xvertices,Yvertices,[1 0.83 0],'facealpha',0.7);
    % Standing leg
    set(gca,'FontSize',14)
    h2 = plot([J0(1,i),J1(1,i)],[J0(2,i),J1(2,i)],'b','LineWidth',2);
    h3 = plot([J1(1,i),J2(1,i)],[J1(2,i),J2(2,i)],'b','LineWidth',2);
    h4 = plot([J2(1,i),J3(1,i)],[J2(2,i),J3(2,i)],'b','LineWidth',2);
    h5 = plot([J2(1,i),J4(1,i)],[J2(2,i),J4(2,i)],'b','LineWidth',2);
    h6 = plot([J4(1,i),J5(1,i)],[J4(2,i),J5(2,i)],'b','LineWidth',2);
    h7 = plot([J5(1,i),J6(1,i)],[J5(2,i),J6(2,i)],'b','LineWidth',2);
    h8 = plot([J5(1,i),J8(1,i)],[J5(2,i),J8(2,i)],'b','LineWidth',2);
    h9 = plot([J6(1,i),J7(1,i)],[J6(2,i),J7(2,i)],'b','LineWidth',2);
    h10 = plot([J7(1,i),J8(1,i)],[J7(2,i),J8(2,i)],'b','LineWidth',2);
    h11 = plot([J7(1,i),J13(1,i)],[J7(2,i),J13(2,i)],'b','LineWidth',2);
    hold on
    h14 = plot(J0(1,i),J0(2,i),'blackv','MarkerSize',10);
    text(J0(1,i)-0.06,J0(2,i),'0');
    h15 = plot(J1(1,i),J1(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J1(1,i),J1(2,i)+0.03,'1');
    h16 = plot(J2(1,i),J2(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J2(1,i)+0.02,J2(2,i),'2');
    h17 = plot(J3(1,i),J3(2,i),'blackv','MarkerSize',10);
    text(J3(1,i)+0.02,J3(2,i),'3');
    h19 = plot(J4(1,i),J4(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J4(1,i)+0.01,J4(2,i)+2,'4');
    h20 = plot(J5(1,i),J5(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J5(1,i)+0.02,J5(2,i),'5');
    h21 = plot(J6(1,i),J6(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J6(1,i)+0.02,J6(2,i),'6');
    h22 = plot(J7(1,i),J7(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J7(1,i)+0.02,J7(2,i),'7');
    h23 = plot(J8(1,i),J8(2,i),'black^','MarkerSize',10);
    text(J8(1,i)-0.06,J8(2,i)+0.03,'8');
    h24 = plot(J13(1,i),J13(2,i));
    text(J13(1,i)-0.01,J13(2,i)-0.02,'13');       
    % Swing leg
    set(gca,'FontSize',14)
    h26 = plot([J0_(1,i),J1_(1,i)],[J0_(2,i),J1_(2,i)],'color','r','LineWidth',2);
    h27 = plot([J1_(1,i),J2_(1,i)],[J1_(2,i),J2_(2,i)],'color','r','LineWidth',2);
    h28 = plot([J2_(1,i),J3_(1,i)],[J2_(2,i),J3_(2,i)],'color','r','LineWidth',2);
    h29 = plot([J2_(1,i),J4_(1,i)],[J2_(2,i),J4_(2,i)],'color','r','LineWidth',2);
    h30 = plot([J4_(1,i),J5_(1,i)],[J4_(2,i),J5_(2,i)],'color','r','LineWidth',2);
    h31 = plot([J5_(1,i),J6_(1,i)],[J5_(2,i),J6_(2,i)],'color','r','LineWidth',2);
    h32 = plot([J5_(1,i),J8_(1,i)],[J5_(2,i),J8_(2,i)],'color','r','LineWidth',2);
    h33 = plot([J6_(1,i),J7_(1,i)],[J6_(2,i),J7_(2,i)],'color','r','LineWidth',2);
    h34 = plot([J7_(1,i),J8_(1,i)],[J7_(2,i),J8_(2,i)],'color','r','LineWidth',2);
    h35 = plot([J7_(1,i),J13_(1,i)],[J7_(2,i),J13_(2,i)],'color','r','LineWidth',2);
    hold on
    h38 = plot(J0_(1,i),J0_(2,i),'blackv','MarkerSize',10);
    text(J0_(1,i)-0.06,J0_(2,i),'0''');
    h39 = plot(J1_(1,i),J1_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J1_(1,i),J1_(2,i)+0.03,'1''');
    h40 = plot(J2_(1,i),J2_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J2_(1,i)+0.02,J2_(2,i)+0.01,'2''');
    h41 = plot(J3_(1,i),J3_(2,i),'blackv','MarkerSize',10);
    text(J3_(1,i)+0.02,J3_(2,i),'3''')
    h43 = plot(J4_(1,i),J4_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J4_(1,i),J4_(2,i)+0.03,'4''');
    h44 = plot(J5_(1,i),J5_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J5_(1,i)+0.02,J5_(2,i),'5''');
    h45 = plot(J6_(1,i),J6_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J6_(1,i)+0.02,J6_(2,i),'6''');
    h46 = plot(J7_(1,i),J7_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J7_(1,i)+0.02,J7_(2,i),'7''');
    h47 = plot(J8_(1,i),J8_(2,i),'black^','MarkerSize',10);
    text(J8_(1,i)-0.06,J8_(2,i)+0.03,'8''');
    h48 = plot(J13_(1,i),J13_(2,i));
    text(J13_(1,i)-0.02,J13_(2,i)-0.03,'13''');
    h49 = plot(J13_(1,17:49),J13_(2,17:49),'black:','LineWidth',1.5);
    h50 = plot([0 0.1],[0 0],'m','Linewidth',1.5);
    h51 = plot([0 0],[0 0.1],'m','Linewidth',1.5);
    h52 = plot(0.1,0,'m>','Linewidth',1.5);
    h53 = plot(0,0.1,'m^','Linewidth',1.5);
    text(0.1,-0.02,'x','Color','m');
    text(0.02,0.12,'y','Color','m');
    h54 = plot(com(1,i),com(2,i),'o','Color',[0 0.5 0],'Linewidth',1.5);
    h55 = plot(com(1,17:49),com(2,17:49),'Color',[0 0.5 0],'Linewidth',1.5);
    hold off  
    legend([h38 h39 h1 h54 h2 h26 h55 h49 h50],{'Hinged Joint','Revolute Joint','Hip','COM of hip','Standing leg','Swing leg','Hip trajectory','Gait pattern swing foot','World frame'})
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
