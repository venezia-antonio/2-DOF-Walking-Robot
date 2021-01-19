%% Quasi-Static Analysis 
clear all
close all
clc
%% Input Parameter (Change k in order to vary end-effector trajectory)
% Conversion factor from centimetres to metres
conv = 0.01;
% Incremental step 
h = 0.1;
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

%% Denavit Hartenberg - Forward Kinematic First Leg
syms t theta1 theta2 theta3 theta4 theta5 theta6 theta7 GAMMA

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
% Transformation matrix FrontFoot to Joint 13
TFF_13 = DH(frontFootLength,0,0,GAMMA);
% Front Foot
TFF0 = T13_0*TFF_13;
% Transformation matrix BackFoot to Joint 13
TBF_13 = DH(backFootLength,0,0,pi+GAMMA);
% Back Foot
TBF0 = T13_0*TBF_13;

%% Denavit Hartenberg - Forward Kinematic Second Leg
syms t theta1_ theta2_ theta3_ theta4_ theta5_ theta6_ theta7_

%% First closed chain
% Transformation matrix Joint 1 to Joint 0 
T10_ =  DH(L1,0,0,theta1_);
% Transformation matrix Joint 2 to Joint 1
T21_ = DH(L2/2,0,0,theta2_);
% Joint 2
T20_ = T10_*T21_;
% Transformation matrix Joint 3 to Joint 2
T32_ = DH(L3,0,0,theta3_);
% Joint 3 (first branch)
T30_1p_ = T10_*T21_*T32_;
% Transformation matrix Joint 3 to Joint 0
T30_1s_ = DH(L0,0,0,0);

%% Second closed chain
% Transformation matrix Joint 4 to Joint 2
T42_ = DH(L2/2,0,0,0);
% Joint 4
T40_ = T10_*T21_*T42_;
% Transformation matrix Joint 5 to Joint 4
T54_ = DH((1/3)*L4,0,0,theta4_);
% Joint 5
T50_ = T40_*T54_;
% Transformation matrix Joint 8 to Joint 5
T85_ = DH(L6,0,0,theta5_);
% Joint 8 (first branch)
T80_2p_ = T50_*T85_;
% Transfromation matrix Joint 3 to Joint 8
T83_ = DH(L13,0,0,3*(pi/2));
% Joint 8 (second branch)
T80_2s_ = T30_1s_*T83_;

%% Third closed chain
% Transformation matrix Joint 6 to Joint 5
T65_ = DH((2/3)*L4,0,0,0);
% Joint 6
T60_ = T50_*T65_;
% Transformation matrix Joint 7 to Joint 6
T76_ = DH((1/3)*L5,0,0,theta6_);
% Joint 7
T70_ = T60_*T76_;
% Joint 8 (first branch)
T80_3p_ = T80_2p_;
% Transformation matrix Joint 7 to Joint 8
T87_ = DH(L7,0,0,theta7_);
% Joint 8 (second branch)
T80_3s_ = T70_*T87_;
% Transformation matrix Joint 13 to Joint 7
T13_7_ = DH((2/3)*L5,0,0,0);
% Joint 13
T13_0_ = T70_*T13_7_;
% Transformation matrix FrontFoot to Joint 13
TFF_13_ = DH(frontFootLength,0,0,pi/2);
% Front Foot
TFF0_ = T13_0_*TFF_13_;
% Transformation matrix BackFoot to Joint 13
TBF_13_ = DH(backFootLength,0,0,3*(pi/2));
% Back Foot
TBF0_ = T13_0_*TBF_13_;


%% First Leg
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

% Closure equation of third closed loop (pantograph)
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


%% Second leg
% Define options of fminsearch method
options = optimset('TolFun',1e-14,'TolX',1e-14,'UseParallel',true);

% Closure equation for first closed loop (Hoeken mechanism)
eq1_ = T30_1p_(1:3,end) == T30_1s_(1:3,end);
eq1_ = lhs(eq1_)-rhs(eq1_);

% Solution of first closure equation using numerical solver
for i = 1:length(Theta1_)
    
    tmp_ = subs(eq1_,theta1_,Theta1_(i));
    eq_ = tmp_(1)^2 + tmp_(2)^2;
    eq1H_ = matlabFunction(eq_);
    eq1Mod_ = @(x)eq1H_(x(1),x(2));
    if i==1
        x0 = [3*pi/2,pi/2];
        x = fminsearch(eq1Mod_,x0,options);  % x = [theta2 theta3]
        angle2_(i) = x(1);
        angle3_(i) = x(2);
    else
        x0 = [angle2_(i-1),angle3_(i-1)];
        x = fminsearch(eq1Mod_,x0,options);  % x = [theta2 theta3]
        angle2_(i) = x(1);
        angle3_(i) = x(2);
    end
end

% Closure equation for second closed loop (pantograph)
eq2_ = T80_2p_(1:3,end) == T80_2s_(1:3,end);
eq2_ = lhs(eq2_)-rhs(eq2_);

% Solution of second closed loop using numerical solver
for i = 1:length(Theta1_)
    
    tmp_ = subs(eq2_,{theta1_,theta2_,theta3_},{Theta1_(i) angle2_(i) angle3_(i)});
    eq_ = tmp_(1)^2 + tmp_(2)^2;  
    eq1H_ = matlabFunction(eq_);
    eq1Mod_ = @(x)eq1H_(x(1),x(2));
    if i==1
        x0 = [pi/4,3*pi/2];
        x = fminsearch(eq1Mod_,x0);  % x = [theta4 theta5]
        angle4_(i) = x(1);
        angle5_(i) = x(2);
    else
        x0 = [angle4_(i-1),angle5_(i-1)];
        x = fminsearch(eq1Mod_,x0,options);  % x = [theta4 theta5]
        angle4_(i) = x(1);
        angle5_(i) = x(2);
    end
end

% Closure equation for third closed loop (pantograph)
eq3_ = T80_3p_(1:3,end) == T80_3s_(1:3,end);
eq3_ = lhs(eq3_)-rhs(eq3_);

% Solution of third closed loop using numerical solver
for i = 1:length(Theta1_)
    
    tmp_ = subs(eq3_,{theta1_,theta2_,theta3_,theta4_,theta5_},{Theta1_(i),angle2_(i),angle3_(i),angle4_(i),angle5_(i)});
    eq_ = tmp_(1)^2 + tmp_(2)^2;  
    eq1H_ = matlabFunction(eq_);
    eq1Mod_ = @(x)eq1H_(x(1),x(2));
    if i==1
        x0 = [3*pi/2,3*pi/2];
        x = fminsearch(eq1Mod_,x0);  % x = [theta6 theta7]
        angle6_(i) = x(1);
        angle7_(i) = x(2);
    else
        x0 = [angle6_(i-1),angle7_(i-1)];
        x = fminsearch(eq1Mod_,x0,options);  % x = [theta6 theta7]
        angle6_(i) = x(1);
        angle7_(i) = x(2);
    end
end


%% Compute angle between world frame and base frame for every point in the straight-line phase
% The rotation angle between base frame 0 and frame attached to end
% effector (in this case the end effector is toe foot) is computed by
% rotation matrix

alfa = zeros(1,length(Theta1));
for i=1:length(Theta1)
    R = T13_0(1:3,1:3);
    R13_0 = double(subs(R,{theta1,theta2,theta3,theta4,theta5,theta6,theta7},{Theta1(1,i),angle2(1,i),angle3(1,i),angle4(1,i),angle5(1,i),angle6(1,i),angle7(1,i)}));
    alfa(i) = AngoloDaTraccia(R13_0);
end

gamma = 2*pi - alfa;
beta = alfa-pi;

%% Standing leg (without underscore)

%Compute matrices for standing leg
R13_0 = T13_0(1:3,1:3);
T0W = [R13_0 zeros(3,1);0 0 0 1]*inv(T13_0);
T1W = T0W*T10;
T2W = T1W*T21;
T3W = T2W*T32;
T4W = T2W*T42;
T5W = T4W*T54;
T6W = T5W*T65;
T7W = T6W*T76;
T8W = T5W*T85;
T13W = T7W*T13_7;
TFFW = T13W*TFF_13;
TBFW = T13W*TBF_13;

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
PFFW = TFFW(1:3,end);
PBFW = TBFW(1:3,end);

% Subsitution of angles
J0 = double(subs(P0W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
J1 = double(subs(P1W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
J2 = double(subs(P2W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
J3 = double(subs(P3W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
J4 = double(subs(P4W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
J5 = double(subs(P5W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
J6 = double(subs(P6W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
J7 = double(subs(P7W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
J8 = double(subs(P8W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
J13 = double(subs(P13W,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
FF = double(subs(PFFW,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
BF = double(subs(PBFW,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
JW = [0 0 0]';

%% Swing leg (with underscore)
% Compute matrices for swing leg
T1W_ = T0W*T10_;
T2W_ = T1W_*T21_;
T3W_ = T2W_*T32_;
T4W_ = T2W_*T42_;
T5W_ = T4W_*T54_;
T6W_ = T5W_*T65_;
T7W_ = T6W_*T76_;
T8W_ = T5W_*T85_;
T13W_ = T7W_*T13_7_;
TFFW_ = T13W_*TFF_13_;
TBFW_ = T13W_*TBF_13_;

% Compute joints' position vectors of swing leg
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
PFFW_ = TFFW_(1:3,end);
PBFW_ = TBFW_(1:3,end);

% Subs angles
J0_ = double(subs(P0W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
    
J1_ = double(subs(P1W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
    
J2_ = double(subs(P2W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
    
J3_ = double(subs(P3W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
    
J4_ = double(subs(P4W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
    
J5_ = double(subs(P5W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
    
J6_ = double(subs(P6W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
    
J7_ = double(subs(P7W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
    
J8_ = double(subs(P8W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
    
J13_ = double(subs(P13W_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
    
FF_ = double(subs(PFFW_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
    
BF_ = double(subs(PBFW_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},...
        {Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));

%% Define mass of each link 
% Total mass [kg]
mTot = 65;                            
% Define mass of each link of mechanism and save it in a structure
Mass = massOfLink(L1,L2,L3,L4,L5,L6,L7,frontFootLength,backFootLength,mTot);

%% Define COM of each link respect to world frame standing leg

% In these cases are symbolical expressions
COM1 = centerOfMass(T0W,T10);
COM2 = centerOfMass(T0W*T10,T21*T42);
COM3 = centerOfMass(T0W*T20,T32);
COM4 = centerOfMass(T0W*T40,T54*T65);
COM5 = centerOfMass(T0W*T60,T76*T13_7);
COM6 = centerOfMass(T0W*T50,T85);
COM7 = centerOfMass(T0W*T70,T87);
COM_FRONT_FOOT = centerOfMass(T13W,TFF_13);
COM_BACK_FOOT = centerOfMass(T13W,TBF_13);
COM_FOOT = (Mass(8).FirstLeg*COM_FRONT_FOOT + Mass(9).FirstLeg*COM_BACK_FOOT)/(Mass(8).FirstLeg + Mass(9).FirstLeg);

%% Subs angles in position of COMs of supporting leg
% Subs of angles
com1 = double(subs(COM1,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
com2 = double(subs(COM2,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
com3 = double(subs(COM3,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
com4 = double(subs(COM4,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
com5 = double(subs(COM5,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
com6 = double(subs(COM6,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
com7 = double(subs(COM7,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));
comFoot = double(subs(COM_FOOT,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,gamma}));

%% Define COM of each link respect to world frame swing leg

% In these cases are symbolical expressions
COM1_ = centerOfMass(T0W*eye(4),T10_);
COM2_ = centerOfMass(T0W*T10_,T21_*T42_);
COM3_ = centerOfMass(T1W_*T21_,T32_);
COM4_ = centerOfMass(T2W_*T42_,T54_*T65_);
COM5_ = centerOfMass(T5W_*T65_,T76_*T13_7_);
COM6_ = centerOfMass(T4W_*T54_,T85_);
COM7_ = centerOfMass(T0W*T70_,T87_);
COM_FRONT_FOOT_ = centerOfMass(T13W_,TFF_13_);
COM_BACK_FOOT_ = centerOfMass(T13W_,TBF_13_);
COM_FOOT_ = (Mass(8).SecondLeg*COM_FRONT_FOOT_ + Mass(9).SecondLeg*COM_BACK_FOOT_)/(Mass(8).SecondLeg + Mass(9).SecondLeg);


%% Subs angles in position of COMs of swing leg 
% Subs of angles
com1_ = double(subs(COM1_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
com2_ = double(subs(COM2_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
com3_ = double(subs(COM3_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
com4_ = double(subs(COM4_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
com5_ = double(subs(COM5_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
com6_ = double(subs(COM6_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
com7_ = double(subs(COM7_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));
comFoot_ = double(subs(COM_FOOT_,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));


%% COM of body
COM_BODY = [P8W(1,end);P8W(2,end);P8W(3,end);1];

comBody = double(subs(COM_BODY,{theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta1_,theta2_,theta3_,theta4_,theta5_,theta6_,theta7_,GAMMA},{Theta1,angle2,angle3,angle4,angle5,angle6,angle7,Theta1_,angle2_,angle3_,angle4_,angle5_,angle6_,angle7_,gamma}));

%% Vector position of standing leg
% In this section of code it has been defined the vector position of each
% joint respect to the center of mass of each link for the standing leg

for i=1:length(Theta1)
    
    %com5
    r5_13(:,i) = vectorPosition(com5(1:3,i),J13(1:3,i));
    r57(:,i) = vectorPosition(com5(1:3,i),J7(1:3,i));
    r56(:,i) = vectorPosition(com5(1:3,i),J6(1:3,i));
    
    %com7
    r77(:,i) = vectorPosition(com7(1:3,i),J7(1:3,i));
    r78(:,i) = vectorPosition(com7(1:3,i),J8(1:3,i));
    
    %com6
    r65(:,i) = vectorPosition(com6(1:3,i),J5(1:3,i));
    r68(:,i) = vectorPosition(com6(1:3,i),J8(1:3,i));
    
    %com4
    r46(:,i) = vectorPosition(com4(1:3,i),J6(1:3,i));
    r45(:,i) = vectorPosition(com4(1:3,i),J5(1:3,i));
    r44(:,i) = vectorPosition(com4(1:3,i),J4(1:3,i));
    
    %com3
    r33(:,i) = vectorPosition(com3(1:3,i),J3(1:3,i));
    r32(:,i) = vectorPosition(com3(1:3,i),J2(1:3,i));
    
    %com2
    r24(:,i) = vectorPosition(com2(1:3,i),J4(1:3,i));
    r21(:,i) = vectorPosition(com2(1:3,i),J1(1:3,i));
    
    %com1
    r10(:,i) = vectorPosition(com1(1:3,i),J0(1:3,i));
    r11(:,i) = vectorPosition(com1(1:3,i),J1(1:3,i));

    %comBody 
    rBody8(:,i) = vectorPosition(comBody(1:3,i),J8(1:3,i));
    rBody3(:,i) = vectorPosition(comBody(1:3,i),J3(1:3,i));
    rBody0(:,i) = vectorPosition(comBody(1:3,i),J0(1:3,i));
    
    %comFoot
    rFoot13(:,i) = vectorPosition(comFoot(1:3,i),J13(1:3,i));
end


%% Vector position of swing leg

% In this section of code it has been defined the vector position of each
% joint respect to the center of mass of each link for the swing leg

for i=1:length(Theta1_)
    
    %com5
    r5_13_(:,i) = vectorPosition(com5_(1:3,i),J13_(1:3,i));
    r57_(:,i) = vectorPosition(com5_(1:3,i),J7_(1:3,i));
    r56_(:,i) = vectorPosition(com5_(1:3,i),J6_(1:3,i));
    
    %com7
    r77_(:,i) = vectorPosition(com7_(1:3,i),J7_(1:3,i));
    r78_(:,i) = vectorPosition(com7_(1:3,i),J8_(1:3,i));
    
    %com6
    r65_(:,i) = vectorPosition(com6_(1:3,i),J5_(1:3,i));
    r68_(:,i) = vectorPosition(com6_(1:3,i),J8_(1:3,i));
    
    %com4
    r46_(:,i) = vectorPosition(com4_(1:3,i),J6_(1:3,i));
    r45_(:,i) = vectorPosition(com4_(1:3,i),J5_(1:3,i));
    r44_(:,i) = vectorPosition(com4_(1:3,i),J4_(1:3,i));
    
    %com3
    r33_(:,i) = vectorPosition(com3_(1:3,i),J3_(1:3,i));
    r32_(:,i) = vectorPosition(com3_(1:3,i),J2_(1:3,i));
    
    %com2
    r24_(:,i) = vectorPosition(com2_(1:3,i),J4_(1:3,i));
    r21_(:,i) = vectorPosition(com2_(1:3,i),J1_(1:3,i));
    
    %com1
    r10_(:,i) = vectorPosition(com1_(1:3,i),J0_(1:3,i));
    r11_(:,i) = vectorPosition(com1_(1:3,i),J1_(1:3,i));

    %comBody 
    rBody8_(:,i) = vectorPosition(comBody(1:3,i),J8_(1:3,i));
    rBody3_(:,i) = vectorPosition(comBody(1:3,i),J3_(1:3,i));
    rBody0_(:,i) = vectorPosition(comBody(1:3,i),J0_(1:3,i));
    
    %comFoot
    rFoot13_(:,i) = vectorPosition(comFoot_(1:3,i),J13_(1:3,i));
end

%% Load data 
% This section is useful in order to not run the previous line code
% It contains all data that must be used in quasi static analysis such as 
% position of center of mass of each link
% vector position of each joint respect to each COM of links
% mass of each link and mass of body
% Remind that COM of hip in this case is coincident to the position of Joint 8
load('QuasiStaticData.mat');

%% Symbolic variables for each leg

% In this section are defined the symbolic variables that define forces
% acting between links

% Standing Leg
syms Fb1x Fb1y F21x F21y F32x F32y Fb3x Fb3y F42x F42y F64x F64y F54x F54y 
syms Cez Caz 
syms F75x F75y F76x F76y Ff6x Ff6y Fb6x Fb6y Fb7x Fb7y 
syms Ff5x Ff5y 
syms R10x R10y R11x R11y R24x R24y R21x R21y R33x R33y R32x R32y R46x R46y R45x R45y 
syms R44x R44y R5_13x R5_13y R57x R57y R56x R56y R65x R65y R68x R68y R77x R77y R78x R78y 
syms Rb8x Rb8y Rb3x Rb3y Rb0x Rb0y RFoot_13x RFoot_13y 
syms Caz_ Cez_ 

% Swing Leg
syms Fb1x_ Fb1y_ F21x_ F21y_ F32x_ F32y_ Fb3x_ Fb3y_ F42x_ F42y_ F64x_ F64y_ F54x_ F54y_ 
syms F75x_ F75y_ Ff5x_ Ff5y_ Fb6x_ Fb6y_ Fb7x_ Fb7y_ 
syms R10x_ R10y_ R11x_ R11y_ R24x_ R24y_ R21x_ R21y_ R33x_ R33y_ R32x_ R32y_ R46x_ R46y_ R45x_ R45y_ 
syms R44x_ R44y_ R5_13x_ R5_13y_ R57x_ R57y_ R56x_ R56y_ R65x_ R65y_ R68x_ R68y_ R77x_ R77y_ R78x_ R78y_ 
syms Rb8x_ Rb8y_ Rb3x_ Rb3y_ Rb0x_ Rb0y_ RFoot_13x_ RFoot_13y_ 
syms Ff5x_ Ff5y_ 

%% Forces for each leg
% Gravity force
g = [0 -9.81 0]';
% Forces for standing leg
Fb1 = [Fb1x Fb1y 0].'; 
F21 = [F21x F21y 0].'; 
F32 = [F32x F32y 0].'; 
Fb3 = [Fb3x Fb3y 0].'; 
F42 = [F42x F42y 0].';
F64 = [F64x F64y 0].';
F54 = [F54x F54y 0].';
F75 = [F75x F75y 0].';
F76 = [F76x F76y 0].'; 
Ff6 = [Ff6x Ff6y 0].';
Fb6 = [Fb6x Fb6y 0].'; 
Fb7 = [Fb7x Fb7y 0].';
Ff5 = [Ff5x Ff5y 0].';

% Forces for swing leg 
Fb1_ = [Fb1x_ Fb1y_ 0].'; 
F21_ = [F21x_ F21y_ 0].'; 
F32_ = [F32x_ F32y_ 0].'; 
Fb3_ = [Fb3x_ Fb3y_ 0].'; 
F42_ = [F42x_ F42y_ 0].';
F64_ = [F64x_ F64y_ 0].';
F54_ = [F54x_ F54y_ 0].';
F75_ = [F75x_ F75y_ 0].'; 
Fb6_ = [Fb6x_ Fb6y_ 0].'; 
Fb7_ = [Fb7x_ Fb7y_ 0].';
Ff5_ = [Ff5x_ Ff5y_ 0].';

%% Vector position of point of application of forces for each leg
% Vector position for standing leg
R10 = [R10x R10y 0].';
R11 = [R11x R11y 0].';
R24 = [R24x R24y 0].';
R21 = [R21x R21y 0].';
R33 = [R33x R33y 0].';
R32 = [R32x R32y 0].';
R46 = [R46x R46y 0].';
R45 = [R45x R45y 0].';
R44 = [R44x R44y 0].';
R5_13 = [R5_13x R5_13y 0].';
R57 = [R57x R57y 0].';
R56 = [R56x R56y 0].';
R65 = [R65x R65y 0].';
R68 = [R68x R68y 0].';
R77 = [R77x R77y 0].';
R78 = [R78x R78y 0].';
RBody8 = [Rb8x Rb8y 0].';
RBody3 = [Rb3x Rb3y 0].';
RBody0 = [Rb0x Rb0y 0].';
RFoot_13 = [RFoot_13x RFoot_13y 0].';

% Vector position for swing leg
R10_ = [R10x_ R10y_ 0].';
R11_ = [R11x_ R11y_ 0].';
R24_ = [R24x_ R24y_ 0].';
R21_ = [R21x_ R21y_ 0].';
R33_ = [R33x_ R33y_ 0].';
R32_ = [R32x_ R32y_ 0].';
R46_ = [R46x_ R46y_ 0].';
R45_ = [R45x_ R45y_ 0].';
R44_ = [R44x_ R44y_ 0].';
R5_13_ = [R5_13x_ R5_13y_ 0].';
R57_ = [R57x_ R57y_ 0].';
R56_ = [R56x_ R56y_ 0].';
R65_ = [R65x_ R65y_ 0].';
R68_ = [R68x_ R68y_ 0].';
R77_ = [R77x_ R77y_ 0].';
R78_ = [R78x_ R78y_ 0].';
RBody8_ = [Rb8x_ Rb8y_ 0].';
RBody3_ = [Rb3x_ Rb3y_ 0].';
RBody0_ = [Rb0x_ Rb0y_ 0].';
RFoot_13_ = [RFoot_13x_ RFoot_13y_ 0].';

% Actuated torque Ca and elastic torque Ce for each leg
Ca = [0 0 Caz].';
Ce = [0 0 Cez].';
Ca_ = [0 0 Caz_].';
Ce_ = [0 0 Cez_].';

%% Newton-Euler formulation
% In this case inertia forces and moments haven't been considered
% Equations for standing leg

% Equations for L1
eq1 =  Fb1 + F21 + Mass(1).FirstLeg*g == 0; 
eq11 = cross(R10,Fb1) + cross(R11,F21) + Ca == 0;


% Equations for L2
eq2 = -F21 + F42 + F32 + Mass(2).FirstLeg*g == 0; 
eq22 = cross(R21,-F21) + cross(R24,F42) == 0;

% Equations for L3
eq3 =  Fb3 - F32 + Mass(3).FirstLeg*g == 0; 
eq33 = cross(R33,Fb3) + cross(R32,-F32) == 0;

% Equations for L4
eq4 = -F42 + F64 + F54 + Mass(4).FirstLeg*g == 0; 
eq44 = cross(R44,-F42) + cross(R45,F64) + cross(R46,F54) == 0;

% Equations for L5
eq5 = -F54 + F75 + Ff5 + Mass(5).FirstLeg*g == 0; 
eq55 = cross(R56,-F54) + cross(R57,F75) + cross(R5_13,Ff5) + Ce == 0;

% Equations for L6
eq6 = -F64 + Fb6 + Mass(6).FirstLeg*g == 0; 
eq66 = cross(R65,-F64) + cross(R68,Fb6) == 0;

% Equations for L7
eq7 = Fb7 -F75 + Mass(7).FirstLeg*g == 0; 
eq77 = cross(R78,Fb7) + cross(R77,-F75) == 0;

% Equations for swing leg

% Equations for L1
eq1_ =  Fb1_ + F21_ + Mass(1).SecondLeg*g == 0; 
eq11_ = cross(R10_,Fb1_) + cross(R11_,F21_) + Ca_ == 0;


% Equations for L2
eq2_ = -F21_ + F42_ + F32_ + Mass(2).SecondLeg*g == 0; 
eq22_ = cross(R21_,-F21_) + cross(R24_,F42_) == 0;

% Equations for L3
eq3_ =  Fb3_ - F32_ + Mass(3).SecondLeg*g == 0; 
eq33_ = cross(R33_,Fb3_) + cross(R32_,-F32_) == 0;

% Equations for L4
eq4_ = -F42_ + F64_ + F54_ + Mass(4).SecondLeg*g == 0; 
eq44_ = cross(R44_,-F42_) + cross(R45_,F64_) + cross(R46_,F54_) == 0;

% Equations for L5
eq5_ = -F54_ + F75_ + Ff5_ + Mass(5).SecondLeg*g == 0; 
eq55_ = cross(R56_,-F54_) + cross(R57_,F75_) + cross(R5_13_,Ff5_) + Ce_ == 0;

% Equations for L6
eq6_ = -F64_ + Fb6_ + Mass(6).SecondLeg*g == 0; 
eq66_ = cross(R65_,-F64_) + cross(R68_,Fb6_) == 0;

% Equations for L7
eq7_ = Fb7_ -F75_ + Mass(7).SecondLeg*g == 0; 
eq77_ = cross(R78_,Fb7_) + cross(R77_,-F75_) == 0;

% Equations for swing foot 
eq8_ = -Ff5_ + (Mass(8).SecondLeg + Mass(9).SecondLeg)*g == 0;
eq88_ = cross(RFoot_13_,-Ff5_) - Ce_ == 0;


% Equations for the hip
eq9 = -Fb1 - Fb3 - Fb6 - Fb7 -Fb1_ - Fb3_ - Fb6_ - Fb7_ + Mass(1).Body*g == 0; 
eq99 = cross(RBody0,-Fb1) + cross(RBody3,-Fb3) + cross(RBody8,-Fb6) + cross(RBody8,-Fb7) - Ca + cross(RBody0_,-Fb1_) + cross(RBody3_,-Fb3_) + cross(RBody8_,-Fb6_) + cross(RBody8_,-Fb7_) - Ca_  == 0;

%% System of linear equations

eq = [  eq1(1),eq1(2),...
        eq2(1),eq2(2),...
        eq3(1),eq3(2),...
        eq4(1),eq4(2),...
        eq5(1),eq5(2),...
        eq6(1),eq6(2),...
        eq7(1),eq7(2),...
        eq9(1),eq9(2),...
          eq11(3),...
          eq22(3),...
          eq33(3),...
          eq44(3),...
          eq55(3),...
          eq66(3),...
          eq77(3),...
          eq99(3)...
        eq1_(1),eq1_(2),...
        eq2_(1),eq2_(2),...
        eq3_(1),eq3_(2),...
        eq4_(1),eq4_(2),...
        eq5_(1),eq5_(2),...
        eq6_(1),eq6_(2),...
        eq7_(1),eq7_(2),...
        eq8_(1),eq8_(2),...
          eq11_(3),...
          eq22_(3),...
          eq33_(3),...
          eq44_(3),...
          eq55_(3),...
          eq66_(3),...
          eq77_(3),...
          eq88_(3)
          ];
      
%% Define output variables of the system of equations

 vars = [Fb1x Fb1y ...
         F21x F21y ...
         F32x F32y ...
         Fb3x Fb3y ...
         F42x F42y ...
         F64x F64y ...
         F54x F54y ...
         F75x F75y ...
         Ff5x Ff5y ...
         Fb6x Fb6y ...
         Fb7x Fb7y ...
         Cez Caz ...
         Fb1x_ Fb1y_ ...
         F21x_ F21y_ ...
         F32x_ F32y_ ...
         Fb3x_ Fb3y_ ...
         F42x_ F42y_ ...
         F64x_ F64y_ ...
         F54x_ F54y_ ...
         F75x_ F75y_ ...
         Ff5x_ Ff5y_ ...
         Fb6x_ Fb6y_ ...
         Fb7x_ Fb7y_ ...
         Cez_ Caz_ ...
         ];
%% Define coefficients matrix A and known terms vector b
% Transform the system of equations into a matrix form
[A,b] = equationsToMatrix(eq,vars);
b = double(b);

%% Subs of vector position of point of application of forces in A
j=1;
for i = 27:49
A_subs = double(subs(A,{R10x,R10y,R11x,R11y,R24x,R24y,R21x,R21y,R33x,R33y,R32x,R32y,R46x,R46y,R45x,R45y,R44x,R44y,R5_13x,R5_13y,R57x,R57y,R56x,R56y,R65x,R65y,R68x,R68y,R77x,R77y,R78x,R78y,Rb8x,Rb8y,Rb3x,Rb3y,Rb0x,Rb0y,RFoot_13x,RFoot_13y...
                        R10x_,R10y_,R11x_,R11y_,R24x_,R24y_,R21x_,R21y_,R33x_,R33y_,R32x_,R32y_,R46x_,R46y_,R45x_,R45y_,R44x_,R44y_,R5_13x_,R5_13y_,R57x_,R57y_,R56x_,R56y_,R65x_,R65y_,R68x_,R68y_,R77x_,R77y_,R78x_,R78y_,Rb8x_,Rb8y_,Rb3x_,Rb3y_,Rb0x_,Rb0y_,RFoot_13x_,RFoot_13y_},...
                        {r10(1,i),r10(2,i),r11(1,i),r11(2,i),r24(1,i),r24(2,i),r21(1,i),r21(2,i),r33(1,i),r33(2,i),r32(1,i),r32(2,i),r46(1,i),r46(2,i),r45(1,i),r45(2,i),r44(1,i),r44(2,i),r5_13(1,i),r5_13(2,i),r57(1,i),r57(2,i),r56(1,i),r56(2,i),r65(1,i),r65(2,i),r68(1,i),r68(2,i),r77(1,i),r77(2,i),r78(1,i),r78(2,i),rBody8(1,i),rBody8(2,i),rBody3(1,i),rBody3(2,i),rBody0(1,i),rBody0(2,i),rFoot13(1,i),rFoot13(2,i)...
                        r10_(1,i),r10_(2,i),r11_(1,i),r11_(2,i),r24_(1,i),r24_(2,i),r21_(1,i),r21_(2,i),r33_(1,i),r33_(2,i),r32_(1,i),r32_(2,i),r46_(1,i),r46_(2,i),r45_(1,i),r45_(2,i),r44_(1,i),r44_(2,i),r5_13_(1,i),r5_13_(2,i),r57_(1,i),r57_(2,i),r56_(1,i),r56_(2,i),r65_(1,i),r65_(2,i),r68_(1,i),r68_(2,i),r77_(1,i),r77_(2,i),r78_(1,i),r78_(2,i),rBody8_(1,i),rBody8_(2,i),rBody3_(1,i),rBody3_(2,i),rBody0_(1,i),rBody0_(2,i),rFoot13_(1,i),rFoot13_(2,i)}));
solution(:,j) = inv(A_subs)*b;
j=j+1;
end
%% Compute actuated torque Ca and elastic torque Ce
Ca = solution(find(vars=='Caz'),:);
Ca_ = solution(find(vars=='Caz_'),:);
Ce = solution(find(vars=='Cez'),:);
Ce_ = solution(find(vars=='Cez_'),:);

%% Definition of beta angle that is the angle between foot and tibia

% In radians
beta = beta(27:49);
% In degree
betaDeg = rad2deg(beta);

%% Compute elastic coefficient k of torsion spring
% Define the repose angle of torsion spring 
% Remind that Ce = -k*(beta-beta0)

deltaBetaRad = beta-beta(1);
deltaBetaDeg = rad2deg(deltaBetaRad);

%% Least-squares approximation to find an approximate value of spring coefficient

p = polyfit(beta,Ce,1);
regressionLine = polyval(p,beta);
figure()
grid on
hold on
plot(regressionLine,'b');
legend('Regression Line');
% ke (spring coefficient) is obteined considering the slope of regression line
ke = -p(1);


%% Plot torques

figure(1)
hold on
grid on
plot(betaDeg,Ca);
plot(betaDeg,Ce);
plot(betaDeg,Ca_);
plot(betaDeg,Ce_);
plot(betaDeg,regressionLine);
legend('Ca','Ce','Ca_','Ce_','RegressionLine','Location','best')

%% Movie
figure('units','normalized','outerposition',[0 0 1 1])
view(2)
set(gca,'nextplot','replacechildren');
v = VideoWriter('QuasiStaticAnalysis.avi');
open(v);
hold on
grid on
for i = 27:49
    j = i-26;
    clf
    hold on
    grid on
    hold on
    grid on
    t = tiledlayout(2,2);
    title(t,'Quasi-Static Analysis','Color','r')
    ax1 = nexttile([2 1]);
    grid on
    axis equal
    axis([-0.5 1.2 -0.2 1.7])
    xlabel(ax1,'X [m]');ylabel(ax1,'Y [m]');
    % Standing leg
    set(gca,'FontSize',10)
    hold on
    h1 = plot([J0(1,i),J1(1,i)],[J0(2,i),J1(2,i)],'b','LineWidth',2);
    h2 = plot([J1(1,i),J2(1,i)],[J1(2,i),J2(2,i)],'b','LineWidth',2);
    h3 = plot([J2(1,i),J3(1,i)],[J2(2,i),J3(2,i)],'b','LineWidth',2);
    h4 = plot([J2(1,i),J4(1,i)],[J2(2,i),J4(2,i)],'b','LineWidth',2);
    h5 = plot([J4(1,i),J5(1,i)],[J4(2,i),J5(2,i)],'b','LineWidth',2);
    h6 = plot([J5(1,i),J6(1,i)],[J5(2,i),J6(2,i)],'b','LineWidth',2);
    h7 = plot([J5(1,i),J8(1,i)],[J5(2,i),J8(2,i)],'b','LineWidth',2);
    h8 = plot([J6(1,i),J7(1,i)],[J6(2,i),J7(2,i)],'b','LineWidth',2);
    h9 = plot([J7(1,i),J8(1,i)],[J7(2,i),J8(2,i)],'b','LineWidth',2);
    h10 = plot([J7(1,i),J13(1,i)],[J7(2,i),J13(2,i)],'b','LineWidth',2);
    h11 = plot([J13(1,i),FF(1,i)],[J13(2,i),FF(2,i)],'b','LineWidth',2);
    h12 = plot([J13(1,i),BF(1,i)],[J13(2,i),BF(2,i)],'b','LineWidth',2);
    hold on
    h13 = plot(J0(1,i),J0(2,i),'blackv','MarkerSize',10);
    text(J0(1,i)-0.06,J0(2,i),'0');
    h13 = plot(J1(1,i),J1(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J1(1,i),J1(2,i)+0.03,'1');
    h14 = plot(J2(1,i),J2(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J2(1,i)+0.02,J2(2,i),'2');
    h15 = plot(J3(1,i),J3(2,i),'blackv','MarkerSize',10);
    text(J3(1,i)+0.02,J3(2,i),'3')
    h17 = plot(J4(1,i),J4(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J4(1,i)+0.01,J4(2,i)+0.02,'4');
    h18 = plot(J5(1,i),J5(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J5(1,i)+0.02,J5(2,i),'5');
    h19 = plot(J6(1,i),J6(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J6(1,i)+0.02,J6(2,i),'6');
    h20 = plot(J7(1,i),J7(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J7(1,i)+0.02,J7(2,i),'7');
    h21 = plot(J8(1,i),J8(2,i),'black^','MarkerSize',10);
    text(J8(1,i)-0.06,J8(2,i)+0.03,'8');
    h22 = plot(J13(1,i),J13(2,i));
    text(J13(1,i)-0.01,J13(2,i)-0.02,'13'); 
    h23 = plot(FF(1,i),FF(2,i));
    text(FF(1,i),FF(2,i),'FF');
    h24 = plot(BF(1,i),BF(2,i));
    text(BF(1,i),BF(2,i),'BF');
    hold on
    % Swing leg
    set(gca,'FontSize',10)
    h25 = plot([J0_(1,i),J1_(1,i)],[J0_(2,i),J1_(2,i)],'color','r','LineWidth',2);
    h26 = plot([J1_(1,i),J2_(1,i)],[J1_(2,i),J2_(2,i)],'color','r','LineWidth',2);
    h27 = plot([J2_(1,i),J3_(1,i)],[J2_(2,i),J3_(2,i)],'color','r','LineWidth',2);
    h28 = plot([J2_(1,i),J4_(1,i)],[J2_(2,i),J4_(2,i)],'color','r','LineWidth',2);
    h29 = plot([J4_(1,i),J5_(1,i)],[J4_(2,i),J5_(2,i)],'color','r','LineWidth',2);
    h30 = plot([J5_(1,i),J6_(1,i)],[J5_(2,i),J6_(2,i)],'color','r','LineWidth',2);
    h31 = plot([J5_(1,i),J8_(1,i)],[J5_(2,i),J8_(2,i)],'color','r','LineWidth',2);
    h32 = plot([J6_(1,i),J7_(1,i)],[J6_(2,i),J7_(2,i)],'color','r','LineWidth',2);
    h33 = plot([J7_(1,i),J8_(1,i)],[J7_(2,i),J8_(2,i)],'color','r','LineWidth',2);
    h34 = plot([J7_(1,i),J13_(1,i)],[J7_(2,i),J13_(2,i)],'color','r','LineWidth',2);
    h35 = plot([J13_(1,i),FF_(1,i)],[J13_(2,i),FF_(2,i)],'color','r','LineWidth',2);
    h36 = plot([J13_(1,i),BF_(1,i)],[J13_(2,i),BF_(2,i)],'color','r','LineWidth',2);
    hold on
    h37 = plot(J0_(1,i),J0_(2,i),'blackv','MarkerSize',10);
    text(J0_(1,i)-0.06,J0_(2,i),'0''');
    h38 = plot(J1_(1,i),J1_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J1_(1,i),J1_(2,i)+3,'1''');
    h39 = plot(J2_(1,i),J2_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J2_(1,i)+0.02,J2_(2,i)+0.01,'2''');
    h40 = plot(J3_(1,i),J3_(2,i),'blackv','MarkerSize',10);
    text(J3_(1,i)+0.02,J3_(2,i),'3''')
    h42 = plot(J4_(1,i),J4_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J4_(1,i),J4_(2,i)+0.03,'4''');
    h43 = plot(J5_(1,i),J5_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J5_(1,i)+0.02,J5_(2,i),'5''');
    h44 = plot(J6_(1,i),J6_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J6_(1,i)+0.02,J6_(2,i),'6''');
    h45 = plot(J7_(1,i),J7_(2,i),'o','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor',[0.7529 0.7529 0.7529]);
    text(J7_(1,i)+0.02,J7_(2,i),'7''');
    h46 = plot(J8_(1,i),J8_(2,i),'black^','MarkerSize',10);
    text(J8_(1,i)-0.06,J8_(2,i)+0.03,'8''');
    h47 = plot(J13_(1,i),J13_(2,i));
    text(J13_(1,i)-0.02,J13_(2,i)-0.03,'13''');
    h48 = plot(FF_(1,i),FF_(2,i));
    text(FF_(1,i),FF_(2,i),'FF''');
    h49 = plot(BF_(1,i),BF_(2,i));
    text(BF_(1,i),BF_(2,i),'BF''');
    h50 = plot(FF_(1,27:49),FF_(2,27:49),'black:','LineWidth',1.5);
    %h55 = plot(com1(1,i),com1(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','black');
    %text(com1(1,i)+0.01,com1(2,i),'1');
    %h56 = plot(com2(1,i),com2(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','black');
    %text(com2(1,i)+0.01,com2(2,i),'2');
    %h57 = plot(com3(1,i),com3(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','black');
    %text(com3(1,i)+0.01,com3(2,i),'3');
    %h58 = plot(com4(1,i),com4(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','black');
    %text(com4(1,i)+0.01,com4(2,i),'4');
    %h59 = plot(com5(1,i),com5(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','black');
    %text(com5(1,i)+0.01,com5(2,i),'5');
    %h60 = plot(com6(1,i),com6(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','black');
    %text(com6(1,i)+0.01,com6(2,i),'6');
    %h61 = plot(com7(1,i),com7(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','black');
    %text(com7(1,i)+0.01,com7(2,i),'7');
    %h62 = plot(comFoot(1,i),comFoot(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','black');
    %text(comFoot(1,i)+0.01,comFoot(2,i),'Foot');
    %h63 = plot(com1_(1,i),com1_(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','g');
    %text(com1_(1,i)+0.01,com1_(2,i),'1''');
    %h64 = plot(com2_(1,i),com2_(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','g');
    %text(com2_(1,i)+0.01,com2_(2,i),'2''');
    %h65 = plot(com3_(1,i),com3_(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','g');
    %text(com3_(1,i)+0.01,com3_(2,i),'3''');
    %h66 = plot(com4_(1,i),com4_(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','g');
    %text(com4_(1,i)+0.01,com4_(2,i),'4''');
    %h67 = plot(com5_(1,i),com5_(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','g');
    %text(com5_(1,i)+0.01,com5_(2,i),'5''');
    %h68 = plot(com6_(1,i),com6_(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','g');
    %text(com6_(1,i)+0.01,com6_(2,i),'6''');
    %h69 = plot(com7_(1,i),com7_(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','g');
    %text(com7_(1,i)+0.01,com7_(2,i),'7''');
    %h70 = plot(comFoot_(1,i),comFoot_(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','g');
    %text(comFoot_(1,i)+0.01,comFoot_(2,i),'Foot''');
    h71 = plot(comBody(1,i),comBody(2,i),'o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','m');
    %text(comBody(1,i)+0.01,comBody(2,i),'Body');
    legend(h71,{'COM Hip'},'Location','NorthEast')
    title(ax1,'Walking Robot')
    
    % Ca e Ca_
    ax2 = nexttile;
    grid on
    hold on
    yyaxis(ax2,'left')
    set(ax2,{'ycolor'},{'g'})
    h72 = plot(beta(1,1:j),Ca(1,1:j),'color','g','LineWidth',1.5);
    axis([1 1.35 min(Ca)-2 max(Ca)+2])
    xlabel(ax2,'\beta');ylabel(ax2,'Torque [Nm]','Color','black');
    hold on
    yyaxis(ax2,'right')
    set(ax2,{'ycolor'},{'r'})
    h74 = plot(beta(1,1:j),Ca_(1,1:j),'color','r','LineWidth',1.5);
    axis([1 1.35 min(Ca_)-2 max(Ca_)+2])
    xlabel(ax2,'\beta [rad]');
    legend({'Ta','Ta'''},'location','North')
    title(ax2,'Actuated Torques')
    %Ce e Ce_
    ax3 = nexttile;
    yyaxis(ax3,'left')
    set(ax3,{'ycolor'},{'m'})
    h73 = plot(beta(1,1:j),Ce(1,1:j),'color','m','LineWidth',1.5);
    hold on
    h76 = plot(beta(1,1:j),regressionLine(1,1:j),'color','black','LineWidth',1.5);
    axis([1 1.35 min(Ce)-2 max(Ce)+2])
    xlabel(ax3,'\beta');ylabel(ax3,'Torque [Nm]','Color','black');
    yyaxis(ax3,'right')
    set(ax3,{'ycolor'},{'b'})
    h75 = plot(beta(1,1:j),Ce_(1,1:j),'color','b','LineWidth',1.5);
    grid on
    axis([1 1.35 min(Ce_)-0.1 max(Ce_)+0.1])
    xlabel(ax3,'\beta [rad]');
    legend({'Te','Regression Line','Te'''},'location','North')
    title(ax3,'Elastic Torques')
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

%% Function that defines mass of each link
function Mass = massOfLink(L1,L2,L3,L4,L5,L6,L7,frontFootLength,backFootLength,mtot)

trunk = 0.25*mtot;       % 25% of total mass
thigh = 0.10*mtot;       % 10% of total mass
shank = 0.05*mtot;       % 5% of total mass
foot = 0.02*mtot;        % 2% of total mass

m1 = round(L1*(thigh/(L1+L2+L3+L4+L6)),1);               %mass of L1
m2 = round(L2*(thigh/(L1+L2+L3+L4+L6)),1);               %mass of L2
m3 = round(L3*(thigh/(L1+L2+L3+L4+L6)),1);               %mass of L3
m4 = round(L4*(thigh/(L1+L2+L3+L4+L6)),1);               %mass of L4
m6 = round(L6*(thigh/(L1+L2+L3+L4+L6)),1);               %mass of L6
m5 = round(L5*(shank/(L5+L7)),1);                        %mass of L5
m7 = round(L7*(shank/(L5+L7)),1);                        %mass of L7
mBody = round(trunk,1);                                  %mass of body
mFrontFoot = round(frontFootLength*foot/(frontFootLength + backFootLength),1);
mBackFoot = round(backFootLength*foot/(frontFootLength + backFootLength),1);
Mass = struct('FirstLeg',{m1,m2,m3,m4,m5,m6,m7,mFrontFoot,mBackFoot},'SecondLeg',{m1,m2,m3,m4,m5,m6,m7,mFrontFoot,mBackFoot},'Body',{mBody,0,0,0,0,0,0,0,0});
end


%% Function that defines center of mass (COM) of each link respect to base frame 0 
function COM = centerOfMass(TiMinus1,Ti_iMinus1)

COM = TiMinus1*[(Ti_iMinus1(1:3,end)/2); 1];

end

%% Function that defines vector position of point of application of forces respect to the COM of link

function rij = vectorPosition(Pi,Pj)

rij = Pj-Pi;

end
