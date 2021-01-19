%Geometric Jacobian matrix (6 x n)
%jointType R means that it has been considered revolute joint
%jointType P means that it has been considered prismatic joint
function [J_I] = Jacobian(T_Iminus1_0,T_E0,jointType)
    if (jointType == 'R')
        JP = cross(T_Iminus1_0(1:3,3),(T_E0(1:3,end)-T_Iminus1_0(1:3,end)));
        JO = T_Iminus1_0(1:3,3);
    else
        JP = T_Iminus1_0(1:3,3);
        JO = [0 0 0]';
    end
    J_I = [JP;JO];
end