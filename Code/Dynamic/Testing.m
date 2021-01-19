%% Compute force between L5 and foot
Ff5xs = solution(find(vars=='Ff5x'),:);
Ff5ys = solution(find(vars=='Ff5y'),:);
figure(1)
plot(beta,Ff5xs,'r')
xlabel('\beta [rad]')
ylabel('Ff5x [N]')
title('x component of Ff5')
figure(2)
plot(beta,Ff5ys,'b')
xlabel('\beta [rad]')
ylabel('Ff5y [N]')
title('y component of Ff5')

%% Compute force between L5' and foot'
Ff5xs_ = solution(find(vars=='Ff5x_'),:);
Ff5ys_ = solution(find(vars=='Ff5y_'),:);
figure(1)
plot(beta,Ff5xs_,'r')
xlabel('\beta [rad]')
ylabel('Ff5x [N]')
title('x component of Ff5''')
figure(2)
plot(beta,Ff5ys_,'b')
xlabel('\beta [rad]')
ylabel('Ff5y [N]')
title('y component of Ff5''')

%% Power consuption
% Compute angular velocity of motor crank 
t = 1; % angular velocity of motor crank equal to 0.1 rad/s
%t = 0.02; % angular velocity of motor crank equal to 5 rad/s
w = diff(Theta1)/t;
% Compute power consuption for each leg 
P = Ca.*w(1:length(Ca));
P_ = Ca_.*w(1:length(Ca_));
% Plot
figure(1)
subplot(2,2,1)
plot(beta,P);
xlabel('\beta [rad]')
ylabel('P [W]')
title('Power consuption Standing Leg [W]');
subplot(2,2,3)
plot(beta,Ca);
xlabel('\beta [rad]')
ylabel('T_a [Nm]')
title('Actuated torque Standing Leg [Nm]');
subplot(2,2,2)
plot(beta,P_);
xlabel('\beta [rad]')
ylabel('P'' [Nm]')
title('Power consuption Swing Leg [W]');
subplot(2,2,4)
plot(beta,Ca_);
xlabel('\beta [rad]')
ylabel('T_a'' [Nm]')
title('Actuated torque Swing Leg [Nm]');
