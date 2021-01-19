com = comBody(1:3,:);

% Compute linear velocity of COM of hip
vBodyx = diff(com(1,:));
vBodyy = diff(com(2,:));

% Compute linear acceleration of COM of hip
aBodyx = diff(vBodyx);
aBodyy = diff(vBodyy);

% Plot 
figure(1)
subplot(2,1,1)
plot(beta,vBodyx(27:49),'r',beta,vBodyy(27:49),'b');legend('v_x','v_y')
title('Linear velocity of hip')
xlabel('\beta [rad]')
ylabel('v [m/s]')
subplot(2,1,2)
plot(beta,aBodyx(27:49),'r',beta,aBodyy(27:49),'b');legend('a_x','a_y')
title('Linear acceleration of hip')
xlabel('\beta [rad]')
ylabel('a [m^2/s]')

