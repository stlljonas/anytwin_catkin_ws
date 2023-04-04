function ik_test_trajectory(legName)
%IKTESTTRAJECTORY Test the inverse kinematics along a predefined
%trajectory.

%   Author(s): C. Dario Bellicoso


if (legName == 'LF')
    fore =  1;
    leg = 0;
elseif (legName == 'RF')
    fore =  1;
    leg = 1;
elseif (legName == 'LH')
    fore = -1;
    leg = 2;
elseif (legName == 'RH')
    fore = -1;
    leg = 3;
else
    disp('Unhandled leg name!');
    q = nan;
    return;
end

if (leg == 0)
    flipX = 1;
    flipY = 1;
elseif (leg == 1)
    flipX = 1;
    flipY = -1;
elseif (leg == 2)
    flipX = -1;
    flipY = 1;
elseif (leg ==3)
    flipX = -1;
    flipY = -1;
end


B_r_BH = [flipX*0.2595;  
          flipY*0.1150;  
          0.0000];

p0 = B_r_BH + [0; 
               0; 
               -0.5];
pf = B_r_BH + [0*flipX*0.15; 
               0*flipY*0.05; 
               -0.2];
  
maxDistanceZ = 0.25;

dt = 0.01;
t0 = 0;
tf = 5;

num_samples = (tf-t0)/dt;

time = linspace(t0, tf, num_samples);

p = zeros(3, num_samples);
p_fromIk = zeros(3, num_samples);
ik_error = zeros(1, num_samples);

for k=1:num_samples
%     p(1:2,k) = p0(1:2) + flipX*k*dt/(tf-t0)*( pf(1:2)-p0(1:2) );
%     p(3, k) = p0(3) - maxDistanceZ*sin( k * pi/((tf-t0)/dt) );
    p(:,k) = p0(:) + k*dt/(tf-t0)*(pf-p0);
    
    q = computeLegJointsFromPositionBaseToFootInBaseFrame(p(:,k), legName);
    p_fromIk(:,k) = computeLegPoseFromQ(q, legName);
    
    ik_error(k) = norm(p(:,k)-p_fromIk(:,k));
    
    joints(:,k) = q;
end


figure1 = figure;
subplot1 = subplot(1,2,1,'Parent',figure1);

title('Leg trajectory and ik');
grid on;
hold on;

% xlim([-0.1 0.2]);
% ylim([-0.1 0.1]);
% zlim([p0(3) p0(3)+maxDistanceZ*1.1]);

view(subplot1,[-20 32]);

plot31 = plot3(p(1,:), p(2,:), p(3,:), 'r');
plot32 = plot3(p_fromIk(1,:), p_fromIk(2,:), p_fromIk(3,:), 'b','Parent',subplot1);

set(plot31,'Color',[1 0 0],'DisplayName','reference');
set(plot32,'Color',[0 0 1],'DisplayName','ik + fk');

legend1 = legend(subplot1,'show');
set(legend1,...
    'Position',[0.4 0.8 0.08 0.07]);


subplot(1,2,2);
title('Inverse kinematics error');
grid on;
hold on;
plot(time, ik_error);


figureKnee = figure;
hold on;
grid on;
plot(time, joints(1,:),'r');
plot(time, joints(2,:),'g');
plot(time, joints(3,:),'b');
legend('qHAA', 'qHFE', 'qKFE');

end