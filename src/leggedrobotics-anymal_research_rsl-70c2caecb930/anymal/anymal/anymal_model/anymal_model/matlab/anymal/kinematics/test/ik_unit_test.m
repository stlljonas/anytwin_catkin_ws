clc;
legName = 'LF';

kin = loadKinematicParametersForLeg(legName);

B_r_BF_des = [0.2; 0.2; -0.60];

figure();

line([0 B_r_BF_des(1)], ...
     [0 B_r_BF_des(2)], ...
     [0 B_r_BF_des(3)], 'Color', 'k', 'LineWidth', 4);

q_cmp = computeLegJointsFromPositionBaseToFootInBaseFrame(B_r_BF_des, legName, 1, 1);
visualizeLeg(q_cmp, legName, 'k');
B_r_BF_cmp = computeLegPoseFromQ(q_cmp, legName);

disp(['p_des: ' mat2str(B_r_BF_des)]);
disp(['p_cmp: ' mat2str(B_r_BF_cmp)]);