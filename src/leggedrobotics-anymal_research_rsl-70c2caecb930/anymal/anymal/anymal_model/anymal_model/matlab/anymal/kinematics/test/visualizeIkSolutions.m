function visualizeIkSolutions(qLeftFore)
%VISUALIZEIKSOLUTIONS This function visualizes the 4 solutions obtained by
%the analytical inverse kinematics algorithm for each leg. The input is the
%vector of joints for the left fore leg.

%   Author(s): C. Dario Bellicoso

figure();
visualizeLegSolutions(getJointsForLegFromLeftForeJoints(qLeftFore, 'LF'), 'LF');
visualizeLegSolutions(getJointsForLegFromLeftForeJoints(qLeftFore, 'RF'), 'RF');
visualizeLegSolutions(getJointsForLegFromLeftForeJoints(qLeftFore, 'LH'), 'LH');
visualizeLegSolutions(getJointsForLegFromLeftForeJoints(qLeftFore, 'RH'), 'RH');

end

function visualizeLegSolutions(q, legName)

B_r_BF_des = computeLegPoseFromQ(q, legName);
line([0 B_r_BF_des(1)], ...
     [0 B_r_BF_des(2)], ...
     [0 B_r_BF_des(3)], 'Color', 'k', 'LineWidth', 4);

q_hipUpElbowDown = computeLegJointsFromPositionBaseToFootInBaseFrame(B_r_BF_des, legName, 1, 1);
q_hipDownElbowDown = computeLegJointsFromPositionBaseToFootInBaseFrame(B_r_BF_des, legName, 0, 1);
q_hipUpElbowUp = computeLegJointsFromPositionBaseToFootInBaseFrame(B_r_BF_des, legName, 1, 0);
q_hipDownElbowUp = computeLegJointsFromPositionBaseToFootInBaseFrame(B_r_BF_des, legName, 0, 0);
visualizeLeg(q_hipUpElbowDown, legName, 'b');
visualizeLeg(q_hipDownElbowDown, legName, 'r');
visualizeLeg(q_hipUpElbowUp, legName, 'g');
visualizeLeg(q_hipDownElbowUp, legName, 'c');

end