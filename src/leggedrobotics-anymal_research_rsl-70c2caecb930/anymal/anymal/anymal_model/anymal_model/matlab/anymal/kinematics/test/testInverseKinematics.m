function testInverseKinematics()
%TESTINVERSEKINEMATICS Test the inverse kinematics for random
%configurations.

%   Author(s): C. Dario Bellicoso

legNames = cell(1,1);
legNames{1}.legName = 'LF';
legNames{1}.fore = 1;

legNames{2}.legName = 'RF';
legNames{2}.fore = 1;

legNames{3}.legName = 'LH';
legNames{3}.fore = 0;

legNames{4}.legName = 'RH';
legNames{4}.fore = 0;

numTests = 1000;

for nameIndex = 1:length(legNames)
    success = 1;
    
    legName = legNames{nameIndex}.legName;
    fore = legNames{nameIndex}.fore;
    
    for k=1:numTests
        q_expected(1) = 3*(rand(1,1) - 0.5);
        q_expected(2) = 2*(rand(1,1) - 0.5);
        q_expected(3) = 2*(rand(1,1) - 0.5);
        q_expected = q_expected(:);
        
        if (abs(q_expected(3)) < 0.0079)
            q_expected(3) = 0.008;
        end

        if (fore == 1)
            if (q_expected(3)>0)
                q_expected(3) = -q_expected(3);
            end
        else
            if (q_expected(3)<0)
                q_expected(3) = -q_expected(3);
            end
        end
        
        B_r_BF_expected = computeLegPoseFromQ(q_expected, legName);
        hipUp = 1;
        elbowUp = 1;
        q_computed = computeLegJointsFromPositionBaseToFootInBaseFrame(B_r_BF_expected, legName, hipUp, elbowUp);
        B_r_BF_computed = computeLegPoseFromQ(q_computed, legName);
        
        if (norm(q_expected-q_computed) > 1e-4 || isnan(norm(q_expected-q_computed)))
            disp(['IK failed for leg ' legName]);
            disp(['q expected: ' mat2str(q_expected)]);
            disp(['q computed: ' mat2str(q_computed)]);
            disp(['error: ' mat2str(q_expected-q_computed)]);
            success = 0;
            return;
        end
        if (norm(B_r_BF_expected-B_r_BF_computed) > 1e-10)
            disp(['IK failed for leg ' legName]);
            disp(['B_r_BF expected: ' mat2str(B_r_BF_expected)]);
            disp(['B_r_BF computed: ' mat2str(B_r_BF_computed)]);
            disp(['error: ' mat2str(B_r_BF_expected-B_r_BF_computed)]);
            success = 0;
        end
        
    end

    if (success)
        disp(['Inverse kinematics succeded for leg ' legName '.']);
    else
        disp(['Inverse kinematics failed for leg ' legName '.']);
    end
end

end