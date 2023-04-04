function qLeg = getJointsForLegFromLeftForeJoints(qLf, legName)


if (legName == 'LF')
    qLeg = qLf;
elseif (legName == 'RF')
    qLeg = [-qLf(1); qLf(2); qLf(3)];
elseif (legName == 'LH')
    qLeg = [qLf(1); -qLf(2); -qLf(3)];
elseif (legName == 'RH')
    qLeg = [-qLf(1); -qLf(2); -qLf(3)];
end

end