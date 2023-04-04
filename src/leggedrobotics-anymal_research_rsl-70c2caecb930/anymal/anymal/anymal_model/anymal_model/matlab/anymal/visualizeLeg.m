function visualizeLeg(q, legName, lineColor)
%VISUALIZELEG Plots the configuration of a leg given a vector of joints and
%a string identifier.

%   Author(s): C. Dario Bellicoso

kin_leg = getKinematicsForLeg(q, legName);

T_BH = kin_leg.T_BH;
T_BT = T_BH*kin_leg.T_HT;
T_BS = T_BT*kin_leg.T_TS;
T_BF = T_BS*kin_leg.T_SF;

cylinder_scale = [0.3; 0.3; 0.06];
cylinder_radius = 0.05;
cylinder_points = 10;

ellipsoid_points = 10;

h = gca();
axis equal; hold on;
visualizeHomogeneousTransform(eye(4), h);
[x,y,z] = ellipsoid(0, 0, 0, ...
                    0.2,0.05,0.05, ellipsoid_points);

hBase = surface(x, y, z);
tBase = hgtransform('Parent',h);
set(hBase,'Parent',tBase);
set(tBase,'Matrix', eye(4));


visualizeHomogeneousTransform(T_BH, h);

[x,y,z] = cylinder(cylinder_radius, cylinder_points);
hHaa = surface(cylinder_scale(1)*x, ...
               cylinder_scale(2)*y, ...
               cylinder_scale(3)*(z-0.5),'FaceColor','red');

tHaa = hgtransform('Parent',h);
set(hHaa,'Parent',tHaa);
set(tHaa,'Matrix', T_BH*makehgtform('yrotate',pi/2));

visualizeHomogeneousTransform(T_BT, h);
[x,y,z] = cylinder(cylinder_radius, cylinder_points);
hHfe = surface(cylinder_scale(1)*x, ...
               cylinder_scale(2)*y, ...
               cylinder_scale(3)*(z-0.5),'FaceColor','red');
tHfe = hgtransform('Parent',h);
set(hHfe,'Parent',tHfe);
set(tHfe,'Matrix', T_BT*makehgtform('xrotate',pi/2));

visualizeHomogeneousTransform(T_BS, h);
[x,y,z] = cylinder(cylinder_radius, cylinder_points);
hKfe = surface(cylinder_scale(1)*x, ...
               cylinder_scale(2)*y, ...
               cylinder_scale(3)*(z-0.5),'FaceColor','red');
tKfe = hgtransform('Parent',h);
set(hKfe,'Parent',tKfe);
set(tKfe,'Matrix', T_BS*makehgtform('xrotate',pi/2));

visualizeHomogeneousTransform(T_BF, h);
[x,y,z] = ellipsoid(0.02, 0, 0, ...
                    0.08,0.05,0.02, ellipsoid_points);

hFoot = surface(x, y, z);
tFoot = hgtransform('Parent',h);
set(hFoot,'Parent',tFoot);
set(tFoot,'Matrix', T_BF);

line_width = 2;

line([0 T_BH(1,4)], ...
     [0 T_BH(2,4)], ...
     [0 T_BH(3,4)], 'Color', lineColor, 'LineWidth', line_width);
 
line([T_BH(1,4) T_BT(1,4)], ...
     [T_BH(2,4) T_BT(2,4)], ...
     [T_BH(3,4) T_BT(3,4)], 'Color', lineColor, 'LineWidth', line_width);

line([T_BT(1,4) T_BS(1,4)], ...
     [T_BT(2,4) T_BS(2,4)], ...
     [T_BT(3,4) T_BS(3,4)], 'Color', lineColor, 'LineWidth', line_width);

line([T_BS(1,4) T_BF(1,4)], ...
     [T_BS(2,4) T_BF(2,4)], ...
     [T_BS(3,4) T_BF(3,4)], 'Color', lineColor, 'LineWidth', line_width);

view([160 23]);


end