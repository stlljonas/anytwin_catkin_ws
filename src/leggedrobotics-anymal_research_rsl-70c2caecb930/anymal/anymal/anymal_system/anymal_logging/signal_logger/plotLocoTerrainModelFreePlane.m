%% Terrain normal
%named_figure('terrain model free plane normal'), clf
h = figure();
set(h, 'Name', 'Terrain model free plane normal');
set(gcf, 'units', 'points', 'position', [plotX0, plotY0, plotWidth, plotHeight]);

subplot(3,1,1)
hold on
plot(logElem(idx_loco_terrainModel_normalInWorldFrame_x).time, logElem(idx_loco_terrainModel_normalInWorldFrame_x).data, 'b')

grid on
xlabel('logElem().time [s]')
ylabel('x [m]')
subplot(3,1,2)
hold on
plot(logElem(idx_loco_terrainModel_normalInWorldFrame_y).time, logElem(idx_loco_terrainModel_normalInWorldFrame_y).data, 'b')
grid on
xlabel('logElem().time [s]')
ylabel('y [m]')
subplot(3,1,3)
hold on
plot(logElem(idx_loco_terrainModel_normalInWorldFrame_z).time, logElem(idx_loco_terrainModel_normalInWorldFrame_z).data, 'b')

grid on
xlabel('logElem().time [s]')
ylabel('z [m]')


%% Terrain reference point position
%named_figure('terrain model free plane point'), clf
h = figure();
set(h, 'Name', 'Terrain model free plane reference point position');
set(gcf, 'units', 'points', 'position', [plotX0, plotY0, plotWidth, plotHeight]);

subplot(3,1,1)
hold on
plot(logElem(idx_loco_terrainModel_positionInWorldFrame_x).time, logElem(idx_loco_terrainModel_positionInWorldFrame_x).data, 'b')

grid on
xlabel('logElem().time [s]')
ylabel('x [m]')
subplot(3,1,2)
hold on
plot(logElem(idx_loco_terrainModel_positionInWorldFrame_y).time, logElem(idx_loco_terrainModel_positionInWorldFrame_y).data, 'b')
grid on
xlabel('logElem().time [s]')
ylabel('y [m]')
subplot(3,1,3)
hold on
plot(logElem(idx_loco_terrainModel_positionInWorldFrame_z).time, logElem(idx_loco_terrainModel_positionInWorldFrame_z).data, 'b')

grid on
xlabel('logElem().time [s]')
ylabel('z [m]')
