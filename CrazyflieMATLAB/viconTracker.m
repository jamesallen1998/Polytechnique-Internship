%% Setup Workspace

clean

%% Read CSV File from VICON Tracker

% CSV after recording and loading trial in VICON Tracker 
viconPos1 = csvread('sequence1.csv', 5, 8);
viconPos2 = csvread('sequence2.csv', 5, 8);

%% Plot x, y, z data for visualisation (Crazyflie Flight Path)

% Shift coordinates to centre 
originx1 = viconPos1(1,4);
originy1 = viconPos1(1,5);
originx2 = viconPos2(1,4);
originy2 = viconPos2(1,5);

% Plot trajectories of Crazyflie (this test is of a simple circle)
plot3((viconPos1(:,4) - originx1)/1000, (viconPos1(:,5) - originy1)/1000, ...
    viconPos1(:,6)/1000,'LineWidth',1.25)
hold on
plot3((viconPos2(:,4) - originx2)/1000, (viconPos2(:,5) - originy2)/1000, ...
    viconPos2(:,6)/1000, 'LineWidth',1.25)
topTitle = {'Crazyflie Vicon Flight Trajectory'};  
xLabel = {"Y Axis (m)"};              
yLabel = {'X Axis (m)'};    
zLabel = {'Z Axis (m)'};    
xlabel(xLabel,'Interpreter','latex','FontSize',12);
ylabel(yLabel,'Interpreter','latex','FontSize',12);
zlabel(zLabel,'Interpreter','latex','FontSize',12);
title(topTitle,'Interpreter','latex','FontSize',12);

