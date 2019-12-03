%  Code Credit (Functions):
%  Olivier Gougeon 2019
%  Leandro Lustosa 2017
%
%  Script By:
%  James Allen 2019
%
%  Refer to [1] for further information:
%
%  REFERENCES
%    [1] Lustosa L.R., Defay F., Moschetta J.-M., "The Phi-theory
%    approach to flight control design of tail-sitter vehicles"
%    @ http://lustosa-leandro.github.io

%% Setup Workspace

clean

%%  Variable Declarations

%   Equilibrium Values
%   -----------------------------------------------------------------------
%    vne     :   Equilibrium forward flight velocity             [m/s]
%    wwe     :   Equilibrium angular velocity of both propellers [rad/s]
%    dde     :   Equilibrium angular position of both elevons    [rad]
%    te      :   Equilibrium pitch angle of vehicle              [rad]
%    phie    :   Equilibrium bank/roll angle of vehicle          [rad]
%    wne     :   Equilibrium horizontal wind velocity component  [m/s]
%    he      :   Constant altitude, he = 0                       [m]
%    we      :   No wind, we = [0;0]                             [m/s]

%   INPUTS:
%   -----------------------------------------------------------------------
%    State def. : x = (vb wb q)                                in R(10x1)
%    Input def. : u = (w1 w2 d1 d2)                            in R( 4x1)

%   OUTPUTS:
%   -----------------------------------------------------------------------
%    Ac - State Matrix        :                                in R(9x9)                          
%    Bc - Input Matrix        :                                in R(9x4)
%    Cc - Output Matrix       :                                in R(9x9)
%    Dc - Feedthrough Matrix  :                                in R(9x4)

%   State Definitions:
%   -----------------------------------------------------------------------
%    vl      : Vehicle velocity in NED axis (m/s)             [3x1 Real]
%    wb      : Vehicle angular velocity in body axis (rad/s)  [3x1 Real]
%    q       : Quaternion attitude (MATLAB convention)        [4x1 Real]

%% Workspace Initialisation

mavionDroneOG = lon_drone();     % Generate longitudinal drone

%% 1. This section finds the equilibrium points for any speed of the MAVion 

vne = 0:30; % vne = 0 to 30m/s 
he  = 0;    % constant altitude 
wne = 0;    % assumed no horizontal wind velocity component

% Find pitch, propeller angular velocities and elevon angular positions 
te = zeros(1,length(vne));
wwe = zeros(1,length(vne));
dde = zeros(1,length(vne));
for i = 1:length(vne)
    [te(i),wwe(i),dde(i)] = lonTrim_vne(vne(i),he,wne,mavionDroneOG);
end

%% Results/Reality Check:

% To check the above results, Lustosa's Longitudinal Trim code will be
% used. This requires an input pitch angle instead of an input velocity, 
% allowing the above te value to be used in the hopes of returning a 
% forward velocity of 20m/s

mavionDroneLL = generateMavion();          % Generate LL drone

% Initialise Matrices
xts = zeros(10,2,length(vne));
uts = zeros(4,2,length(vne));
for i = 1:length(vne)
    [xts(:,:,i),uts(:,:,i),~] = longTrim(te(i),mavionDroneLL);  
end

vneCheck = xts(1,1,21);                    % vneCheck should yield 20m/s
    
% This confirms the parameters generated are the true values required to
% balance the system at a forward velocity of 20m/s (can also check with
% graphs from lon_ctrl.m from LL [1])

%% 2.1 This section linearises the whole system (Long + Lat Eq. of Motion)

% Linearises the nonlinear equations of motion with respect to a given 
% operation point of 20m/s during longitudinal motion

% Initialise Matrices
x = zeros(10,length(vne));
u = zeros(4,length(vne));
Ac = zeros(9,9,length(vne));
Bc = zeros(9,4,length(vne));
Cc = zeros(9,9,length(vne));
Dc = zeros(9,4,length(vne));
for i = 1:length(vne)
    x(:,i) = [xts(1:3,1,i);xts(4:6,1,i);xts(7:10,1,i)]; % State definitions
    u(:,i) = [-wwe(i);wwe(i);dde(i);dde(i)];            % Input definitions
    [Ac(:,:,i),Bc(:,:,i),Cc(:,:,i),Dc(:,:,i)] = jacob(x(:,i),u(:,i),...
        mavionDroneLL);   
end

%% 2.2 This section generates a pole map for the whole dynamical system

% Initialise Matrices
poles = zeros(9,length(vne));
for i = 1:length(vne)
    % Compute the eigenvalues of Ac to find poles of the system
    poles(:,i) = eig(Ac(:,:,i));    

    % Plot pole map (all poles)
    figure(1)
    hold on
    plot(real(poles(:,i)), imag(poles(:,i)), '*')
    xLabel = {"Real Axis (s$^{-1}$)"};          % Real Axis (s$^{-1}$)
    yLabel = {'Imaginary Axis (s$^{-1}$)'};     % Imaginary Axis (s$^{-1}$)
    topTitle = {'Pole Map'};                    % Pole Map of System
    xlabel(xLabel,'Interpreter','latex','FontSize',18);
    ylabel(yLabel,'Interpreter','latex','FontSize',18);
    title(topTitle,'Interpreter','latex','FontSize',18);
    sgrid
end

% The majority of the poles are stable, which makes sense seeing as the
% velocity of the MAVion is so high. Although 7 of the 9 poles are stable,
% 1 of them is unstable and 1 is marginally stable. This is likely caused 
% due to the Hybrid nature of the drone. This means even at a high 
% velocity, stability of the drone is still not ideal like a regular 
% aircraft. 

%% 3. Transfer functions between input and any output 

% Determine state-space model info (No. of outputs, inputs and states)
% All SS models are the same size!
size(ss(Ac(:,:,1),Bc(:,:,1),Cc(:,:,1),Dc(:,:,1))) 

%% Analysis

% Refer to report

%% Decoupling of Dynamics 

% After analysing the modes of the system, the state space model can be
% decoupled. This involves separating the longitudinal and lateral
% components from the original Ac, Bc, Cc and Dc matrices. The result of
% this decoupling is to determine whether the MAVion will need to be
% analysed as a full 6DOF model or if the longitudinal and lateral
% components can be considered completely independently. 

Alat = zeros(5,5,length(vne));
Blat = zeros(5,2,length(vne));
Clat = zeros(5,5,length(vne));
Dlat = zeros(5,2,length(vne));

Alon = zeros(4,4,length(vne));
Blon = zeros(4,2,length(vne));
Clon = zeros(4,4,length(vne));
Dlon = zeros(4,2,length(vne));

zLat = cell(5,2,length(vne));
zLon = cell(4,2,length(vne));

for i = 1:length(vne)
    % Lateral Decomposition
    Alat(:,:,i)=Ac([2 4 6 7 9],[2 4 6 7 9],i);    % Decoupled Ac matrix 
    Blat(:,:,i)=Bc([2 4 6 7 9],[1 3],i);          % Decoupled Bc matrix 
    Clat(:,:,i)=Cc([2 4 6 7 9],[2 4 6 7 9],i);    % Decoupled Cc matrix 
    Dlat(:,:,i)=Dc([2 4 6 7 9],[1 3],i);          % Decoupled Dc matrix 
    sysLat = ss(Alat(:,:,i),Blat(:,:,i),Clat(:,:,i),Dlat(:,:,i));
    zpk(sysLat)

    % Longitudinal Decomposition
    Alon(:,:,i)=Ac([1 3 5 8],[1 3 5 8],i);        % Decoupled Ac matrix 
    Blon(:,:,i)=Bc([1 3 5 8],[1 3],i);            % Decoupled Bc matrix 
    Clon(:,:,i)=Cc([1 3 5 8],[1 3 5 8],i);        % Decoupled Cc matrix 
    Dlon(:,:,i)=Dc([1 3 5 8],[1 3],i);            % Decoupled Dc matrix 
    sysLon = ss(Alon(:,:,i),Blon(:,:,i),Clon(:,:,i),Dlon(:,:,i));
    zpk(sysLon)
    
    [zLat{5,2,i},~,~] = zpkdata(sysLat); 
    [zLon{4,2,i},~,~] = zpkdata(sysLon);
end

%% Check Poles 

% The decoupled longitudinal and lateral poles can now be compared with the
% entire system's poles. If the correlation between these poles matches
% perfectly then the MAVion dynamics can be decoupled and analysed
% separately. 

lonPoles = zeros(4,1,length(vne));
latPoles = zeros(5,1,length(vne));
for i = 1:length(vne)
    damp(Ac(:,:,i));                      % Poles of entire system
    damp(Alon(:,:,i));                    % Poles of Longitudinal system
    damp(Alat(:,:,i));                    % Poles of Lateral system 
    
    lonPoles(:,:,i) = eig(Alon(:,:,i));   % Extract Lon poles for plotting
    latPoles(:,:,i) = eig(Alat(:,:,i));   % Extract Lat poles for plotting
end

% After analysing the poles for forward velocities ranging from vne=0:30
% m/s, it was shown that the system can be decoupled. Although this is the
% case, it can be noted that when vne<=4 m/s, the previously real dutch
% roll poles become a more standard complex conjugate pair. 

%% Comprehensive Pole-Zero Evolution (Longitudinal)

% Check for errors: either clear current figures or initialise new figure 
try
    clf(h)              
catch
    h = initFig('MAVion Longitudinal Dynamics',true,...
        'landscape','golden','painters'); 
end

% Set Properties 
set(h,'Position',[-1919 41 1920 963])
sp1 = subplot(1,2,1);
sp2 = subplot(1,2,2);
cbarOffset = 0.055;
set(sp1,'Position',[0.0589 0.0873 0.9255-cbarOffset 0.8766])
set(sp2,'Position',[0.5620-cbarOffset/2 0.1589 0.3969-cbarOffset 0.2904])
set([sp1,sp2],'NextPlot','add')

% Configure Figure 
colors = getColors('s');
fontSize = 20;
factorSize = 0.80;
tickSize = ceil(factorSize*fontSize);
markerSize = ceil(0.5*fontSize);
lineWidth = 1.5;

% Define line colors (i.e. the colormap)
[~,~,~,~,~,~,~,~,chosenCmap] = figureConfig();
cmap = brewermap(31,chosenCmap);

% Axes
axis(sp1,[-160 160 -45 45])
axis(sp2,[-0.5 2.5 -1.75 1.75])

% Add complex grid to plots
subplot(sp1); sgrid
subplot(sp2); sgrid

% Add labels
xLabel = {"Real Axis (s$^{-1}$)"};       % Real Axis (s$^{-1}$)
yLabel = {'Imaginary Axis (s$^{-1}$)'};  % Imaginary Axis (s$^{-1}$)
xlbl = xlabel(sp1,xLabel,'Interpreter','latex','FontSize',fontSize);
ylbl = ylabel(sp1,yLabel,'Interpreter','latex','FontSize',fontSize);

% Change tick font to LaTeX
set(sp1,'TickLabelInterpreter','latex','FontSize',tickSize)
set(sp2,'TickLabelInterpreter','latex','FontSize',ceil(0.65*tickSize))
sgrid_tick = findobj(sp1.Children,'Tag','CSTgridLines','Type','Text');
for kk = 1:length(sgrid_tick)
    set(sgrid_tick(kk),'Interpreter','latex','FontSize',tickSize)
end
sgrid_tick = findobj(sp2.Children,'Tag','CSTgridLines','Type','Text');
for kk = 1:length(sgrid_tick)
    set(sgrid_tick(kk),'Interpreter','latex','FontSize',ceil(0.65*tickSize))
end

% Title of the close-up
closeUpTitle = sprintf('Close-Up of Phugoid Mode (Longitudinal)');
title(sp2,{closeUpTitle},'Interpreter','latex','FontSize',...
    ceil(factorSize*fontSize*1.5))

for i = 1:length(vne)
    %Plot Poles:
    % Plot poles by keeping the same colors to see the evolution
    p1 = plot(sp1,real(lonPoles(:,:,i)),imag(lonPoles(:,:,i)),'x',...
        'Color',cmap(i,:),'MarkerSize',markerSize,'LineWidth',lineWidth,...
        'DisplayName','Longitudinal Poles'); % Longitudinal
  
    % Plot Zeros:
    latZeros = zLat{5,2,i};   % Access Zeros 
    lonZeros = zLon{4,2,i};   % Access Zeros
    
    % Important for zeros as they change for every transfer function
    for k1 = 1:2
        for k2=1:4
            lonZerosPlot = lonZeros{k2,k1};
            % Plot zeros by keeping the same colors to see the evolution
            p3 = plot(sp1,real(lonZerosPlot),imag(lonZerosPlot),'o',...
                'Color',cmap(i,:),'MarkerSize',markerSize,'LineWidth',...
                lineWidth,'DisplayName','Longitudinal Zeros');%Longitudinal 
        end
    end
     
    % Plot zoom-up of phugoid mode
    p1b = plot(sp2,real(lonPoles(3:4,:,i)),imag(lonPoles(3:4,:,i)),'x',...
        'Color',cmap(i,:),'MarkerSize',markerSize,'LineWidth',lineWidth,...
        'DisplayName','Phugo\"{\i}de'); % Phugoid
   
    % Prepare legend
    set(p1,'Tag','lgd')
    set(p3,'Tag','lgd')
            
    % Legend
    legend(sp1,[p1 p3],{},'Interpreter','latex','FontSize',fontSize,...
        'Orientation','Vertical','Position',[0.6751-cbarOffset 0.6420...
        0.1607 0.2061])
end

% Add colorbar
[fontSize,fontScale,fontInterpreter,tickSize,~,lineWidth,~,~] ...
    = figureConfig();
ax = sp1;
caxis(ax,[0,30])
colormap(ax,brewermap(31,chosenCmap))
cbar = colorbar(ax,'Location','manual','TickLabelInterpreter',...
    fontInterpreter,'FontSize',tickSize);
cbar.Label.FontSize = ceil(fontScale*fontSize);
cbar.Label.Interpreter = fontInterpreter;
cbar.Label.String = '${v_n}_e$ (m/s)';
drawnow
cbarSpacer = 0.02;
cbar.Position(1) = sum(ax.Position([1,3])) + cbarSpacer;
cbar.Position(2) = ax.Position(2);
cbar.Position(3) = cbarSpacer;
cbar.Position(4) = sum(ax.Position([2,4]))-ax.Position(2);

% Correct font sizes
xlbl.FontSize = ceil(factorSize*fontSize*1.5);
ylbl.FontSize = ceil(factorSize*fontSize*1.5);

saveas(gcf,'PoleEvolutionLongitudinal.png')

%% Comprehensive Pole-Zero Evolution (Lateral)

% Check for errors: either clear current figures or initialise new figure 
try
    clf(h1)              
catch
    h1 = initFig('MAVion Lateral Dynamics',true,...
        'landscape','golden','painters'); 
end

% Set Properties 
set(h1,'Position',[-1919 41 1920 963])
sp1 = subplot(1,2,1);
sp2 = subplot(1,2,2);
cbarOffset = 0.055;
set(sp1,'Position',[0.0589 0.0873 0.9255-cbarOffset 0.8766])
set(sp2,'Position',[0.5620-cbarOffset/2 0.1589 0.3969-cbarOffset 0.2904])
set([sp1,sp2],'NextPlot','add')

% Configure Figure 
colors = getColors('s');
fontSize = 20;
factorSize = 0.80;
tickSize = ceil(factorSize*fontSize);
markerSize = ceil(0.5*fontSize);
lineWidth = 1.5;

% Define line colors (i.e. the colormap)
[~,~,~,~,~,~,~,~,chosenCmap] = figureConfig();
cmap = brewermap(31,chosenCmap);

% Axes
axis(sp1,[-22 10 -5 5])
axis(sp2,[-0.5 2.5 -1.75 1.75])

% Add complex grid to plots
subplot(sp1); sgrid
subplot(sp2); sgrid

% Add labels
xLabel = {"Real Axis (s$^{-1}$)"};       % Real Axis (s$^{-1}$)
yLabel = {'Imaginary Axis (s$^{-1}$)'};  % Imaginary Axis (s$^{-1}$)
xlbl = xlabel(sp1,xLabel,'Interpreter','latex','FontSize',fontSize);
ylbl = ylabel(sp1,yLabel,'Interpreter','latex','FontSize',fontSize);

% Change tick font to LaTeX
set(sp1,'TickLabelInterpreter','latex','FontSize',tickSize)
set(sp2,'TickLabelInterpreter','latex','FontSize',ceil(0.65*tickSize))
sgrid_tick = findobj(sp1.Children,'Tag','CSTgridLines','Type','Text');
for kk = 1:length(sgrid_tick)
    set(sgrid_tick(kk),'Interpreter','latex','FontSize',tickSize)
end

for i = 1:length(vne)
    %Plot Poles:
    % Plot poles by keeping the same colors to see the evolution
    p2 = plot(sp1,real(latPoles(:,:,i)),imag(latPoles(:,:,i)),'+',...
        'Color',cmap(i,:),'MarkerSize',markerSize,'LineWidth',lineWidth,...
        'DisplayName','Lateral Poles'); % Lateral 
   
    for k3 = 1:2
        for k4=1:5
            latZerosPlot = latZeros{k4,k3};
            % Plot zeros by keeping the same colors to see the evolution
            p4 = plot(sp1,real(latZerosPlot),imag(latZerosPlot),'*',...
                'Color',cmap(i,:),'MarkerSize',markerSize,'LineWidth',...
                lineWidth,'DisplayName','Lateral Zeros'); % Lateral 
        end
    end

    % Prepare legend
    set(p2,'Tag','lgd')
    set(p4,'Tag','lgd')
            
    % Legend
    legend(sp1,[p2 p4],{},'Interpreter','latex','FontSize',fontSize,...
        'Orientation','Vertical','Position',[0.7455-cbarOffset 0.42...
        0.2 0.2061])
end

 delete(sp2)

% Add colorbar
[fontSize,fontScale,fontInterpreter,tickSize,~,lineWidth,~,~] ...
    = figureConfig();
ax = sp1;
caxis(ax,[0,30])
colormap(ax,brewermap(31,chosenCmap))
cbar = colorbar(ax,'Location','manual','TickLabelInterpreter',...
    fontInterpreter,'FontSize',tickSize);
cbar.Label.FontSize = ceil(fontScale*fontSize);
cbar.Label.Interpreter = fontInterpreter;
cbar.Label.String = '${v_n}_e$ (m/s)';
drawnow
cbarSpacer = 0.02;
cbar.Position(1) = sum(ax.Position([1,3])) + cbarSpacer;
cbar.Position(2) = ax.Position(2);
cbar.Position(3) = cbarSpacer;
cbar.Position(4) = sum(ax.Position([2,4]))-ax.Position(2);

% Correct font sizes
xlbl.FontSize = ceil(factorSize*fontSize*1.5);
ylbl.FontSize = ceil(factorSize*fontSize*1.5);

saveas(gcf,'PoleEvolutionLateral.png')
