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
%    psie    :   Equilibrium yaw angle of vehicle                [rad]
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

% Change vne to check each speed
vne = 30;  % vne = 20m/s but can be easily altered to a range of velocities
he = 0;    % constant altitude 
wne = 0;   % assumed no horizontal wind velocity component

% Find pitch, propeller angular velocities and elevon angular positions 
[te,wwe,dde] = lonTrim_vne(vne,he,wne,mavionDroneOG);

%% Known Equilibrium Points due to Longitudinal motion

ba = 0;      % no bank angle for this motion
psie = 0;    % no yaw during longitudinal flight

%% Results/Reality Check:

% To check the above results, Lustosa's Longitudinal Trim code will be
% used. This requires an input pitch angle instead of an input velocity, 
% allowing the above te value to be used in the hopes of returning a 
% forward velocity of 20m/s

mavionDroneLL = generateMavion();          % Generate LL drone
[xts,uts,~] = longTrim(te,mavionDroneLL);  
vneCheck = xts(1,1);                       % vneCheck should yield 20m/s

% This confirms the parameters generated are the true values required to
% balance the system at a forward velocity of 20m/s (can also check with
% graphs from lon_ctrl.m from LL [1])

%% 2.1 This section linearises the whole system (Long + Lat Eq. of Motion)

% Linearises the nonlinear equations of motion with respect to a given 
% operation point of 20m/s during longitudinal motion

x = [xts(1:3,1);xts(4:6,1);xts(7:10,1)];     % State definitions
u = [-wwe;wwe;dde;dde];                      % Input definitions
[Ac,Bc,Cc,Dc] = jacob(x,u,mavionDroneLL);    % Linearisation around 20m/s

%% 2.2 This section generates a pole map for the whole dynamical system

% Compute the eigenvalues of Ac to find poles of the system
poles = eig(Ac);    

% The majority of the poles are stable, which makes sense seeing as the
% velocity of the MAVion is so high. Although 7 of the 9 poles are stable,
% 1 of them is unstable and 1 is marginally stable. This is likely caused 
% due to the Hybrid nature of the drone. This means even at a high 
% velocity, stability of the drone is still not ideal like a regular 
% aircraft. 

%% 3. Transfer functions between input and any output 

% Determine state-space model info (No. of outputs, inputs and states)
size(ss(Ac,Bc,Cc,Dc))

%% Different Transfer Functions Explained 

% As the following is a MIMO system (Multiple Input, Multiple Output), this
% results in a R(9x4) matrix of transfer functions. For this solution, 
% there are 36 transfer functions (Input x Output). Each input consists of 
% a single and unique transfer function for every output of the system:

%   Transfer Functions (Output/Input):
%   -----------------------------------------------------------------------
%    Inputs      : w1, w2, d1, d2                          
%    Outputs     : vb1, vb2, vb3, wb1, wb2, wb3, psi1, psi2, psi3

% i.e. vb1/w1, vb2/w1, vb3/w1, wb1/w1, wb2/w1... etc

% N.B. Outputs = States seeing as Dc=0 and Cc=I --> Outputs=Cc*Dx

%% Find Transfer Functions 

% This transfer function generates the transfer functions without pole zero
% cancellations
numInputs = 4;
numOutputs = length(poles);
for i = 1:numInputs
    [num,den] = ss2tf(Ac,Bc,Cc,Dc, i);
    for j = 1:numOutputs
        sysTF1 = tf(num(j,:),den);
        sysNoCancel = zpk(sysTF1); % Print transfer functions 
    end
end

%% Pole-Zero Cancellation (Transfer Function Simplification)

% It can be seen from the above transfer functions that there are a number
% of poles and zeros that can be cancelled. For this reason, the code below
% was used to both simplify these transfer functions (through cancellation)
% and also to present it in a more readable format. 

% Specify state-space model as an LTI object 
inputs = {'w1' 'w2' 'd1' 'd2'};
 
% Linear velocities, angular velocities and attitudes (yaw, pitch and roll)
outputs = {'vb1' 'vb2' 'vb3' 'wb1' 'wb2' 'wb3' 'psi1' 'psi2' 'psi3'};

% Creates state-space model object representing the continuous time model
sysMIMO = ss(Ac,Bc,Cc,Dc,...
         'inputname',inputs,...
         'outputname',outputs);
     
sysTF2 = tf(sysMIMO);   % compute ALL transfer functions
sysCancel = zpk(sysTF2) % Print transfer functions 

%% vb1/w1: Purely Longitudinal Motion from MAVion

% Analyse complex conjugate pole (to determine short or phugoid mode)
analyse1 = roots([1 19.33 843.4]);                 % Roots
realPole1 = abs(real(analyse1(1)));                % Real pole
imagPole1 = abs(imag(analyse1(1)));                % Imaginary pole

natFreq1 = sqrt(realPole1^2 + imagPole1^2);        % Natural frequency
dampingRatio1 = realPole1/natFreq1;                % Damping ratio
period1 = 2*pi/(natFreq1*sqrt(1-dampingRatio1^2)); % Period [s]

% Analyse complex conjugate pole (to determine short or phugoid mode)
analyse2 = roots([1 0.5188 0.4111]);               % roots
realPole2 = abs(real(analyse2(1)));                % real pole
imagPole2 = abs(imag(analyse2(1)));                % imaginary pole

natFreq2 = sqrt(realPole2^2 + imagPole2^2);        % Natural frequency
dampingRatio2 = realPole2/natFreq2;                % damping ratio
period2 = 2*pi/(natFreq2*sqrt(1-dampingRatio2^2)); % period [s]

%% Lateral (Unsure seeing as there appears to be no dutch roll mode)

% Spiral Mode
halfSpiral = log(2)/0.001676;                      % [s] (easy to control?)

%% Analysis

% Refer to report

%% Separated Longitudinal and Lateral Pole Map

% After analysing the system, it is now possible to recreate the previous
% pole map with the lateral and longitudinal dynamics being separated. The
% longitudinal poles have been represented by an '*' and the lateral poles 
% have been represented by an 'o'.

plot(real(poles(1:4)), imag(poles(1:4)), 'b*')  % Longitudinal poles
hold on
plot(real(poles(5:9)), imag(poles(5:9)), 'ro')  % Lateral poles
xLabel = {"Real Axis (s$^{-1}$)"};              % Real Axis (s$^{-1}$)
yLabel = {'Imaginary Axis (s$^{-1}$)'};         % Imaginary Axis (s$^{-1}$)
topTitle = {'Pole Map (Longitudinal and Lateral)'};  
xlabel(xLabel,'Interpreter','latex','FontSize',18);
ylabel(yLabel,'Interpreter','latex','FontSize',18);
title(topTitle,'Interpreter','latex','FontSize',18);
sgrid
hold on

%% Decoupling of Dynamics 

% After analysing the modes of the system, the state space model can be
% decoupled. This involves separating the longitudinal and lateral
% components from the original Ac, Bc, Cc and Dc matrices. The result of
% this decoupling is to determine whether the MAVion will need to be
% analysed as a full 6DOF model or if the longitudinal and lateral
% components can be considered completely independently. 

% Lateral Decomposition
Alat=Ac([2 4 6 7 9],[2 4 6 7 9]);    % Decoupled Ac matrix 
Blat=Bc([2 4 6 7 9],[2 4]);          % Decoupled Bc matrix 
Clat=Cc([2 4 6 7 9],[2 4 6 7 9]);    % Decoupled Cc matrix 
Dlat=Dc([2 4 6 7 9],[2 4]);          % Decoupled Dc matrix 
sys_lat = ss(Alat,Blat,Clat,Dlat);   % State space model
zpk(sys_lat);

% Longitudinal Decomposition
Alon=Ac([1 3 5 8],[1 3 5 8]);        % Decoupled Ac matrix 
Blon=Bc([1 3 5 8],[2 4]);            % Decoupled Bc matrix 
Clon=Cc([1 3 5 8],[1 3 5 8]);        % Decoupled Cc matrix 
Dlon=Dc([1 3 5 8],[2 4]);            % Decoupled Dc matrix 
sys_lon = ss(Alon,Blon,Clon,Dlon);   % State space model
zpk(sys_lon);

%% Check Poles 

% The decoupled longitudinal and lateral poles can now be compared with the
% entire system's poles. If the correlation between these poles matches
% perfectly then the MAVion dynamics can be decoupled and analysed
% separately. 

damp(Ac)                             % Poles of entire system
damp(Alon)                           % Poles of Longitudinal system
damp(Alat)                           % Poles of Lateral system 

% After analysing the poles for forward velocities ranging from vne=0:30
% m/s, it was shown that the system can be decoupled. Although this is the
% case, it can be noted that when vne<=4 m/s, the previously real dutch
% roll poles become a more standard complex conjugate pair. 

%% Results 

% For all of the vne values ranging anywhere from 0m/s to 30m/s, decoupling
% is possible. All longitudinal poles appear to remain unchanged no matter
% what velocity the MAVion is travelling at, and the lateral poles all
% appear to remain close enough that decoupling would provide accurate
% results. The only pole that varies for all velocities is the extremely
% small real lateral pole. As this is of such a small degree though, this
% variation would provide little to no effect on the system allowing
% decoupling to be an accurate way of analysing each set of dynamics
% independently. 

%% Convert Euler Angles to Quaternions 

% After the linearisation of the system through the use of the Jacobian
% operation, the state quaternions are converted to euler angles for a more
% convenient representation of the state space model. With the deviation in
% these angle states being represented as roll, pitch and yaw, a simple
% operation can be undertaken to convert these back to quaternions. 

% Below is an example of how each set of euler angle deviations can be
% converted to the set of quaternion deviations:

eulDeviation = [0 1.5299 0];            % 3 euler angles (roll, pitch, yaw)
quatDeviation = eul2quat(eulDeviation); % 4 quaternion deviations
