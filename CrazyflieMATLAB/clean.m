function clean()
% Clean Function made by Olivier Gougeon
 
warning('off','all')
close all force;
fclose all;
% bdclose all; % Close all Simulink system windows unconditionally
clear mex;
evalin('base','clear all');
evalin('base','clear classes');
clear java;
clc;
warning('on','all')
 
end