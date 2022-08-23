% clear all
clc
addpath(genpath('DATA'));

% GM-VOLT DRIVELINE
% PARAM.-----------------------------------------------------------------
r = 0.3170;	% Effective Wheel Radius 0.317 [m] (RHEVE2011)
hpg = 2.24;	% Planetary gear ratio (VOLTEC ELECTRIC DRIVE SYSTEM OVERVIEW)
h = 2.24;	% Planetary gear ratio (VOLTEC ELECTRIC DRIVE SYSTEM OVERVIEW)
io = 2.16;	% Bevel gear and differential ratio (VOLTEC ELECTRIC DRIVE SYSTEM OVERVIEW)
mv = 1700;	% Overall vehicle+battery+driver mass [kg] (GM-Volt performance Simulation)

% GEAR EFFICIENCY INIT. -----------------------------------
load('pgsg.txt')
load('pgkor.txt')

% -----------------GM-VOLT MACHINES INIT.------------------
% --------------MAX Torque characteristics-----------------
load('tuice.txt')
load('tuicemin.txt')
load('tumg1.txt')
load('tumg2.txt')

% --------Efficiency and fuel consumption Maps-------------
load icemft.mat
load mg1eff.mat
load mg2eff.mat

% BATTERY INIT. [Taken from DSCC2009-2745 + VOLT Battery Info]
load('batsoc.txt');	% Uoc(SoC)
Rch = 0.056128; 	% [ohm]
Rdch = 0.115904;	% [ohm]
Qbat = 45*3600;		% [As] (45 Ah)
Ssoc = 0.95;		% [Start SoC]
%----------------------------------------------------------

% ----------- Drive Cycle and Driver -------------------------
dc = load('t-tuL-wL.txt');     % dc(t-vrijeme, tuL - moment koji daje vozac, wuL - brzina koju daje vozac)
uv = load('t-u1-u2-u3.txt');   % uv(t-vrijeme, u1 - tueOpt, u2 - wmg1Opt, u3 - mode)

m2km = 1.6093; % Kilometres vs. miles







