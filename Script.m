% Script.m
%
% Matlab script that calls all the functions for computing the optimal cost
% and policy of the given problem.
%
% Dynamic Programming and Optimal Control
% Fall 2015
% Programming Exercise
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Robin Ritz
% rritz@ethz.ch
%
% --
% Revision history
% [ 09.11.2015, RR ]    first version
%

%% clear workspace and command window
clear all;
close all;
clc;

%% define global problem parameters
% These variables may be used in your implementations. Do not add any
% additional global variable here for your implementations!
global p_c gamma_p pool_num_time_steps detected_additional_time_steps
p_c = 0.01;                         % probability of winning if randomly taking a picture
gamma_p = 0.5;                      % paparazzi camera strength
pool_num_time_steps = 4;            % number of time steps that pass if moving into pool or pond
detected_additional_time_steps = 8;	% additional time steps if detected and brought to gate

%% define problem size and generate map
disp('generate map');
shouldGenerateMap = false;
if shouldGenerateMap
	mapSize = [ 10, 15 ]; % [N, M]
	[ map, gate, mansion, cameras ] = GenerateMap( ...
        mapSize( 1 ), mapSize( 2 ) );
    % This generates a new random map.
else
    load( 'exampleMap.mat' );
    % In order to save time we can just load a pre-generated map.
end
PlotMap( 1, mapSize, map, gate, mansion, cameras );
%PlotMap3( 2, mapSize, map, gate, mansion, cameras );

%% generate state space
disp('generate state space');
stateSpace = [];
for n = 1 : size( map, 2 )
    for m = 1 : size( map, 1 )
        if map( m, n ) <= 0
            stateSpace = [ stateSpace; n, m ];
        end
    end
end
% This generates a (K x 2)-matrix 'stateSpace', where each row represents
% an accessible cell, i.e. an element of the state space.

%% generate control space
disp('generate control space');
controlSpace = [ 'n'; 'w'; 's'; 'e'; 'p' ];
% This defines all possible control actions.
% 'n' -> move north (i.e. move one field in positive y-direction)
% 'w' -> move west (i.e. move one field in negative x-direction)
% 's' -> move south (i.e. move one field in negative y-direction)
% 'e' -> move east (i.e. move one field in positive x-direction)
% 'p' -> take picture (i.e. stay at the same field and try to take a picture)

%% compute transition probabilities
if true % Change this to true to test your implementation
    disp('compute transition probabilities');
    P = ComputeTransitionProbabilities( stateSpace, controlSpace, ...
        map, gate, mansion, cameras );
    % This computes the transition probabilities between all states in the
    % state space for all control inputs.
    % The transition probability matrix has the dimension (K x K x L), i.e.
    % the entry P(i, j, l) representes the transition probability from state i
    % to state j if control input l is applied.
end

%% compute stage costs
if true % Change this to true to test your implementation
    disp('compute stage costs');
    G = ComputeStageCosts( stateSpace, controlSpace, ...
        map, gate, mansion, cameras );
    % This computes the stage costs for all states in the state space for all
    % control inputs.
    % The stage cost matrix has the dimension (K x L), i.e. the entry G(i, l)
    % represents the cost if we are in state i and apply control input l.
end

%% solve stochastic shortest path problem
valueIterationImplemented = false; % Change this to true to test your implementation
if valueIterationImplemented
    disp('solve stochastic shortest path problem with Value Iteration');
    [ J_opt_vi, u_opt_ind_vi ] = ValueIteration( P, G );
end

policyIterationImplemented = true; % Change this to true to test your implementation
if policyIterationImplemented
    disp('solve stochastic shortest path problem with Policy Iteration');
    [ J_opt_pi, u_opt_ind_pi ] = PolicyIteration( P, G );
end

linearProgrammingImplemented = false; % Change this to true to test your implementation
if linearProgrammingImplemented
    disp('solve stochastic shortest path problem with Linear Programming');
    [ J_opt_lp, u_opt_ind_lp ] = LinearProgramming( P, G );
end

% Here we solve the stochastic shortest path problem by Value Iteration,
% Policy Iteration, and Linear Programming.

%% plot results
disp('plot results');
if valueIterationImplemented
    figH2 = PlotMap( 3, mapSize, map, gate, mansion, cameras, stateSpace, ...
            J_opt_vi, u_opt_ind_vi );
    figure(figH2);
    title(strcat('Value Iteration (width=', num2str(mapSize(1)), ', height=', num2str(mapSize(2)), ')'));
    figH3 = PlotMap3( 4, mapSize, map, gate, mansion, cameras, ...
            stateSpace, J_opt_vi, u_opt_ind_vi );
    figure(figH3);
    title(strcat('Value Iteration (width=', num2str(mapSize(1)), ', height=', num2str(mapSize(2)), ')'));
end

if policyIterationImplemented
    figH2 = PlotMap( 5, mapSize, map, gate, mansion, cameras, ...
            stateSpace, J_opt_pi, u_opt_ind_pi );
    figure(figH2);
    title(strcat('Policy Iteration (width=', num2str(mapSize(1)), ', height=', num2str(mapSize(2)), ')'));
    figH3 = PlotMap3( 6, mapSize, map, gate, mansion, cameras, ...
            stateSpace, J_opt_pi, u_opt_ind_pi );
    figure(figH3);
    title(strcat('Policy Iteration (width=', num2str(mapSize(1)), ', height=', num2str(mapSize(2)), ')'));
end

if linearProgrammingImplemented
    figH2 = PlotMap( 7, mapSize, map, gate, mansion, cameras, ...
            stateSpace, J_opt_lp, u_opt_ind_lp );
    figure(figH2);
    title(strcat('Linear Programming (width=', num2str(mapSize(1)), ', height=', num2str(mapSize(2)), ')'));
    figH3 = PlotMap3( 8, mapSize, map, gate, mansion, cameras, ...
            stateSpace, J_opt_lp, u_opt_ind_lp );
    figure(figH3);
    title(strcat('Linear Programming (width=', num2str(mapSize(1)), ', height=', num2str(mapSize(2)), ')'));
end

%% display that terminated
disp('terminated');
