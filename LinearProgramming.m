function [ J_opt, u_opt_ind ] = LinearProgramming( P, G )
%LINEARPROGRAMMING Value iteration
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space.

% put your code here

global n_states n_input;
NIteration = 10000;
J_opt = zeros(n_states,1);
u_opt_ind = zeros(n_states,1);
U_cost=zeros(n_input,1);


for i=1:n_states
    
                for l = 1:n_input
                    U_cost(l)= G(i,l);

                    for j = 1:n_states
                        U_cost(l) = U_cost(l)+ P(i,j,l)*J_opt(j);
                    end
                end 
                [J_opt(i)] = linprog(1,-1,U_cost(l));
end
%f = ones(1, n_states);
%A = ones(n_states,1);
%[u_opt_ind,J_opt] = linprog(-f,A,min_u(1));
end




                 
