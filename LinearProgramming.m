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
J_opt = zeros(n_states,1);
u_opt_ind = zeros(n_states,1);
U_cost = zeros(n_input,1);
f = ones(1,n_states);

b= [] ; 
p_x = [];

            for l = 1:n_input
                 b = [b ; G(:,l)]; 
                  p_x = [p_x ; P(:,:,l)];
            end
            
            A = repmat(eye(n_states),n_input,1) - p_x;
       
            for i = 1:n_states
                  [J_opt] = linprog(-f,A,b);      
            end          
   

     for i = 1:n_states
        [~,u_opt_ind(i)] = min(G(i,:) + sum(squeeze(P(i,:,:)) .* repmat(J_opt,1,n_input)));
    end
                                     
end




                 
