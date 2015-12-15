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
%b = zeros(n_states*n_input,1);
%A = eye(n_states*n_input,n_states);
f = ones(1,n_states);

%j=1; 
%a = [1 1 1 1 1]';
%for i=1:n_states
%A(j:j+4,i)= a;
%j=j+5;
%end
A = eye(n_states);
for i = 1:n_states
    
       
       for l = 1:n_input
                    for j = 1:n_states
                        b(j) = G(i,l)+P(i,j,l)*J_opt(j);
                        
                    end
                  [J_opt] = linprog(f,A,b);
          
       end            
       
end             
                                     
end




                 
