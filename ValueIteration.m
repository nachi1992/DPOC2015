function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
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

for k = 1:NIteration
     J_k = J_opt;   
        
            for i=1:n_states
                for l = 1:n_input
                    U_cost(l)= G(i,l);

                    for j = 1:n_states
                        U_cost(l) = U_cost(l)+ P(i,j,l)*J_opt(j);
                    end
                end
                 min_u = U_cost((U_cost==min(U_cost)));
                 min_u_idx = find(U_cost==min(U_cost));
                J_opt(i) = min_u(1);  
                u_opt_ind(i) = min_u_idx(1);
            end
            
     if(norm((J_k-J_opt))<0.00001)   
         break;
     end
end
    

end

