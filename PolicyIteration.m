function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
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
for i=1:n_states
    u_opt_ind(i) = randi(5); 
end

for k = 1:NIteration
     J_k = J_opt;   
    for i =1:n_states
        J_opt(i)= G(i,u_opt_ind(i));

        for j = 1:n_states
        J_opt(i) = J_opt(i)+ P(i,j,u_opt_ind(i))*J_opt(j);
        end
    end
        
            for i =1:n_states
                for l = 1:n_input
                U_cost(l)= G(i,l);

                    for j = 1:n_states
                        U_cost(l) = U_cost(l)+ P(i,j,l)*J_opt(j);
                    end
                end
                min_u = find(U_cost==min(U_cost));
                u_opt_ind(i) = min_u(1) ;    
            end
            
     if(norm((J_k-J_opt))<0.0000001)   
         break;
     end
end
    

end

