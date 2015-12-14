function P = ComputeTransitionProbabilities( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace,
%   map, gate, mansion, cameras) computes the transition probabilities
%   between all states in the state space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 2)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       controlSpace:
%           A (L x 1)-matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map.
%           Positive values indicate cells that are inaccessible (e.g.
%           trees, bushes or the mansion) and negative values indicate
%           ponds or pools.
%
%   	gate:
%          	A (2 x 1)-matrix describing the position of the gate.
%
%    	mansion:
%          	A (F x 2)-matrix indicating the position of the cells of the
%           mansion.
%
%    	cameras:
%          	A (H x 2)-matrix indicating the positions of the cameras.
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

% put your code here
global M;
global N;
global H;
global space;
global n_states;
global camera;
global Mansion;
global Map;
global F;
space = stateSpace;
camera = cameras;
Mansion =mansion;
Map = map';
n_states = size(stateSpace,1);
n_input  =  size(controlSpace,1);
M = size(Map,1);
N = size(Map,2);
H = size(cameras,1);
F = size(mansion,1);
P = zeros(n_states,n_states,n_input);
for l=1:n_input
        for i = 1:n_states
            j = nextState(i,l);  
            if(l==5)
                    pc = mansion_detect_prob(j);
                    P(i,i,l)=1-pc;
            else
                    P(i,j,l)= 1;
             end
            p_det = cam_detect_prob(j);
            gate_state = cord2idx(gate(1),gate(2));
            if(p_det>0)
                
                P(i,gate_state,l)= p_det;
                if(i~= gate_state)
                P(i,j,l)= 1- p_det;
                end
            end 
               
                
        end
        
end


end

            
function j = nextState(i,l)
 [x,y] =  idx2cord(i);   
 global M;
 global N;
 global Map;
 switch l
     case 1
        y = y+1;
     case 2
        x = x-1;
     case 3
        y = y-1;
     case 4
        x = x+1;
     case 5  
 end
 j = cord2idx(x,y);
 if(x<1 || x>M || y<1 || y>N || Map(x,y)>0)
     j=i;
     display('out of bounds..adjusting');
 end
 
end

function [x,y] =  idx2cord(i)
global space;
   x = space(i,1);
   y = space(i,2);
end

function j =  cord2idx(x,y)
 % state = find((and(any(space == x, 2), any(space == y, 2)))==1);
 global space;
 global n_states;
 for j = 1:n_states
     if(space(j,1)==x && space(j,2)==y)
       return;
     end
 end
end

function p_det = cam_detect_prob(j)
    global H;
    global camera;
    global Map;
    [x,y]= idx2cord(j);   
    p_det = 0;
    p_a = zeros(H,1);
    p_det_u =1;
    for i=1:H
        camx = camera(i,1,:);
        camy = camera(i,2,:);
         ux = 0; uy=0;  
         
        if(x== camx)
           % display('cam_sight x');
            if(y<camy)
                inity = y;
                finy = camy;
            else 
                inity = camy;
                finy = y;
            end
             obj=0;
             for m=inity+1:finy-1
                 if(Map(x,m)>0)
                     obj= obj+1;
                 end
             end
            
            if(obj==0)
            %   if(p_det>0)
            %    p_det_u = p_det_u*p_det;
            %    ux =1;
            %    end;    
                p_a(i) = (camera(i,3,:)/abs(finy-inity));    
            
            end
        end
        
        if( y == camy)
           %  display('cam_sight y');
             if(x<camx)
                initx = x;
                finx = camx;
            else 
                initx = camx;
                finx = x;
             end
             obj= 0;
            
             for m=initx+1:finx-1
                 if(Map(m,y)>0)
                     obj= obj+1;
                 end
             end
            
            if(obj==0)
         %       if(p_det>0)
         %       p_det_u = p_det_u*p_det;
         %       uy =1;
         %      end;
                p_a(i) = (camera(i,3,:)/abs(finx-initx));
            end
        end
        
     %   if (ux==1 || uy == 1)
     %       p_det = p_det - p_det_u;
     %   end
     
         p_a = p_a((p_a>0));
         if(size(p_a)>0)
%             p_det = p_a;
%         elseif(size(p_a)==2)
%             p_det = p_a()
          p_det = probUnion(p_a);
         else
             p_det =0;
         end
                
    end
            
end

function pc = mansion_detect_prob(i)

global F;
global Mansion;
global Map;
global p_c;
global gamma_p;
  [x,y]= idx2cord(i);   
    pc = p_c;
    
    
    for i=1:F
        manx = Mansion(i,1,:);
        many = Mansion(i,2,:);
           
        if(x== manx)
           
            if(y<many)
                inity = y;
                finy = many;
            else 
                inity = many;
                finy = y;
            end
             obj=0;
             for m=inity+1:finy-1
                 if(Map(x,m)>0)
                     obj= obj+1;
                 end
             end
            
            if(obj==0)
             pc = max(p_c, gamma_p/abs(finy-inity));
            else
             pc=p_c;
            end
            
        end
        
        if( y == many)
           if(x<manx)
                initx = x;
                finx = manx;
            else 
                initx = manx;
                finx = x;
            end
             obj=0;
             for m=initx+1:finx-1
                 if(Map(m,y)>0)
                     obj= obj+1;
                 end
             end
            
            if(obj==0)
             pc = max(p_c, gamma_p/abs(finx-initx));
            else
             pc= p_c;
            end
        end
    end   
end

function prob = probUnion(probvec) 
  if (length(probvec) == 1)
    prob = probvec;
  else
    a = probvec(1);
    b = probUnion(probvec(2:end));
    prob = a + b - a*b;
  end
end
