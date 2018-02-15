function [e, ind] = exists(M,V,t)
%exists - Check for existence of row vector V in matrix M
%
% Syntax:  [e, ind] = exists(M,V)
%
% Inputs:ev1
%    M - NxD array of D-dimension values
%    V - 1xD array to check for presence of in M
%    t - Threshold for checking equality
%
% Outputs:
%    e - 1 if V exists in M, 0 otherwise
%    ind - column vector containing row indices of M that correspond to V 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: find

% Author: Joydeep Biswas
% Robotics Institute, Carnegie Mellon University
% email: joydeepb@cs.cmu.edu
% Nov 2009; 

%------------- BEGIN CODE --------------

if isempty(M) || isempty(V) || size(M,2)~=size(V,2) || size(V,1)~=1
    e = 0;
    ind = [];
    return;
end
if nargin<3
    t = 1;
end
D = size(M,2);
N = size(M,1);
M2 = repmat(V,N,1);
e = sum(abs(M-M2)<t,2);
ind = find(e==D);
e = sum(e==D)>0;

%------------- END OF CODE --------------