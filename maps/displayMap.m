function displayMap(V,E,im,scaleX,scaleY,origin)
%displayMap - Display the graph with Vertices V and edges E on background
%              image im, scaling the Vertex locations by scaleX and scaleY
% Syntax:  [] = displayMap(V,E,im,scaleX,scaleY)
%
% Inputs:
%    V - Resized to Nx3 array for nV vertices. Column 1 is the index number, columns 2
%         and 3 the X and Y coordinates respectively of the vertices
%    E - Resized to Mx2 array for nE edges. The two columns contain the indices of the
%         vertices (in V) connected by the edge
%
% Outputs:
%    none
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: plot, line, scatter

% Author: Joydeep Biswas
% Robotics Institute, Carnegie Mellon University
% email: joydeepb@cs.cmu.edu
% Nov 2009; 

%------------- BEGIN CODE --------------
if ~isempty(V)
    V = V(:,1:3);
    if ~isempty(origin)
        V(:,2:3) = V(:,2:3)+repmat(origin,length(V),1);
    end
end
if ~isempty(E)
    E = E(:,1:2);
end
nE = length(E);
%figure(1); 
clf; 
if ~isempty(im)
    imagesc(im); 
end 
axis equal; colormap gray;
zoom(1.75);
if ~isempty(E)
    i = E';
    i = i(:);
    X = V(i,2)*scaleX;
    Y = V(i,3)*scaleY; 
    X = reshape(X,2,nE);
    Y = reshape(Y,2,nE);
    hold on; plot(X,Y,'r','linewidth',2.5); hold off;
end
if ~isempty(V)
    hold on; 
    scatter(V(:,2)*scaleX,V(:,3)*scaleY,'xb',...
        'sizedata',200,'linewidth',2); 
    hold off;
    text(V(:,2)*scaleX+2,V(:,3)*scaleY+2,num2str(V(:,1)),'color','b');
end

%------------- END OF CODE --------------