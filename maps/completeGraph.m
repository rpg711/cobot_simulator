function [V2 E2 A M D] = completeGraph(V,E,maxEdgeLength,W)
%completeGraph - Convert skeleton graph into complete graph
%              image im, scaling the Vertex locations by scaleX and scaleY
% Syntax:  [] = displayMap(V,E,im,scaleX,scaleY)
%
% Inputs:
%    V - Skeleton Graph vertices. Column 1 is the index number, columns 2
%         and 3 the X and Y coordinates respectively of the vertices
%    E - Skeleton Graph edges. The first two columns contain the indices 
%         of the vertices (in V) connected by the edge. The third column
%         (optional) contains the sequence number relating the WiFi data W.
%    W - (Optional) WiFi data. Column 1: Sequence number. Column 2: Vertex
%         number in the sequence
%    maxEdgeLength
%
% Outputs:
%    V2 - Vertices of the completed graph
%    E2 - Edges of the completed graph
%    A  - List of Access Points (APs)
%    M  - Mean WiFi signal strengths from every AP at every vertex
%    D  - S.D. of WiFi signal from every AP at every vertex
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: 

% Author: Joydeep Biswas
% Robotics Institute, Carnegie Mellon University
% email: joydeepb@cs.cmu.edu
% Feb 2010; 

%------------- BEGIN CODE --------------
V2 = [];
E2 = [];
visited = zeros(size(V,1),1);
if nargin<3 
    return  %need at least 3 parameters to execute succesfully
end
dynLength = 0;
processWifi = 0;
if nargin==4 
    if size(E,2)<3
        return  %need the sequence number for each edge
    else
        dynLength = 1;
    end
    processWifi = 1;
end
nv = size(V,1);
ne = size(E,1);
extendedDataExists = size(E,2)>3;

fprintf('Compiling list of Access Points...\n');
if processWifi==1
    A = unique(W(:,3));
    A = [(1:length(A))' A];
    relateV = [];
end

fprintf('Building complete graph...\nEdge ');
for i=1:ne
    fprintf('%d ',i);
    if mod(i,20)==0
        fprintf('\n');
    end
    a = E(i,1);
    b = E(i,2);
    v1 = V(a,2:3);
    v2 = V(b,2:3);
    l = norm(v1-v2);
    firstSkipped = 0;
    if dynLength==1
        %need to compute the edge length
        s = E(i,3);
        subW = W(W(:,1)==s,:);
        n = max(double(subW(:,2))) - 1; %num of smaller edges on this skeleton edge
        if visited(a)==1
            n = n+1; %the first vertex had been seen before, hence skipped
            firstSkipped = 1;
        end
        maxEdgeLength = sqrt((v1-v2)*(v1-v2)')/n;
        visited(a) = 1;
        visited(b) = 1;
    end
    if(extendedDataExists)
        extendedData = E(i,4:end);
    else
        extendedData = [];
    end
    if l>maxEdgeLength
        %need to subdivide edge
        n = ceil(l/maxEdgeLength);
        dv = (v2-v1)/n;
        k = 1;
        for j=0:(n-1)
            ev1 = v1+j*dv;
            ev2 = v1+(j+1)*dv;
            [ex1, e1] = exists(V2(:,2:end),ev1);
            [ex2, e2] = exists(V2(:,2:end),ev2);
            if ex1~=1
                V2 = [V2; 0 ev1];
                e1 = size(V2,1);
                if processWifi==1
                    if ~(firstSkipped==1 && k==1)
                        relateV(e1,:) = [i k];
                        k = k+1;
                    else
                        error('Correlation error')
                    end
                end
            end
            if ex2~=1
                V2 = [V2; 0 ev2];
                e2 = size(V2,1);
                if processWifi==1
                    relateV(e2,:) = [i k];
                    k = k+1;
                end
            end
            E2 = [E2;e1 e2 extendedData];
        end
    else
        %just add this edge as is
        ev1 = v1;
        ev2 = v2;
        [ex1, e1] = exists(V2(:,2:end),ev1);
        [ex2, e2] = exists(V2(:,2:end),ev2);
        k = 1;
        if ex1~=1
            V2 = [V2; 0 ev1];
            e1 = size(V2,1);
            if processWifi==1
                if ~(firstSkipped==1 && k==1)
                    relateV(e1,:) = [i k];
                    k = k+1;
                else
                    error('Correlation error')
                end
            end
        end
        if ex2~=1
            V2 = [V2; 0 ev2];
            e2 = size(V2,1);
            if processWifi==1
                relateV(e2,:) = [i k];
            end
        end
        E2 = [E2;e1 e2 extendedData];
    end
end
V2(:,1) = (1:size(V2,1))';
fprintf('\nIndexing Access Points...\n');
for i=1:length(W)
    W(i,3) = A(A(:,2)==W(i,3),1);
end
if processWifi==1
    %Populate the M and D matrices
    fprintf('Populating M and D...\nVertex ');
    M = zeros(length(V2),length(A));
    D = zeros(length(V2),length(A));
    for i=1:length(V2)
        fprintf('%d ',i);
        if mod(i,20)==0
            fprintf('\n');
        end
        subW = W(W(:,1)==relateV(i,1) & W(:,2)==relateV(i,2),3:end);
        subA = unique(subW(:,1));
        for j=1:length(subA)
            M(i,subA(j)) = mean(double(subW(subW(:,1)==subA(j),2)));
            D(i,subA(j)) = std(double(subW(subW(:,1)==subA(j),2)));
        end
    end
end
fprintf('\nDone!\n');