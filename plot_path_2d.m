function plot_path_2d(W,S,path,start_points,end_points, options)

% plot_fast_marching_2d - plot the result of the fast marching.
%
%   plot_fast_marching_2d(W,S,path);
%
%   Copyright (c) 2004 Gabriel Peyré


options.null = 0;
if isfield(options, 'path_width')
    path_width = options.path_width;
else
    path_width = 3; %anchura del camino que se dibuja
end

if isfield(options, 'point_size')
    point_size = options.point_size;
else
    point_size = 3; % anchura del pto final en el dibujo
end


open_list = find( S==0 );
close_list = find( S==-1 );

hold on;
Z = W;
% Z(open_list) = 0;
% Z(close_list) = Z(close_list) - 0.2;
imagesc(Z');
axis image;
axis off;
% axis square;
set(gca,'box','on');
if nargin>=4 & ~isempty(start_points)
    if size(start_points,1)~=2
        start_points = start_points';
    end
    if size(start_points,1)~=2
        error('start_points must be of size 2xk');
    end
    for i=1:size(start_points,2)
	    plot(start_points(1,i), start_points(2,i), 'kx', 'MarkerSize', point_size);
    end
end
if nargin>=5 & ~isempty(end_points)
    if size(end_points,1)~=2
        start_points = start_points';
    end
    if size(end_points,1)~=2
        error('start_points must be of size 2xk');
    end
    for i=1:size(end_points,2)
        plot(end_points(1,i), end_points(2,i), 'k*', 'MarkerSize', point_size);
    end
end
if nargin>=3
    if ~iscell(path)
        path = {path};
    end
    for i=1:length(path)
        plot(path{i}(:,1), path{i}(:,2), '-', 'LineWidth', path_width);
    end
end
hold off;