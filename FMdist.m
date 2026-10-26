function D= FMdist(W);
W = rescale( double(W) );
[row,col]=find(W<0.5);
start_points=[row,col]';
options.nb_iter_max = Inf;
options.Tmax = sum(size(W));
%disp('Performing front propagation.');
[D,S] = perform_fast_marching_2d(W, start_points, options);
D = rescale( double(D) );