clear
clc
% Crear una matriz de 100x100 llena de unos
image = ones(100, 300);

% Establecer a 0 las primeras y últimas filas/columnas
image(:, 1) = 0.0;   % Primera columna
image(:, end) = 0.0;  % Última columna
image(1, :) = 0.0;    % Primera fila
image(end, :) = 0.0;  % Última fila

% Establecer un bloque central a 0 (de 25:75 en ambas dimensiones)
image(25:75, 25:75) = 0.0;

start_points = [6; 51];
end_points = [96; 200];

%%
clear
clc

img = imread('/home/alicia/despacho_2.pgm');
image = zeros(size(img));
image(img > 250) = 1;
se = strel('disk',4);
image = imopen(image,se);
imshow(image)

start_points = [100; 280];
end_points = [350; 160];

%% Primera aplicación del fast marching
W=FMdist(image');

%%

clc

%  FM para path global
options.nb_iter_max = Inf;
options.end_points = end_points;
options.Tmax = sum(size(W));
[D,S] = perform_fast_marching_2d(W, start_points, options);

%%

path = extract_path_2d(D,end_points, options);
path=flip(path);

%% Crear la figura y graficar

x = path(:, 1);
y = path(:, 2);

figure;
imshow(image);
hold on
plot(x, y, '-o');
hold off