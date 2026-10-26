% CARGA DEL MAPA PARA LOCALIZACION

function [mapmin,mapmax]=lim_mapa(mapa,rejilla)
% 
%Creacion del mapa como *.mat
%mapa=imread('mapa','bmp')
%save('mapa');
%load('mapa');

% Dibujo del mapa
imagesc(mapa)
set(gcf,'Color','white')
colormap('gray')
axis xy
hold on

% Limites del mapa x,y en numero de celdas , angulo en grados
v=axis;
mapmin=[v(1)+0.5,v(3)+0.5,0.0];
mapmax=[v(2)-0.5,v(4)-0.5,360.0];

%Rejilla
if rejilla==1,
    for i=v(1):v(2),
        for j=v(3):v(4),
            line([i,i],[i,j],'Color','k','Linestyle',':')
            line([i,j],[j,j],'Color','k','LineStyle',':')
        end
    end
end
        
