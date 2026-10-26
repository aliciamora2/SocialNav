%DIBUJA EL MAPA nuevo

function dibuja_mapanew(mapanew)

%
% se dibuja el mapa traspuesto ya que la dim 1 corresponde a x y la 2 a y
%
clf;
imagesc(mapanew,'CDataMapping','scaled')
set(gcf,'Color','white');
set(gcf,'BackingStore','off');
set(gca,'DataAspectRatio',[1 1 1]);


colormap('gray')
axis xy
hold on

%for i=1:NP,  
%    if(mapa(round(poblacion(i,2)),round(poblacion(i,1)))~=0)
%        plot(poblacion(i,1),poblacion(i,2),'r.','MarkerSize',3);
%    else 
%        plot(poblacion(i,1),poblacion(i,2),'w.','MarkerSize',5);
%    end
%end      