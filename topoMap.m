function [topoGraph,nodeCentroids,perimeters]=topoMap(bw4)
    %Localización de puertas usando Watershed

    J = bwareafilt(bw4,1);
    D = FMdist(J)*10; % D: mapa de distancias
    D=-D;
    mask = imextendedmin(D,2); % Evita seleccionar mínimos locales
    D2 = imimposemin(D,mask);
    L = watershed(D2);
    L(~J) = 0;

    %Rellena líneas de separación entre habitaciones con una etiqueta
    doorLoc=zeros(size(J));
    for i=1:size(L,1)
        for j=1:size(L,2)
            if L(i,j)==0 && J(i,j)==1
                doorLoc(i,j)=1;
                options=[L(i-1,j-1), L(i-1,j), L(i,j-1), L(i+1,j+1), L(i+1,j), L(i,j+1), L(i+1,j-1), L(i-1,j+1)];
                options=nonzeros(options)';
                newLabel=min(options);
                L(i,j)=newLabel;
            end
        end
    end

%     RGB = label2rgb(L);
%     figure(10)
%     imshow(RGB)
%     hold on

    %Posición del punto central de cada puerta y su orientación
    doorCentroids = regionprops(imbinarize(doorLoc),'centroid');
    doorCentroids = round(cat(1,doorCentroids.Centroid));
    doorOrientation= regionprops(imbinarize(doorLoc),'Orientation');
    doorOrientation = cat(1,doorOrientation.Orientation);
    
    %Comprobar qué habitaciones se tocan en cada puerta
    neighborsTotal=[];
    for i=1:length(doorCentroids)
        neighbors=[L(doorCentroids(i,2),doorCentroids(i,1)),L(doorCentroids(i,2)-1,doorCentroids(i,1)),L(doorCentroids(i,2),doorCentroids(i,1)-1),L(doorCentroids(i,2)-1,doorCentroids(i,1)-1),L(doorCentroids(i,2)+1,doorCentroids(i,1)),L(doorCentroids(i,2),doorCentroids(i,1)+1),L(doorCentroids(i,2)+1,doorCentroids(i,1)+1),L(doorCentroids(i,2)+1,doorCentroids(i,1)-1),L(doorCentroids(i,2)-1,doorCentroids(i,1)+1),L(doorCentroids(i,2)-2,doorCentroids(i,1)-2),L(doorCentroids(i,2)-2,doorCentroids(i,1)+2),L(doorCentroids(i,2)+2,doorCentroids(i,1)-2),L(doorCentroids(i,2)+2,doorCentroids(i,1)+2)];
        neighbors=unique(nonzeros(neighbors))';
        neighborsTotal=[neighborsTotal;neighbors];
    end
    
    %Punto central de cada hab. y su perímetro
    pxCentr=regionprops(L,'Centroid'); %Centroide de las habitaciones
    pxCentr=round(vertcat(pxCentr.Centroid));
    tempCell.vertices=0;
    perimeters=repmat(tempCell,1,size(pxCentr,1));
    for i=1:size(pxCentr,1) %Calculamos perímetro de cada habitación
        BWtemp=zeros(size(L));
        BWtemp(L==i)=1;
        perimeter=cell2mat(bwboundaries(BWtemp));
        Preduced = reducepoly(perimeter);
        perimeters(i).vertices=Preduced;
    end
    
    %Grafo con pesos
    edges=[]; %Forma: puerta, habitación1, habitación2
    edgesDist=[]; %Forma: distancia puerta-hab.1, distancia puerta-hab.2
    for i=1:size(doorCentroids,1)
        edges=[edges;i+length(perimeters),i+length(perimeters)*2,neighborsTotal(i,1)];
        edges=[edges;i+length(perimeters)*2,i+length(perimeters),neighborsTotal(i,2)];
        edgesDist=[edgesDist;1.2,round(pdist2(doorCentroids(i,:),pxCentr(neighborsTotal(i,1),:))*0.05,2)];
        edgesDist=[edgesDist;1.2,round(pdist2(doorCentroids(i,:),pxCentr(neighborsTotal(i,2),:))*0.05,2)];
        %c=c+2;
    end
    
    vecinos=[];
    for i=1:size(edges,1)
        vecinos=[vecinos;double(edges(i,1)) double(edges(i,2)) edgesDist(i,1); double(edges(i,1)) double(edges(i,3)) edgesDist(i,2)];
    end
    vecinos(:,1:2)=sort(vecinos(:,1:2),2);
    vecinos = unique(vecinos,'rows');
    
    nodeCentroids=pxCentr;
    auxArray=[];
    for i=1:size(doorCentroids,1)
        coord1=[doorCentroids(i,1)+(0.6/0.05)*cosd(doorOrientation(i)+90),doorCentroids(i,2)+(0.6/0.05)*sind(doorOrientation(i)+90)];
        coord2=[doorCentroids(i,1)-(0.6/0.05)*cosd(doorOrientation(i)+90),doorCentroids(i,2)-(0.6/0.05)*sind(doorOrientation(i)+90)];
        if L(round(coord1(2)),round(coord1(1)))==edges(1+(i-1)*2,3)
            nodeCentroids=[nodeCentroids;coord1];
            auxArray=[auxArray;coord2];
        else
            nodeCentroids=[nodeCentroids;coord2];
            auxArray=[auxArray;coord1];
        end
    end
    nodeCentroids=[nodeCentroids;auxArray];
    
    s=vecinos(:,1)';
    t=vecinos(:,2)';
    w=vecinos(:,3)';
    topoGraph=graph(s,t,w);
    %plot(topoGraph,'XData',nodeCentroids(:,1),'YData',nodeCentroids(:,2))
end