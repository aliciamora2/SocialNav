function [mapa, peopleNew, flagReplan] = socialNav(resolution,mapaOld,peopleOld,H,Xstart,Ystart,Tpose,Xorigin,Yorigin)

    Xstart=double(round((Xstart+abs(Xorigin))/resolution));
    Ystart=double(round(H-((Ystart+abs(Yorigin))/resolution)));

    flagReplan=0;

    mapa=zeros(size(mapaOld));

    sub = rossubscriber('/personPose'); %Láser

    people=[];
    %Primero: subscriber para leer posicion de personas - bucle
    for i=1:10
        try
            msg = receive(sub,0.1);
            people=[people;msg.Data'];
        catch
            continue;
        end
    end

    if ~isempty(people)
        %people(:,1:2)=round(people(:,1:2),1);
        [people2,ia,~]=uniquetol(people(:,1:2),0.5,'ByRows',true);
        people=[people2,people(ia,3)];
        if ~isempty(peopleOld)
            peopleOld(:,1)=peopleOld(:,1)-Xstart;
            peopleOld(:,2)=peopleOld(:,2)-Ystart;
            coincidente=size(intersect(people,peopleOld,'rows'),1);
        else
            coincidente=0;
        end

        %Comparar que las personas no son las mismas
        if size(people,1)~=size(peopleOld,1) || coincidente~=size(people,1)
    
            peopleNew=people;
            flagReplan=1;
    
            %Cluster para diferenciar persona/grupo
            if size(people,1)>1
                T=clusterdata(people(:,1:2),'Cutoff',1.22,'Criterion','distance');
                numGroups=max(T)
            else
                numGroups=1;
                T=1;
            end
       
            for i=1:numGroups
                % Llamar a la funcion de Gaussians
                groupElem=find(T==i);
                groupPose=people(groupElem,:);
                groupPose(:,2)=groupPose(:,2)+5;
                groupPose(:,1)=groupPose(:,1)+1;
                imagen = gaussians([groupPose(:,2),groupPose(:,1),groupPose(:,3)]);

                %Reescalar imagenes
                imagen = imresize(imagen,resolution*(71/10));
                imagen=flip(imagen',2);
                imagen = imrotate(imagen,rad2deg(Tpose),'bilinear','crop');

                %Calcular el punto de insercion de la gaussiana
                lengthMap=round(size(imagen,1)*5.5/10);
                lengthInsert=round(size(imagen,1)/2);
                insertX=round(lengthMap*cos(-Tpose))+Xstart;
                insertY=round(lengthMap*sin(-Tpose))+Ystart;

                %Actualizar mapa
                mapa(insertY-lengthInsert+1:insertY+lengthInsert-1,insertX-lengthInsert+1:insertX+lengthInsert-1)=mapa(insertY-lengthInsert+1:insertY+lengthInsert-1,insertX-lengthInsert+1:insertX+lengthInsert-1)+imagen;

            end
        else
            peopleNew=peopleOld;
        end
    else
        peopleNew=[];
    end
end