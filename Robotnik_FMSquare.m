function Robotnik_FMSquare(Xorigin,Yorigin,resolution,H,maxLinVel,maxAngVel,bw,bw4,end_points)
    %Navegación en espacios grandes (habitaciones) usando Fast Marching Square

    usePeopleModule=1; %Flag para saber si queremos detectar personas o no (1 - SI, 0 - NO)

    sub = rossubscriber('/robot/front_laser/scan'); %Láser
    sub2 = rossubscriber('/robot/amcl_pose'); %Localizador
    pub = rospublisher('/matlabVel','geometry_msgs/Twist'); %Velocidad del robot
    msgPub = rosmessage(pub);
    msgPub.Linear.X=0;
    msgPub.Angular.Z=0;
    pause(1);
    
    % Recibir pose inicial del robot
    msg2=sub2.LatestMessage;
    Xstart=double(round((msg2.Pose.Pose.Position.X+abs(Xorigin))/resolution));
    Ystart=double(round(H-((msg2.Pose.Pose.Position.Y+abs(Yorigin))/resolution)));
    start_points=[Xstart;Ystart];
    
    % Calcular camino y ejecutarlo
    
    % Primera aplicación del fast marching
    W=FMdist(bw4');
    Wini=W;
    W1= rescale(double(bw')); % para representacion
    
    %  FM para path global
    options.nb_iter_max = Inf;
    options.end_points = end_points;
    options.Tmax = sum(size(W));
    [D,S] = perform_fast_marching_2d(W, start_points, options);
    path = extract_path_2d(D,end_points, options); %Path global
    %path = unique(round(path),'rows','stable');
    %path = dpsimplify(path,1);
    path=flip(path);

    Wpeople=W;

    %figure(2);
    plot_path_2d(Wpeople,S,path,start_points,end_points);
    xlim([437 700])
    ylim([310 653])
    colormap gray(256);
    
    % Ejecucion del camino
    %figure(1)
    h=plot(0,0);
    h2=plot(0,0);
    
    pp = controllerPurePursuit('MaxAngularVelocity', maxLinVel,'LookaheadDistance',6);
    
    peopleOld=[];

    nErrors=0;
    flagReplan=0;
    %isError=false;
    navigate=true;
    while navigate
        W=FMdist(bw4');

        figure(1)
        hold on
%         imshow(W)
%         xlim([437 700])
%         ylim([310 653])
%         hold on

%         plot_path_2d(W,S,path,start_points,end_points);
%         xlim([437 700])
%         ylim([310 653])
% 
%         hold on

        % ------------------- SENSORES ------------------- %
        msg2=sub2.LatestMessage; % Recibe info. de pose del robot
    
        delete(h);
        delete(h2);
    
        % Extraccion de la pose del robot [X Y T]
        Xpose=msg2.Pose.Pose.Position.X;
        Ypose=msg2.Pose.Pose.Position.Y;
        Tpose=quat2eul([msg2.Pose.Pose.Orientation.X msg2.Pose.Pose.Orientation.Y msg2.Pose.Pose.Orientation.Z msg2.Pose.Pose.Orientation.W]);
        Tpose=Tpose(3);
    
        % Cambio a coordenadas de la imagen del mapa
        XposePlot=double(round((Xpose+abs(Xorigin))/resolution));
        YposePlot=double(round(H -((Ypose+abs(Yorigin))/resolution)));
        %figure(1)
        h=plot(XposePlot,YposePlot,'Marker','o','LineWidth',2);
        xlim([437 700])
        ylim([310 653])
        hold on
        %hold on;
        drawnow;
        
        %Deteccion de personas y modelado
        %Wt=[]; %Borramos los datos anteriores
        Wt=bw; % se carga el plano inicial de la planta y se meten las medidas observadas (SIN EROSION)
        if usePeopleModule==1
            [mapaGaussianas, peopleNew, flagReplan] = socialNav(resolution,Wini,peopleOld,H,Xpose,Ypose,Tpose,Xorigin,Yorigin);
            peopleOld=peopleNew;
            mapaGaussianas=mapaGaussianas.*4;
            mapaGaussianas(mapaGaussianas>1)=1;
            maskGaussianas = imbinarize(abs(mapaGaussianas),0.1);
            PathGaussian = improfile(mapaGaussianas,path(:,1),path(:,2));
            if ~any(PathGaussian>0.15)
                flagReplan = 0;
                disp("NO hay persona en el camino")
            else
                disp("SI hay personas en el camino")
            end
        end

        msg = receive(sub); % Recibe info. del laser

        % Cambio de medidas del laser a la pose del robot en la imagen del mapa
        v0=msg.Ranges/resolution;
        num_sensores = length(msg.Ranges);
        medidas_obs=zeros(1,num_sensores);
        ang_robot=-Tpose-2.1511182785;
        incr_ang=msg.AngleIncrement;%en radianes
        medidas_obsCartesian=zeros(2,num_sensores);
        for index=1:num_sensores
          medidas_obs(index)=v0(index)*exp(j*(ang_robot+(index-1)*incr_ang));
          medidas_obsCartesian(1,index)= XposePlot + round(real(medidas_obs(index)));
          medidas_obsCartesian(2,index)= YposePlot + round(imag(medidas_obs(index)));
        end
        %figure(1)
        h2=plot(medidas_obsCartesian(1,:),medidas_obsCartesian(2,:),'g.','MarkerSize',3);
        xlim([437 700])
        ylim([310 653])
        hold on
        drawnow;
    
        % ------------------- REPLANNING ------------------- %
        for index=1:num_sensores
          if (v0(index)<50)
              ni=round(medidas_obsCartesian(1,index));
              nj=round(medidas_obsCartesian(2,index));
              if usePeopleModule==1
                  if maskGaussianas(nj,ni)==0
                      Wt(nj,ni)=0;
                  end
              else
                  Wt(nj,ni)=0;
              end
          end
        end

        c=improfile(Wt,path(:,1),path(:,2)); %Valores del mapa bajo el camino
        if any(~c) || flagReplan %Si existe colision o detectamos persona, se cambia el path
            try
                disp('Replanning...')
                % Extraccion de mapa local
                x_c=XposePlot;
                y_c=YposePlot;
                [fils,cols]=size(Wt);
                if x_c<100, x_c_i=1;x_c_f=200;
                elseif x_c>cols-100, x_c_i=cols-200; x_c_f=cols;
                else, x_c_i=x_c-100;x_c_f=x_c+100;
                end
                Wloc=[];
                Wloc=Wt(1:fils,x_c_i:x_c_f);
                SE=strel('disk',2);
                wl4= imerode(Wloc,SE); %Incremento de obstaculos
                Wlocal=FMdist(wl4');
                % Union de dist. transf. local y global

                Wtot=Wini; %Wini es la tranformada de distancias inicial
                ily=0;
                for iy=x_c_i:x_c_f
                    ily=ily+1;
                    ilx=0;
                    for ix=1:fils
                        ilx=ilx+1;
                        Wtot(iy,ix)=min(Wtot(iy,ix),Wlocal(ily,ilx));
                    end
                end
                %Saturacion para un camino mas recto, natural - filtro de velocidad, se limita la vel. maxima de propagacion del frente de onda
                for ix=1:fils
                    for iy=1:cols
                        if Wtot(iy,ix)>0.4, Wtot(iy,ix)=0.4; end 
                    end
                end

                W=Wtot; %Actualiza dist. transf. original
                %bw=Wt; %se salvan los datos introducidos en el mapa
%                 W1= rescale(double(Wt')); % para representacion

                if usePeopleModule==1
                    if flagReplan
                        disp('PERSONAS INCLUIDAS EN EL MAPA')
                        Wpeople=[];
                        Wpeople=W.*imcomplement(mapaGaussianas');
                    else
                        Wpeople=W;
                    end
                else
                    Wpeople=W;
                end
            
                start_points=[XposePlot;YposePlot];
                options.nb_iter_max = Inf;
                options.end_points = end_points;
                options.Tmax = sum(size(Wpeople));
                [D,S] = perform_fast_marching_2d(Wpeople, start_points, options);
                path = extract_path_2d(D,end_points, options);
                path = unique(round(path),'rows','stable');
                path = dpsimplify(path,1);
                path=flip(path);
        
                %figure(2);
                plot_path_2d(Wpeople,S,path,start_points,end_points);
                xlim([437 700])
                ylim([310 653])
                hold on
                colormap gray(256);
            catch
                disp('ERROR DETECTADO');
                msgPub.Linear.X=0;
                msgPub.Angular.Z=0;
                send(pub,msgPub);
    
                %Recuperamos los valores originales
                %bw=matrix;
                W=FMdist(bw4');
                %Wini=W;
                W1= rescale(double(bw'));
                [D,S] = perform_fast_marching_2d(W, start_points, options);
                path = extract_path_2d(D,end_points, options); %Path global
                path = unique(round(path),'rows','stable');
                path = dpsimplify(path,1);
                path=flip(path);
                
                pause(5); %Espera 5 segundos para que el obstáculo se quite de en medio
    
                %isError=true;
                nErrors=nErrors+1;
                if nErrors < 3
                    continue;
                else
                    disp('LA META NO PUEDE SER ALCANZADA');
                    rosshutdown
                end
                
            end
        else
            Wpeople=W;
            plot_path_2d(Wpeople,S,path,start_points,end_points);
            xlim([437 700])
            ylim([310 653])
        end

    
        nErrors=0;
        % ------------------- ACTUADORES ------------------- 
        while size(path,1)>2 && pdist2([XposePlot,YposePlot],[path(2,1), path(2,2)])<5 %Elimina puntos alcanzados del camino
            path(1,:)=[];
        end
    
        Xwp=path(:,1);
        Ywp=path(:,2);
    
        waypoints = [(Xwp*resolution)-abs(Xorigin),((H-Ywp)*resolution)-abs(Yorigin)];
        
        desiredAngle = atan2(YposePlot-Ywp(2),Xwp(2)-XposePlot);

        if abs(angdiff(Tpose,desiredAngle))<1.0
            pp = controllerPurePursuit('DesiredLinearVelocity',maxLinVel,'MaxAngularVelocity', maxAngVel/2,'LookaheadDistance',0.75);
            disp('RECTO');
        else
            pp = controllerPurePursuit('DesiredLinearVelocity',maxLinVel/3,'MaxAngularVelocity', maxAngVel,'LookaheadDistance',0.5);
            disp('GIRO');
        end
    
        pp.Waypoints=waypoints;
        [vPure,wPure]=pp([Xpose Ypose Tpose]);
        msgPub.Linear.X=vPure;
        msgPub.Angular.Z=wPure;
        
        if pdist2([XposePlot,YposePlot],end_points')<4
            navigate=false;
             msgPub.Linear.X=0;
             msgPub.Angular.Z=0;
             send(pub,msgPub);
            disp('FM2: meta alcanzada');
        end
        send(pub,msgPub);
    end
end