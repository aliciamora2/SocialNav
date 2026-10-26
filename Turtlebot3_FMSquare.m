%% Script para navegación con FMSquare en el Turtlebot3 (simulado)
% En terminal 1: roslaunch turtlebot3_gazebo turtlebot3_world.launch
% En terminal 2: roslaunch turtlebot3_navigation turtlebot3_navigation.launch
% En terminal 3, sólo si se quiere teleoperar: roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 

%% Primer paso: cargar mapa binario
clear
clc
addpath('/home/alicia/Documentos/Codigo/toolbox_fast_marching','/home/alicia/Documentos/Codigo/toolbox_fast_marching/data','/home/alicia/Documentos/Codigo/toolbox_fast_marching/toolbox');
load('turtlebot3.mat');
[H,Wi]=size(matrix); % Dimensiones del mapa

bw=matrix;
SE=strel('square',5);
bw4 = imerode(bw,SE);

figure(1)
imshow(bw)
hold on

%% Segundo paso: conectar con ROS
rosinit

sub = rossubscriber('/scan');
sub2 = rossubscriber('/amcl_pose');
pub = rospublisher('/cmd_vel','geometry_msgs/Twist');
msgPub = rosmessage(pub);
pause(1);

%% Tercer paso: primera aplicación del fast marching
% Recibir pose inicial del robot
msg2 = sub2.LatestMessage; % Recibe info. de pose del robot
Xstart=Wi -((msg2.Pose.Pose.Position.Y+abs(Yorigin))/resolution);
Ystart=H -((msg2.Pose.Pose.Position.X+abs(Xorigin))/resolution);
start_points=[Xstart;Ystart];

end_points=[195;170]; %AQUI CAMBIAR POR PUNTO FINAL

W=FMdist(bw4');
Wini=W;
W1= rescale( double(bw')); % para representacion

% FM para path global
options.nb_iter_max = Inf;
options.end_points = end_points;
options.Tmax = sum(size(W));
[D,S] = perform_fast_marching_2d(W, start_points, options);
path = extract_path_2d(D,end_points, options); %Path global
path = unique(round(path),'rows','stable');
path = dpsimplify(path,1);
path=flip(path);
original_path=path;

figure(2);
plot_path_2d(W1,S,path,start_points,end_points);
colormap gray(256);

%% Cuarto paso: ejecucion del camino

figure(1)
h=plot(0,0);
h2=plot(0,0);

pp = controllerPurePursuit('MaxAngularVelocity', 0.08,'LookaheadDistance',4);
navigate=true;
while navigate
    % ------------------- SENSORES ------------------- %
    msg = receive(sub); % Recibe info. del laser
    msg2 = sub2.LatestMessage; % Recibe info. de pose del robot
    
    delete(h);
    delete(h2);
    
    % Extraccion de la pose del robot [X Y T]
    Xpose=msg2.Pose.Pose.Position.X;
    Ypose=msg2.Pose.Pose.Position.Y;
    Tpose=quat2eul([msg2.Pose.Pose.Orientation.X msg2.Pose.Pose.Orientation.Y msg2.Pose.Pose.Orientation.Z msg2.Pose.Pose.Orientation.W]);
    Tpose=Tpose(3);
    
    % Cambio a coordenadas de la imagen del mapa
    XposePlot=Wi -((Ypose+abs(Xorigin))/resolution);
    YposePlot=H -((Xpose+abs(Yorigin))/resolution);
    figure(1)
    h=plot(round(XposePlot),round(YposePlot),'Marker','o','LineWidth',2);
    hold on;
    drawnow;
    
    % Cambio de medidas del laser a la pose del robot en la imagen del mapa
    v0=msg.Ranges/resolution;
    num_sensores = length(msg.Ranges);
    medidas_obs=zeros(1,num_sensores);
    ang_robot=Tpose-(90*pi/180);
    incr_ang=msg.AngleIncrement;%en radianes
    medidas_obsCartesian=zeros(2,num_sensores);
    for index=1:num_sensores
      medidas_obs(index)=v0(index)*exp(j*(ang_robot+(index-1)*incr_ang));
      medidas_obsCartesian(1,index)= XposePlot - round(real(medidas_obs(index)));
      medidas_obsCartesian(2,index)= YposePlot + round(imag(medidas_obs(index)));
    end
    
    figure(1)
    h2=plot(medidas_obsCartesian(1,:),medidas_obsCartesian(2,:),'g.','MarkerSize',3);
    
    % ------------------- REPLANNING ------------------- %
    Wt=bw; % se carga el plano inicial de la planta y se meten las medidas observadas
    for index=1:num_sensores
      if (v0(index)<50 )
          ni=round(medidas_obsCartesian(1,index));
          nj=round(medidas_obsCartesian(2,index));
          Wt(nj,ni)=0;
      end   
    end

    c=improfile(Wt,path(:,1),path(:,2)); %Valores del mapa bajo el camino
    if any(~c) %Si existe colision, se cambia el path
        disp('Replanning...')
        % Extraccion de mapa local
        x_c=round(XposePlot);
        y_c=round(YposePlot);
        [fils,cols]=size(Wt);
        if x_c<100, x_c_i=1;x_c_f=200;
        elseif x_c>cols-100, x_c_i=cols-200; x_c_f=cols;
        else, x_c_i=x_c-100;x_c_f=x_c+100;
        end
        Wloc=Wt(1:fils,x_c_i:x_c_f);
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
        %Saturacion para un camino mas recto, natiral - filtro de velocidad, se limita la vel. maxima de propagacion del frente de onda
        for ix=1:fils
            for iy=1:cols
                if Wtot(iy,ix)>0.4, Wtot(iy,ix)=0.4; end 
            end
        end
        W=Wtot; %Actualiza dist. transf. original
        bw=Wt; %se salvan los datos introducidos en el mapa
        W1= rescale(double(Wt')); % para representacion
    
    
        start_points=[XposePlot;YposePlot];
        options.nb_iter_max = Inf;
        options.end_points = end_points;
        options.Tmax = sum(size(W));
        [D,S] = perform_fast_marching_2d(W, start_points, options);
        path = extract_path_2d(D,end_points, options);
        path = unique(round(path),'rows','stable');
        path = dpsimplify(path,1);
        path=flip(path);
    
        %se dibuja el camino restante
        figure(2);
        plot_path_2d(W1,S,path,start_points,end_points);
        colormap gray(256);
    end
    %------------------------------------------------------
    
    % ------------------- ACTUADORES ------------------- %
    if size(path,1)>2 && pdist2([XposePlot,YposePlot],[path(2,1), path(2,2)])<5 %Elimina puntos alcanzados del camino
        path(1,:)=[];
    end

    Xwp=path(:,1);
    Ywp=path(:,2);

    waypoints = [((H-Ywp)*resolution)-abs(Yorigin),((Wi-Xwp)*resolution)-abs(Xorigin)];

    desiredAngle = atan2(XposePlot-Xwp(2),YposePlot-Ywp(2));
    if abs(angdiff(Tpose,desiredAngle))<0.25
        pp = controllerPurePursuit('DesiredLinearVelocity',0.08,'MaxAngularVelocity', 0.1,'LookaheadDistance',4);
        disp('recto')
    else
        pp = controllerPurePursuit('DesiredLinearVelocity',0.03,'MaxAngularVelocity', 0.3,'LookaheadDistance',1);
        disp('giro')
    end

    pp.Waypoints=waypoints;
    [vPure,wPure]=pp([Xpose Ypose Tpose]);
    msgPub.Linear.X=vPure;
    msgPub.Angular.Z=wPure;

    if pdist2([XposePlot,YposePlot],end_points')<6
        navigate=false;
         msgPub.Linear.X=0;
         msgPub.Angular.Z=0;
         send(pub,msgPub);
        disp('Fin del trayecto');
    end
    send(pub,msgPub);

end
%%
rosshutdown