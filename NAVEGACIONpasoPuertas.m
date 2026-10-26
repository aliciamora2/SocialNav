%% Script para navegación topo-geométrica con FMSquare en el RB1-UR3 Dual
%% Primer paso: cargar mapa binario
rosshutdown
clear
clc
%pyenv('Version','/usr/bin/python2.7'); % Solo hacerlo si esta la 3.8
addpath('/home/alicia/Documentos/Codigo/toolbox_fast_marching','/home/alicia/Documentos/Codigo/toolbox_fast_marching/data','/home/alicia/Documentos/Codigo/toolbox_fast_marching/toolbox');
load('RB1dual.mat'); %Carga mapa

% Cambio del tamaño del mapa
matrix=imresize(matrix,0.5);
resolution=resolution*2;
[H,Wi]=size(matrix); % Dimensiones del mapa

bw=matrix;
SE=strel('disk',4);
bw4 = imerode(bw,SE);

[topoGraph,nodeCentroids,perimeters]=topoMap(bw4); %Mapa topológico
nodeCentroids=round(nodeCentroids);

figure(1)
imshow(bw)
xlim([437 700])
ylim([310 653])
hold on
plot(topoGraph,'XData',nodeCentroids(:,1),'YData',nodeCentroids(:,2))

%% Segundo paso: Fast Marching con ROS
% Conectar con ROS
rosinit

maxLinVel=0.1;
maxAngVel=0.1;

sub2 = rossubscriber('/robot/amcl_pose','DataFormat','struct'); %Localizador
startSub = rossubscriber('/targetBase'); %Este topic envía al robot a qué punto queremos que se dirija
%Fuera del despacho: [546;475]
%Mesa: [525;492]
%Nodo 7: [519;478]
%Nodo 12: [543;478]
%end_points=[554;487];

pause(1)

while true

    Xstart=double(round((sub2.LatestMessage.Pose.Pose.Position.X+abs(Xorigin))/resolution));
    Ystart=double(round(H-((sub2.LatestMessage.Pose.Pose.Position.Y+abs(Yorigin))/resolution)));
    start_points=[Xstart;Ystart]; %Punto en el que se encuentra el robot
    topoInit=checkLocation(perimeters,start_points); %Nodo topológico correspondiente

    startMsg = receive(startSub); %Espera a recibir coordenada de destino
    poseX=startMsg.X;
    poseY=startMsg.Y;
    poseAngulo=startMsg.Ori;
    end_points=[double(poseX);double(poseY)];
    topoDes=checkLocation(perimeters,end_points); %Nodo topológico correspondiente

    if topoInit==topoDes %No hay que cruzar puertas
        disp('NO SE CRUZAN PUERTAS')
        disp('FAST MARCHING...')
        Robotnik_FMSquare(Xorigin,Yorigin,resolution,H,maxLinVel,maxAngVel,bw,bw4,end_points); %Llegar hasta el punto deseado
    else %Combinacion habs. - puertas
        topoList=shortestpath(topoGraph,topoInit,topoDes,'Method','positive');
        for i=1:round(length(topoList)/3) %Tramos hab.-puerta
            %Tramo 1: aproximarse a la puerta
            disp('FAST MARCHING...')
            Robotnik_FMSquare(Xorigin,Yorigin,resolution,H,maxLinVel,maxAngVel,bw,bw4,nodeCentroids(topoList(3*i-1),:)');
            disp('GIRANDO HACIA LA PUERTA...')
            robotOrientation(nodeCentroids(topoList(3*i),:)',nodeCentroids(topoList(3*i-1),:)'); %Orientar robot hacia la puerta
            disp('CRUZANDO LA PUERTA...')
            crossDoor(Xorigin,Yorigin,resolution,H,nodeCentroids(topoList(3*i),:)');
        end
        disp('FAST MARCHING...')
        Robotnik_FMSquare(Xorigin,Yorigin,resolution,H,maxLinVel,maxAngVel,bw,bw4,end_points); %Llegar hasta el punto deseado
    end

    pub = rospublisher('/baseOK','std_msgs/Bool'); %Avisa de que ha llegado a 
    msgPub = rosmessage(pub);
    msgPub.Data=1;
    send(pub,msgPub);
    pause(5)

end

rosshutdown

function polyN=checkLocation(perimeters,point)
    found=false;
    polyN=0;
    while ~found
        polyN=polyN+1;
        if inpolygon(point(2),point(1),perimeters(polyN).vertices(:,1),perimeters(polyN).vertices(:,2))
            found=true;
        end
    end
end
