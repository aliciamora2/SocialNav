function crossDoor(Xorigin,Yorigin,resolution,H,end_points)
    %Estrategia para el paso de puertas usando el LiDAR 2D

    sub=rossubscriber('/robot/front_laser/scan','DataFormat','struct');
    sub2 = rossubscriber('/robot/amcl_pose'); %Localizador

    pub = rospublisher('/matlabVel','geometry_msgs/Twist');
    msgPub = rosmessage(pub);
    msgPub.Linear.X=0.04;
    msgPub.Angular.Z=0;
    send(pub,msgPub);

    goalreached=false;
    while ~goalreached
        msg2=receive(sub2); %Comprobar si la meta se ha alcanzado
        XposePlot=double(round((msg2.Pose.Pose.Position.X+abs(Xorigin))/resolution));
        YposePlot=double(round(H -((msg2.Pose.Pose.Position.Y+abs(Yorigin))/resolution)));
        if pdist2([XposePlot,YposePlot],end_points')<0.6/resolution
            goalreached=true;
            msgPub.Linear.X=0;
            msgPub.Angular.Z=0;
            send(pub,msgPub);
            continue;
        end

        msg=receive(sub);
        laserVals=msg.Ranges;
        if laserVals(393)<2 || laserVals(593)<2
            if laserVals(393)>laserVals(593) && abs(laserVals(393)-laserVals(593))>0.2
                %giro izq.
                msgPub.Angular.Z=0.05;
                send(pub,msgPub);
                disp('GIRO LIGERO A LA IZQUIERDA')
            elseif laserVals(393)<laserVals(593) && abs(laserVals(393)-laserVals(593))>0.2
                %Giro der
                msgPub.Angular.Z=-0.05;
                send(pub,msgPub);
                disp('GIRO LIGERO A LA DERECHA')
            end
        elseif laserVals(293)<2 || laserVals(693)<2
            if laserVals(293)>laserVals(693) && abs(laserVals(293)-laserVals(693))>0.2
                %giro izq.
                msgPub.Angular.Z=0.05;
                send(pub,msgPub);
                disp('GIRO LIGERO A LA IZQUIERDA')
            elseif laserVals(293)<laserVals(693) && abs(laserVals(293)-laserVals(693))>0.2
                %Giro der
                msgPub.Angular.Z=-0.05;
                send(pub,msgPub);
                disp('GIRO LIGERO A LA DERECHA')
            end
        else
            msgPub.Angular.Z=0;
            send(pub,msgPub);
        end
    end

    msgPub.Linear.X=0;
    msgPub.Angular.Z=0;
    send(pub,msgPub);
    disp('PUERTA CRUZADA')
end