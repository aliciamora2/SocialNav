function robotOrientation(desPt,currPt)
    %El robot gira para mirar hacia la puerta que tiene que atravesar
    
    disp('COMPROBANDO ORIENTACION...')
    sub = rossubscriber('/robot/amcl_pose','DataFormat','struct'); %Localizador
    pub = rospublisher('/matlabVel','geometry_msgs/Twist'); %Velocidad del robot
    msgPub = rosmessage(pub);
    msgPub.Linear.X=0;
    msgPub.Angular.Z=0;

    currentOr=quat2eul([sub.LatestMessage.Pose.Pose.Orientation.W sub.LatestMessage.Pose.Pose.Orientation.X sub.LatestMessage.Pose.Pose.Orientation.Y sub.LatestMessage.Pose.Pose.Orientation.Z]);
    desiredOr=-atan2(desPt(2)-currPt(2),desPt(1)-currPt(1));

    while abs(angdiff(desiredOr,currentOr(1)))>0.15 %Corregir orientación
        if angdiff(desiredOr,currentOr(1))>0 %Giro derecha
            disp('GIRO DERECHA')
            msgPub.Angular.Z=-0.07;
            send(pub,msgPub);
        else %Giro izquierda
            disp('GIRO IZQUIERDA')
            msgPub.Angular.Z=0.07;
            send(pub,msgPub);
        end
        msg=receive(sub);
        currentOr=quat2eul([msg.Pose.Pose.Orientation.W msg.Pose.Pose.Orientation.X msg.Pose.Pose.Orientation.Y msg.Pose.Pose.Orientation.Z]);
    end
    disp('ORIENTACION CORRECTA')
    msgPub.Linear.X=0;
    msgPub.Angular.Z=0;
    send(pub,msgPub);
end