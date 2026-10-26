% CALCULO DEL VECTOR DE LAS DISTANCIAS VECTORIALES OBSERVADAS EN EL MAPA
% Nuevo
% 
%distancias estimadas segun el mapanew aprendido
%
function [v1]=dist_est(posicion,mapanew,mapmax,mapmin,num_medidas,incr_th0)
%
% posicionnew(1)=x, posicionnew(2)=y
% posicionnew(3)=theta del robot en grados
% incr_th0 cada cuantos º se toma una medida en radianes
% mapa= reticula del mapa
%
% Alcance maximo operativo del sensor en metros
% laser        30 metros
% ultrasonidos  8 metros
% tamaño de la celda usada en la discretizacion 0.125 metros
% Por defecto suponemos US
cell_size=0.121;
s_range=10/cell_size;

%se inicializan las medidas
v1=zeros(1,num_medidas);

ths=+90*pi/180; %angulo respecto al eje del robot de la primera medida
phi=0;

% Vector traslacion. T=[0.001 a 1]
T=1;

for j=1:num_medidas,     
    %
    x=posicion(1);
    y=posicion(2);
    thr=(-posicion(3)+90)*pi/180;% se pasa a radianes   
    % 0 en el mapa indica obstaculo
    % 1 libre
    % 0.5 sin informacion   
    sin_sensor=sin(thr+ths);
    cos_sensor=cos(thr+ths);
    incr_x=T*sin_sensor;
    incr_y=T*cos_sensor;
    
    final=0;
    dis=0;

    while(final < 2),
        x_round=round(x);
        y_round=round(y);

        if mapanew(x_round,y_round)==0,   %~=0),  
            if dis==0,
              dis=sqrt((x-posicion(1))^2+(y-posicion(2))^2);
              final=final+1;
            end           
        end    

        x=x+incr_x;
        y=y+incr_y;  
         if x>mapmax(1), final=2;end
         if x<mapmin(1), final=2;end
         if y>mapmax(2), final=2;end
         if y<mapmin(2), final=2;end    
    end                
    
    % se verifica
    if dis<0,dis=0;end
    if dis>s_range, dis=s_range;end

    v1(j)=dis;
    ths=ths-incr_th0;%(incr_th0*pi/180); % incremento de angulo entre sensores
end 
