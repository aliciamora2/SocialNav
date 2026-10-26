% 
%   Path_planning_FMsquare utilizando fast marching y la transformada de distancia
%   realizada tambien con fast marching
%
n = 200;
 map_name = 'uc3m_p3.png';
%map_name = 'uc3m_p3_parcial.bmp';

[Wo, cm] = imread(map_name);

bw=imread('uc3m_p3.png');
% bw=imread('uc3m_p3_parcial.bmp');
SE=strel('square',5);
bw3 = imdilate(~bw,SE);
bw4=~bw3;
% figure;imshow(bw4);
% pause

tic;
%W=bwdist(~bw4');
W=FMdist(bw4');
toc;
Wini=W;

% para representacion
W1= rescale( double(Wo') );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pick starting point
% se traspone para representarlo correctamente en pantalla
[start_points,end_points] = pick_start_end_point(Wo');

options.nb_iter_max = Inf;
options.end_points = end_points;
options.Tmax = sum(size(W));
%disp('Performing front propagation.');
[D,S] = perform_fast_marching_2d(W, start_points, options);
%disp('Extracting path.');
tic;
path = extract_path_2d(D,end_points, options);
toc;

figure(1);
%plot_fast_marching_2d(W,S,path,start_points,end_points);
%plot_fast_marching_2d(W1,S,path,start_points,end_points);
%plot_path_2d(W,S,path,start_points,end_points);
plot_path_2d(W1,S,path,start_points,end_points);
colormap gray(256);
%------------------------------------------------------


% Creacion del mapa como *.mat
% Estos mapas son los que se usan en la representacion
% grafica
mapa=imread('uc3m_p3_open','bmp');
mapa_close=imread('uc3m_p3_close','bmp');
mapa_obstacles=imread('uc3m_p3_obstacles','bmp');
% mapa=imread('uc3m_p3parcial','bmp');
save('mapa');
save('mapa_close');
save('mapa_obstacles');

% cargamos el mapa del entorno
load('mapa');
load('mapa_close');
load('mapa_obstacles');
% mapa traspuesto, es el que se usa en la prediccion de
% las medidas
mapa2=mapa' ; 
mapa2_close=mapa_close';
mapa2_obstacles=mapa_obstacles';

[mapmin,mapmax]=lim_mapa(mapa,0);

figure(2);
dibuja_mapanew(mapa_obstacles);

num_sensores=181; %361 medidas;
incr_th0=1*pi/180;   %0.5; % incremento entre sensores en grados
cell_size=12.1;  % en centimetros
s_range=15/cell_size;
v0=zeros(1,num_sensores);

%posicion de comienzo
posicion=zeros(1,3);
posicion(1:2)=start_points;

[fil,col]=size(path);
path_real=zeros(fil,5);%x,y,theta,vref,v
t_real=0;
speed=0;
final_path=0;

while final_path==0,
  th0=posicion(3);
  
  %------------------------------------------------------------
  % toma de datos de los sensores y de los despl. segun la odometria
  % y se pasa a unidades de reticula (laser en cm)
  v0=dist_est(posicion,mapa2_obstacles,mapmax,mapmin,num_sensores,incr_th0);
  %------------------------------------------------------------
  
  %--------------------------------------------------------------
  % medidas reales
  medidas_obs=zeros(1,num_sensores);
  robot=posicion(1)+j*posicion(2);
  ang_robot=(posicion(3)-90)*pi/180;% se pasa a radianes
  incr_ang=incr_th0;%en radianes
  for index=1:num_sensores,
    medidas_obs(index)=robot+v0(index)*exp(j*(ang_robot+(index-1)*incr_ang));
  end;
  
  %dibujamos las medidas en el mapa teorico abierto 
  figure(2); 
  dibuja_sensor(mapa,posicion,num_sensores,medidas_obs);
 
  %Dibujo de la posicion final del robot
  plot(posicion(1),posicion(2),'ko','LineWidth',1,'MarkerSize',3); 
  %-----------------------------------------------------------------
  
  %-----------------------------------------------------------------
  % sensor based replanning
  %Wt=Wo;
  Wt=bw; % se carga el plano inicial de la planta
         % y se meten las medidas observadas
  for index=1:num_sensores,
    if (v0(index)<50 )
        ni=round(real(medidas_obs(index)));
        nj=round(imag(medidas_obs(index)));
        Wt(nj,ni)=0;
    end   
  end;
  
  %fast marching de zona local---------------
  tic;
  x_c=round(posicion(1));
  y_c=round(posicion(2));
  [fils,cols]=size(Wt);
  if x_c<100, x_c_i=1;x_c_f=200;
  elseif x_c>cols-100, x_c_i=cols-200; x_c_f=cols;
  else x_c_i=x_c-100;x_c_f=x_c+100;
  end
  
  Wloc=Wt(1:fils,x_c_i:x_c_f); 
  
  SE=strel('square',5);
  wl3 =imdilate(~Wloc,SE);
  wl4=~wl3;
  Wlocal=FMdist(wl4');
   
  Wtot=Wini;
  [ifil,icol]=size(Wlocal);
  
  ily=0;
  for iy=x_c_i:x_c_f,
      ily=ily+1;
      ilx=0;
      for ix=1:fils,
          ilx=ilx+1;
          Wtot(iy,ix)=min(Wtot(iy,ix),Wlocal(ily,ilx));
      end
  end  
  
  % filtro de velocidad, se limita la vel. maxima de prpagacion
  % del frente de onda
  for ix=1:fils,
      for iy=1:cols,
          if Wtot(iy,ix)>0.4, Wtot(iy,ix)=0.4; end
      end
  end
  
  toc;
  
  figure(8);
  dibuja_mapanew(Wtot'); 
  W=Wtot;
  bw=Wt; %se salvan los datos introducidos en el mapa
  %-----------------------------------
  
%   tic;
%   bw=Wt;
%   SE=strel('square',5);
%   bw3 = imdilate(~bw,SE);
%   bw4=~bw3;
%   %figure;imshow(bw4);
%   
%   %W=bwdist(~bw4');
%   W=FMdist(bw4');
%   toc;

  % para representacion
%   W1= rescale( double(Wt') );
  W1= rescale( double(Wt') );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % pick starting point
  % se traspone para representarlo correctamente en pantalla
  start_points=posicion(1:2)';

  options.nb_iter_max = Inf;
  options.end_points = end_points;
  options.Tmax = sum(size(W));
  %disp('Performing front propagation.');
  [D,S] = perform_fast_marching_2d(W, start_points, options);
  %disp('Extracting path.');
  tic;
  path = extract_path_2d(D,end_points, options);
  length(path)
  toc;

  %se dibuja el camino restante
  figure(1);
  %clf;
  %plot_fast_marching_2d(W,S,path,start_points,end_points);
  %plot_fast_marching_2d(W1,S,path,start_points,end_points);
  %plot_path_2d(W,S,path,start_points,end_points);
  plot_path_2d(W1,S,path,start_points,end_points);
  colormap gray(256);
  %------------------------------------------------------
  
  

  
  
  %--------------------------------------------
  % Modelo de estado Xk=[x_k y_k teta_k]'
  % Entrada de control Uk=[T_k inc_teta]
  % Modelo de estado
  % G = [x_k+(T_k*cos(teta_k))
  %     y_k+(T_k*sin(teta_k))
  %     teta_k+inc_teta];
  %--------------------------------------------
  
  [fil,col]=size(path);

  % ojo el pto inicial del camino es el �ltimo de path
  dibuja_robot(posicion);
  
  i_alc=fil;
  p_inc=(path(i_alc,1)-posicion(1))+j*(path(i_alc,2)-posicion(2));
  a_inc=angle(p_inc);
  dist_g=abs(p_inc);
  
  % Estado inicial del vehiculo
  % cada ciclo de control se supone de 0.05 seg
  % y cada 0.5 segs se supone un barrido laser
  for ind=1:10,

    while (dist_g<3)&(i_alc>1),     
      v_dist=(path(i_alc,1)-posicion(1))+j*(path(i_alc,2)-posicion(2));
      dist_g=abs(v_dist);
      p_inc=v_dist;
      a_inc=angle(p_inc)*180/pi-posicion(3);
      cv=1;
      if i_alc>1,
        i_alc=i_alc-1;
      end  
    end  
    
    if (i_alc==1)&(dist_g>=0.75),
      v_dist=(path(i_alc,1)-posicion(1))+j*(path(i_alc,2)-posicion(2));
      dist_g=abs(v_dist);
      p_inc=v_dist;
      a_inc=angle(p_inc)*180/pi-posicion(3);   
      cv=dist_g/10;
    elseif (i_alc==1)& (dist_g<0.75),
      v_dist=(path(i_alc,1)-posicion(1))+j*(path(i_alc,2)-posicion(2));
      dist_g=abs(v_dist);
      p_inc=v_dist;
      a_inc=angle(p_inc)*180/pi-posicion(3);   
%       p_inc=0;
%       a_inc=0;
      cv=dist_g/10;
      final_path=1;
    end
    
    plot(path(i_alc+1,1),path(i_alc+1,2),'g.','MarkerSize',5);
    
    Xk_k= posicion';
    ref_speed=cv*W(round(Xk_k(1)),round(Xk_k(2)));
    inc_speed=ref_speed-speed;
    if (inc_speed>0.01),
      speed=speed+0.01;
    elseif (inc_speed>0) & (inc_speed<=0.01),
      speed=speed+inc_speed;
    elseif (inc_speed<-0.01),
      speed=speed-0.01;
    elseif (inc_speed<0) & (inc_speed>=-0.01),
      speed=speed-inc_speed;   
    end    
    
    Uk(1)= speed;        % avanza speed celdas cor ciclo
    Uk(2)=(a_inc/10);  
      
    Xk1_k(1) = Xk_k(1)+Uk(1)*cos(Xk_k(3)*pi/180)+Uk(1)*0.03*randn;
    Xk1_k(2) = Xk_k(2)+Uk(1)*sin(Xk_k(3)*pi/180)+Uk(1)*0.03*randn;
    Xk1_k(3) = Xk_k(3)+(Uk(2)+0.01*randn);
    
    posicion(1:3)= Xk1_k(1:3)';
    t_real=t_real+1;  %en decimas de segundo
    path_real(t_real,1:3)=Xk1_k(1:3)';
    path_real(t_real,4)=ref_speed*12.1/0.1; %speed en cm/s
    path_real(t_real,5)=speed*12.1/0.1; %speed en cm/s    
    dibuja_robot(posicion);
  end
  robot=Xk1_k(1)+j*Xk1_k(2);
  
  %pause

  figure(2);
  dibuja_mapanew(mapa_obstacles);  
  
%   figure(6);
%   dibuja_mapanew(W');  

end


  figure(5);
  dibuja_mapanew(mapa);  
  for i=1:length(path_real)
        plot(path_real(i,1), path_real(i,2), '-', 'LineWidth', 1);
  end
  figure(3);
  plot(path_real(1:t_real-1,4));
  figure(4);
  plot(path_real(1:t_real-1,5));
  