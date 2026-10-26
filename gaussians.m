function Imagen = gaussians(poses)
    N = size(poses,1);
    disp('--------------')
    poses
    disp('--------------')
    disp(N)
    switch N
        case  1
            %% Modelado Gaussiano para una persona
            disp('Una persona');
            mu = [poses(1) poses(2)];          % Mean of the 1st component
            sigma1 = [0.23+0.25,0;0,0.92+0.25]; % Covariance of the 1st component
            sigma2 = [0.23+0.25,0;0,0.23+0.25];  % Covariance of the 2nd component
            
            r1 = mvnrnd(mu,sigma1,1000);
            r1(r1(:,2)<mu(2),:)=[];
            r2 = mvnrnd(mu,sigma2,1000);
            X = [r1; r2];
            
            X1=X;
            X1(:,1)=X1(:,1)-mu(1);
            X1(:,2)=X1(:,2)-mu(2);
            
            % Create rotation matrix
            theta = poses(3); % to rotate 90 counterclockwise
            R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            % Rotate your point(s)
            X2 = R*X1';
            
            X2=X2';
            
            Xnew(:,1)=X2(:,1)+mu(1);
            Xnew(:,2)=X2(:,2)+mu(2);
            
            gm = fitgmdist(Xnew,2);
            
            gmPDF = @(x,y) arrayfun(@(x0,y0) pdf(gm,[x0 y0]),x,y);
            %fsurf(gmPDF,[4 6])
            figure(100)
            cont=fcontour(gmPDF,[0 10]);
%             figure(2)
            Imagen  = flipdim(cont.ZData ,1);
            Imagen = rescale(Imagen);
            close figure 100

            %             Imagen = imrotate(Imagen,90);
    
        case  2
            %% Caso de 2 personas
            disp('Dos personas');
            mu1 = [poses(1,1) poses(1,2)];
            mu11 = [poses(2,1) poses(2,2)];
            Centroide = (mu1 + mu11)/2;
            Dh = pdist([mu1;mu11]);
            Di = 1.2+0.25;
            mupar = Centroide;          % Mean of the 1st component
            sigma = [(Dh/4)^2,0;0,(Di/2)^2]; % Covariance of the 1st component
            ang = atan((mu11(1) - mu1(1))/(mu11(2) - mu1(2)));
            m = makehgtform('zrotate',-ang);
            m = m(1:2,1:2);
            sigma = m * sigma * m.';
            
            rng('default') % For reproducibility
            Gauss = mvnrnd(mupar,sigma,1000);
            
            gm = fitgmdist(Gauss,2);
            
            gmPDF = @(x,y) arrayfun(@(x0,y0) pdf(gm,[x0 y0]),x,y);
            %fsurf(gmPDF,[4 6])
            figure(100)
            cont=fcontour(gmPDF,[0 10]);
%             figure(6)
            Imagen  = flipdim(cont.ZData ,1);
            Imagen = rescale(Imagen);
            close figure 100
%             imshow(Imagen);
        otherwise
            %% Caso de multiples personas

            mu = poses();
            mu(:,3) = [];

            forma = polyshape(mu(:,1)',mu(:,2)'); 
            [Cx,Cy] = centroid(forma);
            muPol = [Cx,Cy];
            Dh = perimeter(forma)/N;
            Di = -1;
            pose = 0;
            for j=1:N
                punto = [mu(j,:);[Cx,Cy]];
                dist = pdist(punto);
                if dist > Di
                    Di = dist;
                    pose = j;
                end
            end
            Di = Di + 0.265+0.25;
            ang = atan((mu(pose,1) - Cx)/(mu(pose,2) - Cy));
            m = makehgtform('zrotate',-ang);
            m=m(1:2,1:2);
            sigma = [(Dh/4)^2,0;0,(Di/2)^2];
            sigma = m *sigma*m.';
            rng('default') % For reproducibility
            Gauss = mvnrnd(muPol,sigma,1000);
            
            gm = fitgmdist(Gauss,2);
            
            gmPDF = @(x,y) arrayfun(@(x0,y0) pdf(gm,[x0 y0]),x,y);
            %fsurf(gmPDF,[4 6])
            figure(100)
            cont=fcontour(gmPDF,[0 10]);
            Imagen  = flipdim(cont.ZData ,1);
            Imagen = rescale(Imagen);

            close figure 100
            %imshow(Imagen)

    end
end
