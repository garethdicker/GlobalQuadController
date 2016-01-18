function [ ] =simvisualization(t,X,sideview)

global propLocation 

X = X';
t = t';
Rbumper = 0.11;
Cbumper = propLocation(:,1);
Abumper = deg2rad(11);

disprate = 30; %Hz
recordrate = disprate/10; %Hz
disprate_idx = round((size(t,1)/(t(end)-t(1)))/disprate);
% disprate_idx = 1;

figure('units','normalized','outerposition',[0 0 1 1])

%% Create body-fixed centers of 4 bumpers + virtual bumper
load('locations2');

c1 = propLocation(:,1);
c2 = propLocation(:,2);
c3 = propLocation(:,3);
c4 = propLocation(:,4);

n1_b = rotmat('Z',deg2rad(45))'*rotmat('Y',Abumper + deg2rad(90))'* [1;0;0];
n2_b = rotmat('Z',deg2rad(135))'*rotmat('Y',Abumper + deg2rad(90))'* [1;0;0];
n3_b = rotmat('Z',deg2rad(-135))'*rotmat('Y',Abumper + deg2rad(90))'* [1;0;0];
n4_b = rotmat('Z',deg2rad(-45))'*rotmat('Y',Abumper + deg2rad(90))'* [1;0;0];
   
cR = Cbumper;

clear p1 p2 p3 p4;

%% Create body-fixed points of spherical bumper

[sx,sy,sz] = sphere;
sx = sx(9:13,:);
sy = sy(9:13,:);
sz = sz(9:13,:)+propLocation(3,1);
sr = Rbumper;
sxR = zeros(size(sx));
syR = zeros(size(sy));
szR = zeros(size(sz));

%% Create body-fixed points of Spiri body
p1 = [0.08;0.0115;0]-CoM;
p2 = [0;0.0575;0]-CoM;
p3 = [-0.1;0;0]-CoM;
p4 = [0;-0.0575;0]-CoM;
p5 = [0.08;-0.0115;0]-CoM;


%% Create body-fixed axes
po = [0;0;0];
px = [0.1;0;0];
py = [0;0.1;0];
pz = [0;0;0.1];

%%  Calculate axes ranges for plotting
axis_min = min([min(X(:,7))-0.4,min(X(:,8))-0.4,min(X(:,9))-0.4]);
axis_max = max([max(X(:,7))+0.4,max(X(:,8))+0.4,max(X(:,9))+0.4]);

for i = 1:disprate_idx:size(t,1)
   %% Rotate body-fixed points to world-frame points
   q = [X(i,10);X(i,11);X(i,12);X(i,13)];
   q = q/norm(q);
   R = quat2rotmat(q);
   T = [X(i,7);X(i,8);X(i,9)];
   
   p1_p = R'*p1 + T;
   p2_p = R'*p2 + T;
   p3_p = R'*p3 + T;
   p4_p = R'*p4 + T;
   p5_p = R'*p5 + T;
   
   c1_p = R'*c1 + T;
   c2_p = R'*c2 + T;
   c3_p = R'*c3 + T;
   c4_p = R'*c4 + T;
   cR_p = R'*cR + T;
   
   po_p = R'*po + T;
   px_p = R'*px + T;
   py_p = R'*py + T;
   pz_p = R'*pz + T;
   
   pts = [p1_p p2_p p3_p p4_p p5_p p1_p];
   
   %% Plot Spiri body points
   plot3(pts(1,:),pts(2,:),pts(3,:),'Color',[154 215 227]/255,'LineWidth',2);
   hold on;
   plot3(T(1),T(2),T(3),'rx','MarkerSize',8); %Centre of mass
   
   %% Plot Spiri 2-d bumpers 
   normal = cross(p1_p-p2_p,p2_p-p3_p);

   n1_w = R'*n1_b;
   n2_w = R'*n2_b;
   n3_w = R'*n3_b;
   n4_w = R'*n4_b;
   
   plotCircle3D(c1_p,n1_w,Rbumper);
   plotCircle3D(c2_p,n2_w,Rbumper);
   plotCircle3D(c3_p,n3_w,Rbumper);
   plotCircle3D(c4_p,n4_w,Rbumper);
   
%    normal2 = cross(c1_p-c3_p,c2_p-c4_p);
%    plotCircle3D(cR_p,normal2,Rbumper); %Virtual bumper
   
   %% Plot Spiri spherical bumper
   for j = 1:size(sx,1)
       for k = 1:size(sx,2)
           sxR(j,k) = R(:,1)'*[sx(j,k);sy(j,k);sz(j,k)];
           syR(j,k) = R(:,2)'*[sx(j,k);sy(j,k);sz(j,k)];
           szR(j,k) = R(:,3)'*[sx(j,k);sy(j,k);sz(j,k)];       
       end
   end     

    %% Plot body-fixed axes
    xpts = [po_p px_p];
    ypts = [po_p py_p];
    zpts = [po_p pz_p];   

    plot3(xpts(1,:),xpts(2,:),xpts(3,:),'r-','LineWidth',1);
    plot3(ypts(1,:),ypts(2,:),ypts(3,:),'g-','LineWidth',1);
    plot3(zpts(1,:),zpts(2,:),zpts(3,:),'b-','LineWidth',1);

   %% Figure settings
   axis([axis_min,axis_max,axis_min,axis_max,axis_min,axis_max]);

   xlabel('x^W');
   ylabel('y^W');
   zlabel('z^W');
   title(strcat('t = ',num2str(t(i),'%.2f'),' s'));
   
   if sideview == 'XZ'
        view([0 0]); %view XZ plane
   elseif sideview == 'YZ'
        view(90, 0); %view YZ plane
        
   elseif sideview == 'XY'
       view([0 0 1]);
       
   elseif sideview == 'V1'
       view([-14.5,6]);
       
   elseif sideview == 'V2'
       view([-19.5,28]);
   end
   
   grid on;
   axis square;
   
   drawnow;
   
   hold off;
end

end

