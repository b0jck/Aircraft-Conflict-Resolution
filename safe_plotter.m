clear all
clc 

%% Works with: (90,180); (181, 270). Singular for 180 and |phi| <= 90
phi0 = 180;
phi0 = deg2rad(phi0);
r = 5;
R = 5.5;
    
figure()
%grid on
hold on
% Plot q2 trajectory
semicircle = phase2_poly(r,R,phi0);
semicircle = semicircle.simplify();
pl1 = plot(semicircle,'DisplayName','W^1|q_2');
pl1.FaceColor = [0.9290 0.6940 0.1250];
axis equal
%legend  

pause

% Plot q3 trajectory 
delete(pl1)

L = R*sqrt(2-2*cos(phi0));
polyline = phase1_poly(r, 2*L, phi0);
polyline = polyline.simplify();
pl2 = plot(polyline, 'DisplayName','W^1|q_3');
pl2.FaceColor = [0 0.4470 0.7410];
%legend

pause

% Plot together
delete(pl2)
semicircle = rotate(semicircle, -90, [0,0]);
pl1 = plot(semicircle,'DisplayName','Rot[W_1 ^c|q_2]');
pl1.FaceColor = [0.9290 0.6940 0.1250];
pl2 = plot(polyline, 'DisplayName','CUPre[W_1 ^c|q_1],0');
pl2.FaceColor = [0 0.4470 0.7410];
%legend

% Plot Safe manouver
inter = intersect(semicircle,polyline);
safe = subtract(polyline, inter);
verts = safe.Vertices;
sigma_on = verts(2:length(verts)/2,:);
sigma_off = verts(length(verts)/2+2 : length(verts) , :);
pl3 = plot(safe, 'DisplayName','Safe Manouver Zone');
pl3.FaceColor = "#4DBEEE";
pl3.FaceAlpha = 1;
plot(sigma_on(:,1),sigma_on(:,2), 'LineWidth',3, 'Color',"#EDB120",...
    'DisplayName','\sigma on Zone')
plot(sigma_off(:,1),sigma_off(:,2), 'LineWidth',3, 'Color',"#A2142F", ...
    'DisplayName','\sigma forced Zone')


%%
function polygon = phase2_poly(r, R, phi0)
    t = 0.02:0.02:1;

    x1 = r*cos(pi/2+t*pi);
    y1 = r*sin(pi/2+t*pi);
    
    L = R*sqrt(2-2*cos(phi0));
    x2 = (L + r)*cos(3/2*pi+t*pi);
    y2 = L + (L + r)*sin(3/2*pi+t*pi);
    
    x3 = r*cos(pi/2+t*pi);
    y3 = 2*L +r*sin(pi/2+t*pi);
    
    x4 = (L - r)*cos(pi/2-t*pi);
    y4 = L + (L - r)*sin(pi/2-t*pi);
    
    xx = [x1 x2 x3 x4];
    yy = [y1 y2 y3 y4];

    pl = polyshape(xx,yy);
    polygon = rotate(pl, rad2deg((phi0-pi)/2), [0,0]);
end

function polyline = phase1_poly(r, L, phi0)
    t = 0.02:0.02:1;

    x1 = r*cos(pi/2+t*pi);
    y1 = r*sin(pi/2+t*pi);
    
    x2 = L*t;
    y2 = - r * ones(size(t));
    
    x3 = L + r*cos(-pi/2+t*pi);
    y3 = r*sin(-pi/2+t*pi);
    
    x4 = L - L*t;
    y4 = r * ones(size(t));
    
    xx = [x1 x2 x3 x4];
    yy = [y1 y2 y3 y4];

    pl = polyshape(xx,yy);
    polyline = rotate(pl, rad2deg((phi0-pi)/2), [0,0]);
end