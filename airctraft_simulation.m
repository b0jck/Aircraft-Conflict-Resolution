clear all
close all
clc
%% Simulation parameters
global v t_turn t_circle R
syms R t_man

% Velocity
v = 2;

% Initial Orientations
phi01 = input("Insert the orientation of the first aircraft -> ");
phi02 = input("Insert the orientation of the second aircraft -> ");
phi_r = phi02-phi01;

% Planes initial positions
x01 = 0;
y01 = 0;

% Collision time
tcoll = 10;

x02 = v*tcoll*(cos(phi01)-cos(phi02));
y02 = v*tcoll*(sin(phi01)-sin(phi02));

% Simulation time
simTim = tcoll+5;

% Safe radius
L = 5;

% Minimum time necessary for collision avoidance
tmax = L/v*(1/sqrt(2*(1-cos(phi_r))));
disp(strcat("Estimated collision time: ", num2str(tcoll)))
disp(strcat("Minimum time for collision avoidance: " , num2str(tcoll-tmax)))

%% Use backwards integration to solve xr^2+yr^2 >= 25
Rmin = solve(4*R^2 - 4*R*v*tmax + v^2*tmax^2 - L^2/(2*(1-cos(phi_r)))==0,R);
Rmin = double(max(Rmin));

disp(strcat("Minimum curvature radius: ", num2str(Rmin)))

decision = input("Decide if you want to assign the curvature radius (press 1) or the minimum starting manouver time (press 2) -> ");


if decision == 1
    R = input(strcat("Insert the curvature radius (>= ", num2str(Rmin), ") -> "));
    eq = 4*R^2 - 4*R*v*t_man + v^2*t_man^2 - L^2/(2*(1-cos(phi_r)));
    sol = solve(eq == 0, t_man);
    t_man = double(min(sol));
    t_decision = tcoll - t_man;
    t_decision = input(strcat("Insert starting manouver time between: [", num2str(t_decision), ";", num2str(tcoll-tmax), "] -> "));
elseif decision == 2
    t_decision = input(strcat("Insert the starting manouver time (<= ", num2str(tcoll-tmax), ") -> "));
    t_man = tcoll - t_decision;
    eq = 4*R^2 - 4*R*v*t_man + v^2*t_man^2 - L^2/(2*(1-cos(phi_r)));
    sol = solve(eq == 0, R);
    R = double(max(sol));
    R = input(strcat("Insert curvature radius (>= " , num2str(R), ") -> "));
else
    disp("Wrong input");
end

%% Consider now the two single dynamics for the two planes

% Set minimum time for starting circle manover
t_turn = t_decision;

% Circling time
t_circle = pi;

% Simulate phase 1: before Circle manouver
p01 = [x01;y01;phi01;x02;y02;phi02];
tspan1=0:0.01:t_turn;
[t1, p1]=ode45(@system1,tspan1,p01);

%Simulate phase 2: Circle manouver
p02 = [p1(end,1);p1(end,2);p1(end,3)-pi/2;p1(end,4);p1(end,5);p1(end,6)-pi/2];
tspan2=t_turn:0.01:(t_turn+t_circle);
[t2, p2]=ode45(@system2,tspan2,p02);

%Simulate phase 3: After Circle manouver
p03 = [p2(end,1);p2(end,2);p2(end,3)-pi/2;p2(end,4);p2(end,5);p2(end,6)-pi/2];
tspan3=(t_turn+t_circle):0.01:simTim;
[t3, p3]=ode45(@system1,tspan3,p03);

% Positions and orientation for t in [0,simTim]
p = [p1;p2;p3];

%% Video Animation
x1 = p(:,1);
y1 = p(:,2);
phi1 = p(:,3);
x2 = p(:,4);
y2 = p(:,5);
phi2 = p(:,6);

r = 2.5;

pmin = min(p);
xmin = min([pmin(1), pmin(4)])-(r+1);
ymin = -20;
%min([pmin(2), pmin(5)])-(r+1);

pmax = max(p);
xmax = max([pmax(1), pmax(4)])+(r+1);
ymax = 20;
%max([pmax(2), pmax(5)])+(r+1);



time = [tspan1, tspan2, tspan3];
fig = figure()
fig.WindowState = 'maximized';
hold on
axis equal
 

ax = gca;

h = hgtransform('Parent',ax);
j = hgtransform('Parent',ax);

air1 = rectangle('Position', [x01-r, y01-r, 2*r, 2*r],...
                      'Curvature', [1 1],...
                      'EdgeColor', 'k',...
                      'LineWidth', 1,...
                      'LineStyle', '-',...
                      'FaceColor', [0.9290 0.6940 0.1250 0.5],...
                      'Parent', h);

air2 = rectangle('Position', [x02-r, y02-r, 2*r, 2*r],...
                      'Curvature', [1 1],...
                      'EdgeColor', 'k',...
                      'LineWidth', 1,...
                      'LineStyle', '-',...
                      'FaceColor', [0 0.5 1 0.5],...
                      'Parent', j);

vec1 = quiver(x01, y01, x01+r*cos(phi1(1)), y01+r*sin(phi1(1)), 'Color', 'k', 'LineWidth', 2,'MaxHeadSize', 0.5);
vec2 = quiver(x02, y02, x02+r*cos(phi1(1)), y02+r*sin(phi2(1)), 'Color', 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);

timeval = text(xmin+2,ymax-0.5,strcat("time: ", num2str(time(1)), " s"),'Parent',j,...
'VerticalAlignment','top','FontSize',14);

d = sqrt((x02-x01)^2+(y02-y01)^2)-5;
distance = text(xmin+2,ymax-2.5,strcat("Distance: ", num2str(time(1)), " miles"),'Parent',j,...
'VerticalAlignment','top','FontSize',14);
for k = 2:length(x1)
    hold on
    t = time(k);
    d = sqrt((x2(k)-x1(k))^2+(y2(k)-y1(k))^2)-5;

    set(timeval,'String',strcat("Distance: ", num2str(d), " miles"),'Parent',j,...
        'VerticalAlignment','top','FontSize',14);
    set(distance,'String',strcat("time: ", num2str(t), " s"),'Parent',j,...
        'VerticalAlignment','top','FontSize',14);

    set(air1, 'EdgeColor', 'k', 'FaceColor', [0.9290 0.6940 0.1250 0.5], ...
        'Position', [x1(k)-r, y1(k)-r, 2*r, 2*r]);
    
    set(air2, 'EdgeColor', 'k', 'FaceColor', [0 0.5 1 0.5], ...
        'Position', [x2(k)-r, y2(k)-r, 2*r, 2*r]);
   
    % Update the aircraft orientation vector
    set(vec1, 'XData', x1(k), 'YData', y1(k), 'UData', r*cos(phi1(k)), 'VData',r*sin(phi1(k)) );
    set(vec2, 'XData', x2(k), 'YData', y2(k), 'UData', r*cos(phi2(k)), 'VData',r*sin(phi2(k)) );
   
    xlim([xmin, xmax]);
    ylim([ymin, ymax]);
    drawnow
end
%% Static Plot
figure()
plot(p(:,1),p(:,2),'.','Color','red','Linewidth',1.5)
hold on
plot(p(:,4),p(:,5),'.','Color','blue')
legend("Aircraft 1", "Aircraft 2")
axis("equal")

%% Dynamics simulation
function dp = system1(t,p)
global  v
phi1 = p(3);
phi2 = p(6);
dp = [v*cos(phi1); v*sin(phi1); 0; v*cos(phi2); v*sin(phi2); 0];
end

function dp = system2(t,p)
global R
v = 1*R;
phi1 = p(3);
phi2 = p(6);
dp = [v*cos(phi1); v*sin(phi1); 1; v*cos(phi2); v*sin(phi2); 1];
end