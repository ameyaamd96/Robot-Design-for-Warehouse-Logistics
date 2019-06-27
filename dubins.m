function [opt_dist] = dubins(x1,y1,theta1,x2,y2,theta2,rho,plot_flag)
% Calculates the length of the path for the mobile robot between two configurations
% Inputs: (x1,y1,theta1) and (x2,y2,theta2) and minimum turn radius 'rho'
% The input angles theta1 and theta2 are specified in radians
% Output is the length of the path from (x1,y1,theta1) to (x2,y2,theta2)
% If you also want to plot the path, use plot_flag = 1, else use plot_flag = 0


% Waqar A. Malik

t1 = theta1; t2 = theta2;

if nargin==7, plot_flag=0; end

% coordinate change
dx = x2 - x1;   dy = y2 - y1;   theta = mod2pi(atan2(dy,dx)); t1=mod2pi(t1); t2=mod2pi(t2);
x = cos(theta - t1)*sqrt(dx*dx+dy*dy);   y = sin(theta - t1)*sqrt(dx*dx+dy*dy);
phi = t2 - t1;

ap = rho*sin(phi);       am = -rho*sin(phi);
b1 = rho*(cos(phi)-1);   b2 = rho*(cos(phi)+1);

%   CCC
% curves is [ num,  length, tn, un,  vn ]
curves(1,:) = ccc(x,y,phi,ap,b1,1,rho);
curves(2,:) = ccc(x,-y,-phi,am,b1,2,rho);

%   C S C
curves(3,:)  = csca(x,y,phi,ap,b1,3,rho);
curves(4,:) = csca(x,-y,-phi,am,b1,4,rho);
curves(5,:) = cscb(x,y,phi,ap,b2,5,rho);
curves(6,:) = cscb(x,-y,-phi,am,b2,6,rho);

% Find optimal curve
[y,ind]=min(curves(:,2));
opt_curve=curves(ind,:);
opt_dist = opt_curve(2);
if plot_flag~=0 
    hold on;
    plot_dubins(opt_curve(1),opt_curve(3),opt_curve(4),opt_curve(5),x1,y1,t1,rho);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function curve = csca(x,y,phi,rs,rc,num,rho)
a = x-rs;
b = y+rc;
t = mod2pi((atan2(b,a)));
u = sqrt(a*a+b*b);
v = mod2pi(phi-t);

length_dubin = rho*(t+v) + u;
curve = [num length_dubin t u v];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function curve = cscb(x,y,phi,rs,rc,num,rho)
a = x+rs;
b = y-rc;
u1 = sqrt(a*a+b*b);
if (u1 < (rho*2)) 
    curve=[num inf inf inf inf]; return;
end
theta = mod2pi(atan2(b,a));
u = sqrt(u1*u1 -  (rho*2)^2 );
alpha = mod2pi(atan2((rho*2),u));
t = mod2pi(theta+alpha);
v = mod2pi(t-phi);

length_dubin = rho*(t+v) + u;
curve = [num length_dubin t u v];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function curve = ccc(x,y,phi,rs,rc,num,rho)
a = x-rs;   b = y+rc;
if ((abs(a)<eps) && (abs(b)<eps)) 
    curve=[num inf inf inf inf];
    return ;
end
u1 = sqrt(a*a+b*b);
if (u1>rho*4) 
    curve=[num inf inf inf inf]; return;
end
theta = mod2pi(atan2(b,a));
alpha = acos(u1/(rho*4));
t = mod2pi((pi/2) + alpha + theta); 
u = mod2pi((pi)+2*alpha);
v = mod2pi(phi-t+u);
length_dubin = rho*(t+u+v);
curve = [num length_dubin t u v];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plot_dubins(num,t,u,v,x1,y1,t1,rho)   

right = 1; left = 2; straight = 3;

switch(num)
%   C C C 
    case 1 
         [x, y, th] = draw_segment(left,t,x1,y1,t1,rho);
         [x, y, th] = draw_segment(right,u,x,y,th,rho);
         [x, y, th] = draw_segment(left,v,x,y,th,rho);
    case 2
         [x, y, th] = draw_segment(right,t,x1,y1,t1,rho);
         [x, y, th] = draw_segment(left,u,x,y,th,rho);
         [x, y, th] = draw_segment(right,v,x,y,th,rho);
         
%   C S C   
    case 3 
         [x, y, th] = draw_segment(left,t,x1,y1,t1,rho);
         [x, y, th] = draw_segment(straight,u,x,y,th,rho);
         [x, y, th] = draw_segment(left,v,x,y,th,rho);
    case 4
         [x, y, th] = draw_segment(right,t,x1,y1,t1,rho);
         [x, y, th] = draw_segment(straight,u,x,y,th,rho);
         [x, y, th] = draw_segment(right,v,x,y,th,rho);
    case 5 
         [x, y, th] = draw_segment(left,t,x1,y1,t1,rho);
         [x, y, th] = draw_segment(straight,u,x,y,th,rho);
         [x, y, th] = draw_segment(right,v,x,y,th,rho);
    case 6
         [x, y, th] = draw_segment(right,t,x1,y1,t1,rho);
         [x, y, th] = draw_segment(straight,u,x,y,th,rho);
         [x, y, th] = draw_segment(left,v,x,y,th,rho);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x, y, th] = draw_segment(ty,val,x1,y1,t1,rho)

switch(ty)
    case 1 % circular arc toward the right
        xc = x1 + rho*sin(t1);
        yc = y1 - rho*cos(t1);
        plot(xc+rho*cos(0:.1:2*pi),yc+rho*sin(0:.1:2*pi),'r'); plot_arrow(x1,y1, x1+rho/2*cos(t1),y1+rho/2*sin(t1),'color','g','facecolor','g','edgecolor','g');
        va1 = t1+(pi/2);
        va2 = va1-val;
        x = xc + rho*cos(va2);
        y = yc + rho*sin(va2);
        th = t1-val;
        if va1 < va2
            plot(xc+rho*cos(va1:.1:va2),yc+rho*sin(va1:.1:va2),'b','LineWidth',2);
        else
            plot(xc+rho*cos(va1:-.1:va2),yc+rho*sin(va1:-.1:va2),'b','LineWidth',2);
        end
        plot_arrow(x,y, x+rho/2*cos(va2-pi/2),y+rho/2*sin(va2-pi/2),'color','g','facecolor','g','edgecolor','g');   
    case 2 % circular arc toward the left
        xc = x1 - rho*sin(t1);
        yc = y1 + rho*cos(t1);
        plot(xc+rho*cos(0:.1:2*pi),yc+rho*sin(0:.1:2*pi),'r'); plot_arrow(x1,y1, x1+rho/2*cos(t1),y1+rho/2*sin(t1),'color','g','facecolor','g','edgecolor','g');
        va1 = t1-(pi/2);
        va2 = va1+val;
        x = xc + rho*cos(va2);
        y = yc + rho*sin(va2);
        th = t1+val;
        if va1 < va2
            plot(xc+rho*cos(va1:.1:va2),yc+rho*sin(va1:.1:va2),'b','LineWidth',2);
        else
            plot(xc+rho*cos(va1:-.1:va2),yc+rho*sin(va1:-.1:va2),'b','LineWidth',2);
        end
        plot_arrow(x,y, x+rho/2*cos(va2+pi/2),y+rho/2*sin(va2+pi/2),'color','g','facecolor','g','edgecolor','g');   
    case 3 %    straight line
        x = x1+val*cos(t1);
        y = y1+val*sin(t1);
        t1 = mod2pi(t1);
        th = t1;
        plot([x1 x],[y1 y],'b','LineWidth',2);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function angle = mod2pi(angle)
% Coverts to [0, 2pi]   
angle = mod(angle,2*pi);
% while (angle < 0.0) 
%     angle = angle + 2*pi;
% end
% while (angle >= (pi*2)) 
%     angle = angle - 2*pi;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%