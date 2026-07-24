clear all;
close all;
clc;

%% CINEMATICA INVERSA DE UN ROBOT DIFERENCIAL

%% CONDICIONES INICIALES

x(1)=0;
y(1)=0;
theta(1)=pi/2;

%% TIEMPO

ts=16;
ti=0.01;
t=0:ti:ts;

%% PARAMETROS DEL ROBOT

L=0.2;
r=0.04;

%% INICIALIZACION

u=zeros(size(t));
w=zeros(size(t));

VD=zeros(size(t));
VI=zeros(size(t));

thetaDot_D=zeros(size(t));
thetaDot_I=zeros(size(t));

RPM_D=zeros(size(t));
RPM_I=zeros(size(t));

%% TRAYECTORIA CIRCULAR DESEADA

R=2;
omega_c=0.4;

xd=R*cos(omega_c*t);
yd=R*sin(omega_c*t);

x_punto=-R*omega_c*sin(omega_c*t);
y_punto= R*omega_c*cos(omega_c*t);

theta_punto=omega_c*ones(size(t));

%CINEMATICA INVERSA
% Velocidad lineal y angular del robot
for k = 1:length(t)-1

    u(k) = x_punto(k)*cos(theta_punto(k)) + y_punto(k)*sin(theta_punto(k));

    w(k) = theta_punto(k);

    VD(k) = u(k) + (L/2)*w(k);
    VI(k) = u(k) - (L/2)*w(k);

    thetaDot_D(k) = VD(k)/r;
    thetaDot_I(k) = VI(k)/r;

    RPM_D(k) = 60*thetaDot_D(k)/(2*pi);
    RPM_I(k) = 60*thetaDot_I(k)/(2*pi);
% Integración por Euler
    x(k+1) = x(k) + ti*x_punto(k);
    y(k+1) = y(k) + ti*y_punto(k);
    theta(k+1) = theta(k) + ti*theta_punto(k);

end

u(end) = u(end-1);
w(end) = w(end-1);

VD(end) = VD(end-1);
VI(end) = VI(end-1);

thetaDot_D(end) = thetaDot_D(end-1);
thetaDot_I(end) = thetaDot_I(end-1);

RPM_D(end) = RPM_D(end-1);
RPM_I(end) = RPM_I(end-1);
%% COMPLETAR EL ÚLTIMO VALOR
%% COLOR DE LAS GRAFICAS
rosa = [0.96 0.55 0.73];

%% FIGURA 1. VELOCIDADES DESEADAS

figure(1)

subplot(3,1,1)
plot(t,x_punto,'Color',rosa,'LineWidth',3)
grid on
title('Velocidad deseada en X')
xlabel('Tiempo [s]')
ylabel('\dot{x} [m/s]','Interpreter','latex')
set(gca,'FontSize',14)

subplot(3,1,2)
plot(t,y_punto,'Color',rosa,'LineWidth',3)
grid on
title('Velocidad deseada en Y')
xlabel('Tiempo [s]')
ylabel('\dot{y} [m/s]','Interpreter','latex')
set(gca,'FontSize',14)

subplot(3,1,3)
plot(t,theta_punto,'Color',rosa,'LineWidth',3)
grid on
title('Velocidad angular deseada')
xlabel('Tiempo [s]')
ylabel('\dot{\theta} [rad/s]','Interpreter','latex')
set(gca,'FontSize',14)

exportgraphics(gcf,'velocidades_deseadas.png','Resolution',300)

%% FIGURA 2. VELOCIDAD DEL ROBOT

figure(2)

subplot(2,1,1)
plot(t,u,'Color',rosa,'LineWidth',3)
grid on
title('Velocidad lineal del robot')
xlabel('Tiempo [s]')
ylabel('u [m/s]')
set(gca,'FontSize',14)

subplot(2,1,2)
plot(t,w,'Color',rosa,'LineWidth',3)
grid on
title('Velocidad angular del robot')
xlabel('Tiempo [s]')
ylabel('\omega [rad/s]','Interpreter','latex')
set(gca,'FontSize',14)

exportgraphics(gcf,'velocidad_robot.png','Resolution',300)

%% FIGURA 3. TRAYECTORIA DESEADA

figure(3)

plot(xd,yd,'Color',rosa,'LineWidth',3)
hold on

plot(xd(1),yd(1),'go',...
    'MarkerSize',10,...
    'MarkerFaceColor','g')

plot(xd(end),yd(end),'rs',...
    'MarkerSize',10,...
    'MarkerFaceColor','r')

grid on
axis equal

title('Trayectoria deseada')
xlabel('X [m]')
ylabel('Y [m]')

legend('Trayectoria','Inicio','Final')

set(gca,'FontSize',14)

exportgraphics(gcf,'trayectoria_deseada.png','Resolution',300)

%% FIGURA 4. TRAYECTORIA OBTENIDA

figure(4)
plot(x,y,'Color',rosa,'LineWidth',3)

hold on

plot(x(1),y(1),'go',...
    'MarkerSize',10,...
    'MarkerFaceColor','g')

plot(x(end),y(end),'rs',...
    'MarkerSize',10,...
    'MarkerFaceColor','r')

grid on
axis equal

title('Trayectoria obtenida')
xlabel('X [m]')
ylabel('Y [m]')

legend('Trayectoria','Inicio','Final')

set(gca,'FontSize',14)

exportgraphics(gcf,'trayectoria_obtenida.png','Resolution',300)
%% FIGURA 5. COMPARACION DE TRAYECTORIAS

figure(5)

plot(xd,yd,'Color',rosa,'LineWidth',3)
hold on

plot(x,y,'--k','LineWidth',2)

grid on
axis equal

title('Comparacion de trayectorias')
xlabel('X [m]')
ylabel('Y [m]')

legend('Trayectoria deseada','Trayectoria obtenida','Location','best')

set(gca,'FontSize',14)

exportgraphics(gcf,'comparacion_trayectorias.png','Resolution',300)
%% FIGURA 6. VELOCIDAD LINEAL

figure(6)

plot(t,u,'Color',rosa,'LineWidth',3)

grid on

title('Velocidad lineal')
xlabel('Tiempo [s]')
ylabel('u [m/s]')

set(gca,'FontSize',14)

exportgraphics(gcf,'velocidad_lineal.png','Resolution',300)
%% FIGURA 7. VELOCIDAD ANGULAR

figure(7)

plot(t,w,'Color',rosa,'LineWidth',3)

grid on

title('Velocidad angular')
xlabel('Tiempo [s]')
ylabel('\omega [rad/s]','Interpreter','tex')

set(gca,'FontSize',14)

exportgraphics(gcf,'velocidad_angular.png','Resolution',300)
%% FIGURA 8. VELOCIDADES DE LAS RUEDAS

figure(8)

%=========================================
% VELOCIDADES LINEALES
%=========================================
subplot(2,1,1)

plot(t,VD,'Color',rosa,'LineWidth',3)
hold on
plot(t,VI,'--','Color',[0.75 0.15 0.50],'LineWidth',3)

grid on

title('Velocidades lineales de las ruedas')
xlabel('Tiempo [s]')
ylabel('Velocidad [m/s]')

legend('Rueda derecha','Rueda izquierda','Location','best')

set(gca,'FontSize',12)

%=========================================
% VELOCIDADES ANGULARES
%=========================================
subplot(2,1,2)

plot(t,thetaDot_D,'Color',rosa,'LineWidth',3)
hold on
plot(t,thetaDot_I,'--','Color',[0.75 0.15 0.50],'LineWidth',3)

grid on

title('Velocidades angulares de las ruedas')
xlabel('Tiempo [s]')
ylabel('\omega [rad/s]')

legend('Rueda derecha','Rueda izquierda','Location','best')

set(gca,'FontSize',12)

exportgraphics(gcf,'velocidades_ruedas.png','Resolution',300)
%% FIGURA 9. ANIMACION DE LA TRAYECTORIA

figure(9)
clf
set(gcf,'Position',[150 80 900 700])

% Calcular límites automáticamente
xmin = min([x xd]) - 0.5;
xmax = max([x xd]) + 0.5;
ymin = min([y yd]) - 0.5;
ymax = max([y yd]) + 0.5;

filename = 'animacion_robot.gif';

for k = 1:5:length(t)

    clf
    hold on

    % Trayectoria completa (gris)
    plot(xd,yd,'--',...
        'Color',[0.7 0.7 0.7],...
        'LineWidth',2);

    % Trayectoria recorrida
    plot(xd(1:k),yd(1:k),...
        'Color',rosa,...
        'LineWidth',3);

    % Robot
    plot(xd(k),yd(k),...
        'o',...
        'MarkerFaceColor',rosa,...
        'MarkerEdgeColor',rosa,...
        'MarkerSize',14);

    % Flecha de orientación
    quiver(xd(k),yd(k),...
        0.25*cos(theta(k)),...
        0.25*sin(theta(k)),...
        0,...
        'Color',[0.75 0.15 0.50],...
        'LineWidth',2,...
        'MaxHeadSize',2);

    axis equal
    axis([xmin xmax ymin ymax])

    grid on
    box on

    xlabel('X [m]')
    ylabel('Y [m]')
    title('Animación de la trayectoria')

    drawnow

    % Guardar GIF
    frame = getframe(gcf);
    im = frame2im(frame);
    [A,map] = rgb2ind(im,256);

    if k==1
        imwrite(A,map,filename,...
            'gif',...
            'LoopCount',Inf,...
            'DelayTime',0.05);
    else
        imwrite(A,map,filename,...
            'gif',...
            'WriteMode','append',...
            'DelayTime',0.05);
    end

end
