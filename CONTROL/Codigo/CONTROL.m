%% INICIALIZACION

clear all;
clc;
close all;

%------------------------------------------------------------
% CONTROL CINEMATICO DE UN ROBOT DIFERENCIAL
% REGULACION DE PUNTO
%
% Simulacion 1 : Sin saturacion
% Simulacion 2 : Con saturacion proporcional
%------------------------------------------------------------

%% TIEMPO DE SIMULACION

ts = 20;                 % Tiempo total [s]
ti = 0.01;               % Tiempo de integracion [s]
t = 0:ti:ts;

%% PARAMETROS DEL ROBOT

L = 0.18;                % Distancia entre ruedas [m]
r = 0.03;                % Radio de las ruedas [m]

%% POSICION DESEADA

xd = 5;
yd = 5;

%% GANANCIAS DEL CONTROLADOR

k_rho = 0.3;
k_theta = 4;

thetaDot_max = 20;       % Velocidad angular maxima [rad/s]
tolerancia = 0.03;       % Distancia minima a la meta

%% CONDICIONES INICIALES

x0 = 0;
y0 = 0;
theta0 = 0;

%% VARIABLES PARA SIMULACION SIN SATURACION

x_ns = zeros(1,length(t));
y_ns = zeros(1,length(t));
theta_ns = zeros(1,length(t));

x_ns(1) = x0;
y_ns(1) = y0;
theta_ns(1) = theta0;

rho_ns = zeros(1,length(t));
e_theta_ns = zeros(1,length(t));

u_ns = zeros(1,length(t));
w_ns = zeros(1,length(t));

vD_ns = zeros(1,length(t));
vI_ns = zeros(1,length(t));

thetaDot_D_ns = zeros(1,length(t));
thetaDot_I_ns = zeros(1,length(t));
%% SIMULACION SIN SATURACION

for k = 1:length(t)-1

    % ERRORES DE POSICION
    ex = xd - x_ns(k);
    ey = yd - y_ns(k);

    % Distancia hacia la meta
    rho_ns(k) = sqrt(ex^2 + ey^2);

    % Angulo deseado
    theta_d = atan2(ey,ex);

    % Error angular
    e_theta_ns(k) = atan2(sin(theta_d-theta_ns(k)),...
        cos(theta_d-theta_ns(k)));

    % CONTROL CINEMATICO
    u_ns(k) = k_rho*rho_ns(k);
    w_ns(k) = k_theta*e_theta_ns(k);

    % DETENER EL ROBOT SI LLEGA A LA META
    if rho_ns(k) < tolerancia
        u_ns(k) = 0;
        w_ns(k) = 0;
    end

    % CINEMATICA INVERSA
    vD_ns(k) = u_ns(k) + (L/2)*w_ns(k);
    vI_ns(k) = u_ns(k) - (L/2)*w_ns(k);

    thetaDot_D_ns(k) = vD_ns(k)/r;
    thetaDot_I_ns(k) = vI_ns(k)/r;

    % MODELO CINEMATICO
    x_ns(k+1) = x_ns(k) + ti*u_ns(k)*cos(theta_ns(k));
    y_ns(k+1) = y_ns(k) + ti*u_ns(k)*sin(theta_ns(k));
    theta_ns(k+1) = theta_ns(k) + ti*w_ns(k);

end

%% COMPLETAR EL ULTIMO VALOR

rho_ns(end) = rho_ns(end-1);
e_theta_ns(end) = e_theta_ns(end-1);

u_ns(end) = u_ns(end-1);
w_ns(end) = w_ns(end-1);

vD_ns(end) = vD_ns(end-1);
vI_ns(end) = vI_ns(end-1);

thetaDot_D_ns(end) = thetaDot_D_ns(end-1);
thetaDot_I_ns(end) = thetaDot_I_ns(end-1);
%% VARIABLES PARA SIMULACION CON SATURACION

x_sat = zeros(1,length(t));
y_sat = zeros(1,length(t));
theta_sat = zeros(1,length(t));

x_sat(1) = x0;
y_sat(1) = y0;
theta_sat(1) = theta0;

rho_sat = zeros(1,length(t));
e_theta_sat = zeros(1,length(t));

u_sat = zeros(1,length(t));
w_sat = zeros(1,length(t));

u_real = zeros(1,length(t));
w_real = zeros(1,length(t));

vD_sat = zeros(1,length(t));
vI_sat = zeros(1,length(t));

thetaDot_D = zeros(1,length(t));
thetaDot_I = zeros(1,length(t));

thetaDot_D_sat = zeros(1,length(t));
thetaDot_I_sat = zeros(1,length(t));

valor_maximo = zeros(1,length(t));
%% SIMULACION CON SATURACION

for k = 1:length(t)-1

    % ERRORES DE POSICION
    ex = xd - x_sat(k);
    ey = yd - y_sat(k);

    % Distancia hacia la meta
    rho_sat(k) = sqrt(ex^2 + ey^2);

    % Angulo deseado
    theta_d = atan2(ey,ex);

    % Error angular
    e_theta_sat(k)=atan2(sin(theta_d-theta_sat(k)),...
        cos(theta_d-theta_sat(k)));

    % CONTROL CINEMATICO
    u_sat(k)=k_rho*rho_sat(k);
    w_sat(k)=k_theta*e_theta_sat(k);

    % DETENER EL ROBOT
    if rho_sat(k)<tolerancia
        u_sat(k)=0;
        w_sat(k)=0;
    end

    % CINEMATICA INVERSA
    thetaDot_D(k)=(u_sat(k)+(L/2)*w_sat(k))/r;
    thetaDot_I(k)=(u_sat(k)-(L/2)*w_sat(k))/r;

    % SATURACION PROPORCIONAL
    valor_maximo(k)=max(abs([thetaDot_D(k) thetaDot_I(k)]));

    if valor_maximo(k)>thetaDot_max

        alpha=thetaDot_max/valor_maximo(k);

        thetaDot_D_sat(k)=alpha*thetaDot_D(k);
        thetaDot_I_sat(k)=alpha*thetaDot_I(k);

    else

        thetaDot_D_sat(k)=thetaDot_D(k);
        thetaDot_I_sat(k)=thetaDot_I(k);

    end

    % VELOCIDADES REALES DEL ROBOT
    u_real(k)=(r/2)*(thetaDot_D_sat(k)+thetaDot_I_sat(k));
    w_real(k)=(r/L)*(thetaDot_D_sat(k)-thetaDot_I_sat(k));

    % VELOCIDADES LINEALES DE RUEDAS
    vD_sat(k)=r*thetaDot_D_sat(k);
    vI_sat(k)=r*thetaDot_I_sat(k);

    % MODELO CINEMATICO
    x_sat(k+1)=x_sat(k)+ti*u_real(k)*cos(theta_sat(k));
    y_sat(k+1)=y_sat(k)+ti*u_real(k)*sin(theta_sat(k));
    theta_sat(k+1)=theta_sat(k)+ti*w_real(k);

end
%% COMPLETAR EL ULTIMO VALOR

rho_sat(end)=rho_sat(end-1);
e_theta_sat(end)=e_theta_sat(end-1);

u_sat(end)=u_sat(end-1);
w_sat(end)=w_sat(end-1);

u_real(end)=u_real(end-1);
w_real(end)=w_real(end-1);

vD_sat(end)=vD_sat(end-1);
vI_sat(end)=vI_sat(end-1);

thetaDot_D(end)=thetaDot_D(end-1);
thetaDot_I(end)=thetaDot_I(end-1);

thetaDot_D_sat(end)=thetaDot_D_sat(end-1);
thetaDot_I_sat(end)=thetaDot_I_sat(end-1);
%% COLORES

rosa = [1.00 0.40 0.70];
morado = [0.55 0.25 0.80];
%% FIGURA 1

figure(1)

plot(x_ns,y_ns,'Color',rosa,'LineWidth',4)
hold on

plot(x_ns(1),y_ns(1),'go',...
    'MarkerSize',12,...
    'MarkerFaceColor','g')

plot(xd,yd,'rp',...
    'MarkerSize',18,...
    'MarkerFaceColor','r')

plot(x_ns(end),y_ns(end),'ks',...
    'MarkerSize',12,...
    'MarkerFaceColor',morado)

xlabel('Posición X [m]')
ylabel('Posición Y [m]')

title('Trayectoria de regulación de punto')

legend('Trayectoria',...
    'Inicio',...
    'Meta',...
    'Final',...
    'Location','best')

grid on
axis equal

set(gca,'FontSize',16)
%% FIGURA 2

figure(2)

plot(x_sat,y_sat,'Color',morado,'LineWidth',3)
hold on
plot(x_sat(1),y_sat(1),'o','Color','g','MarkerFaceColor','g','MarkerSize',10)
plot(xd,yd,'p','Color','r','MarkerFaceColor','r','MarkerSize',14)
plot(x_sat(end),y_sat(end),'s','Color',rosa,'MarkerFaceColor',rosa,'MarkerSize',10)

xlabel('Posición X [m]')
ylabel('Posición Y [m]')
title('Trayectoria con saturación')
legend('Trayectoria','Inicio','Meta','Final','Location','best')

grid on
axis equal
set(gca,'FontSize',16)

%% FIGURA 3

figure(3)

plot(x_ns,y_ns,'Color',rosa,'LineWidth',3)
hold on

plot(x_sat,y_sat,'Color',morado,'LineWidth',3)

plot(xd,yd,'rp','MarkerSize',16,'MarkerFaceColor','r')

plot(x0,y0,'go','MarkerSize',10,'MarkerFaceColor','g')

xlabel('Posición X [m]')
ylabel('Posición Y [m]')
title('Comparación de trayectorias')

legend('Sin saturación','Con saturación','Meta','Inicio','Location','best')

grid on
axis equal
set(gca,'FontSize',16)
%% FIGURA 4

figure(4)

plot(t,rho_ns,'Color',rosa,'LineWidth',3)
hold on

plot(t,rho_sat,'Color',morado,'LineWidth',3)

xlabel('Tiempo [s]')
ylabel('\rho [m]')

title('Error de posición')

legend('Sin saturación','Con saturación','Location','best')

grid on
set(gca,'FontSize',16)

%% FIGURA 5

figure(5)

plot(t,e_theta_ns,'Color',rosa,'LineWidth',3)
hold on

plot(t,e_theta_sat,'Color',morado,'LineWidth',3)

xlabel('Tiempo [s]')
ylabel('e_{\theta} [rad]')

title('Error angular')

legend('Sin saturación','Con saturación','Location','best')

grid on
set(gca,'FontSize',16)


%% FIGURA 6

figure(6)

plot(t,u_ns,'Color',rosa,'LineWidth',3)
hold on

plot(t,u_real,'Color',morado,'LineWidth',3)

xlabel('Tiempo [s]')
ylabel('u [m/s]')

title('Velocidad lineal')

legend('Sin saturación','Con saturación','Location','best')

grid on
set(gca,'FontSize',16)
%% FIGURA 7

figure(7)

plot(t,w_ns,'Color',rosa,'LineWidth',3)
hold on

plot(t,w_real,'Color',morado,'LineWidth',3)

xlabel('Tiempo [s]')
ylabel('w [rad/s]')

title('Velocidad angular')

legend('Sin saturación','Con saturación','Location','best')

grid on
set(gca,'FontSize',16)
%% FIGURA 8

figure(8)

subplot(2,1,1)

plot(t,thetaDot_D_ns,'Color',rosa,'LineWidth',3)
hold on
plot(t,thetaDot_D_sat,'Color',morado,'LineWidth',3)

ylabel('\omega_D [rad/s]')

title('Rueda derecha')

legend('Sin saturación','Con saturación')

grid on

subplot(2,1,2)

plot(t,thetaDot_I_ns,'Color',rosa,'LineWidth',3)
hold on
plot(t,thetaDot_I_sat,'Color',morado,'LineWidth',3)

ylabel('\omega_I [rad/s]')
xlabel('Tiempo [s]')

title('Rueda izquierda')

legend('Sin saturación','Con saturación')

grid on

set(gca,'FontSize',16)
figure(9)

filename='control_cinematico.gif';

for k=1:5:length(t)

    clf

    plot(x_sat,y_sat,'Color',[0.8 0.8 0.8],'LineWidth',2)
    hold on

    plot(x_sat(1),y_sat(1),...
        'go',...
        'MarkerFaceColor','g',...
        'MarkerSize',10)

    plot(xd,yd,...
        'rp',...
        'MarkerFaceColor','r',...
        'MarkerSize',16)

    plot(x_sat(1:k),y_sat(1:k),...
        'Color',morado,...
        'LineWidth',4)

    plot(x_sat(k),y_sat(k),...
        'o',...
        'MarkerFaceColor',rosa,...
        'MarkerEdgeColor',morado,...
        'MarkerSize',12)

    xlabel('Posición X [m]')
    ylabel('Posición Y [m]')
    title('Control cinemático')

    axis equal
    grid on

    xlim([-0.5 5.5])
    ylim([-0.5 5.5])

    drawnow

    frame=getframe(gcf);
    im=frame2im(frame);
    [A,map]=rgb2ind(im,256);

    if k==1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.05);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.05);
    end

end
