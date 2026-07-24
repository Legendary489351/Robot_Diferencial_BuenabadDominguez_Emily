%% TRAYECTORIA LIBRE - FLOR 
% Proyecto Final ¿
% Esta versión genera una trayectoria tipo "flor" únicamente
% modificando las velocidades angulares de las ruedas por intervalos.

clear; clc; close all;

%% Parámetros
L = 0.20;
r = 0.04;

ti = 0.01;
ts = 24;
t = 0:ti:ts;

x=zeros(size(t));
y=zeros(size(t));
theta=zeros(size(t));

%% Intervalos
% [ti tf thetaD thetaI]
tabla = [
 0  1   12  7;
 1  2    7 12;
 2  3   12 -8;
 3  4   12  7;
 4  5    7 12;
 5  6   12 -8;
 6  7   12  7;
 7  8    7 12;
 8  9   12 -8;
 9 10   12  7;
10 11    7 12;
11 12   12 -8;
12 13   12  7;
13 14    7 12;
14 15   12 -8;
15 16   12  7;
16 17    7 12;
17 18   12 -8;
18 21   10 10;
21 22    8 12;
22 23   12  8;
23 24   10 10;
];

%% Simulación

for k=1:length(t)-1

    td=10;
    tiw=10;

    for j=1:size(tabla,1)
        if t(k)>=tabla(j,1) && t(k)<tabla(j,2)
            td=tabla(j,3);
            tiw=tabla(j,4);
            break
        end
    end

    u=(r/2)*(td+tiw);
    w=(r/L)*(td-tiw);

    x(k+1)=x(k)+ti*u*cos(theta(k));
    y(k+1)=y(k)+ti*u*sin(theta(k));
    theta(k+1)=theta(k)+ti*w;
    %%

end

%%================== ANIMACION ==================%

figure('Color','w');
hold on;
grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');
title('Trayectoria libre - Flor');

% Ajustar la vista automáticamente
margen = 0.2;

xmin = min(x)-margen;
xmax = max(x)+margen;
ymin = min(y)-margen;
ymax = max(y)+margen;

axis([xmin xmax ymin ymax]);

% Trayectoria (rosa)
trayectoria = plot(NaN,NaN,...
    'Color',[1 0.2 0.7],...
    'LineWidth',2);

% Robot (punto negro)
robot = plot(NaN,NaN,...
    'ko',...
    'MarkerFaceColor','k',...
    'MarkerSize',8);

% Texto
datos = text(...
    xmin,...
    ymax,...
    '',...
    'FontSize',11,...
    'FontWeight','bold',...
    'VerticalAlignment','top');
%%================== VIDEO ==================%

video = VideoWriter('Trayectoria_Flor.mp4','MPEG-4');
video.FrameRate = 30;

open(video);


%%================== ANIMACION ==================%

for k = 1:length(t)

    % Buscar velocidades correspondientes
    thetaD = 10;
    thetaI = 10;

    for j = 1:size(tabla,1)

        if t(k)>=tabla(j,1) && t(k)<tabla(j,2)

            thetaD = tabla(j,3);
            thetaI = tabla(j,4);
            break

        end

    end

    % Velocidades del robot
    u = (r/2)*(thetaD+thetaI);
    w = (r/L)*(thetaD-thetaI);

    % Actualizar trayectoria
    set(trayectoria,...
        'XData',x(1:k),...
        'YData',y(1:k));

    % Actualizar robot
    set(robot,...
        'XData',x(k),...
        'YData',y(k));

    % Actualizar texto
    texto = sprintf([...
        'Tiempo = %.2f s\n\n',...
        '\\theta_D = %.2f rad/s\n',...
        '\\theta_I = %.2f rad/s\n\n',...
        'u = %.3f m/s\n',...
        '\\omega = %.3f rad/s'],...
        t(k),...
        thetaD,...
        thetaI,...
        u,...
        w);

    set(datos,'String',texto);

    drawnow;

    frame = getframe(gcf);

    writeVideo(video,frame);

end

close(video);
