%% CINEMÁTICA DIRECTA - ROBOT DIFERENCIAL
clear all;
clc;
close all;

%% PARÁMETROS DEL ROBOT

ts = 5;              % Tiempo de simulación [s]
ti = 0.01;           % Tiempo de integración [s]
t = 0:ti:ts;

L = 0.20;            % Distancia entre ruedas [m]
r = 0.04;            % Radio de la rueda [m]

%% CONDICIONES INICIALES

x(1) = 0;
y(1) = 0;
theta(1) = 0;

%% SELECCIÓN DEL CASO

caso = 8;


    switch caso

        case 1
            thetaDot_D = 10;
            thetaDot_I = 10;
            movimiento = 'Linea recta';

        case 2
            thetaDot_D = -10;
            thetaDot_I = -10;
            movimiento = 'Linea recta hacia atras';

        case 3
            thetaDot_D = 12;
            thetaDot_I = 6;
            movimiento = 'Curva izquierda';

        case 4
            thetaDot_D = 6;
            thetaDot_I = 12;
            movimiento = 'Curva derecha';

        case 5
            thetaDot_D = 10;
            thetaDot_I = -10;
            movimiento = 'Giro sobre su propio eje';

        case 6
            thetaDot_D = 10;
            thetaDot_I = 0;
            movimiento = 'Curva con rueda izquierda detenida';

        case 7
            thetaDot_D = 10;
            thetaDot_I = 8;
            movimiento = 'Curva suave izquierda';

        case 8
            thetaDot_D = 10;
            thetaDot_I = 2;
            movimiento = 'Curva pronunciada izquierda';

    end

%% CINEMÁTICA DIRECTA

u = (r/2)*(thetaDot_D + thetaDot_I);
w = (r/L)*(thetaDot_D - thetaDot_I);

for k = 1:length(t)-1

    x(k+1) = x(k) + ti*u*cos(theta(k));
    y(k+1) = y(k) + ti*u*sin(theta(k));
    theta(k+1) = theta(k) + ti*w;

end

%% ANIMACIÓN

figure('Color','w');
hold on;
grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');

title(['Caso ',num2str(caso),' - ',movimiento])

xlim([min(x)-0.5 max(x)+0.5]);
ylim([min(y)-0.5 max(y)+0.5]);

nombreVideo = ['Caso',num2str(caso),'.mp4'];

video = VideoWriter(nombreVideo,'MPEG-4');
video.FrameRate = 30;
open(video);

for k = 1:3:length(t)

    cla

    plot(x,y,'k--','LineWidth',1)

    plot(x(1),y(1),'go','MarkerFaceColor','g','MarkerSize',8)

    plot(x(k),y(k),'bo','MarkerFaceColor','b','MarkerSize',10)

    quiver(x(k),y(k),...
           0.15*cos(theta(k)),...
           0.15*sin(theta(k)),...
           0,...
           'r','LineWidth',2);

    grid on
    axis equal

    xlim([min(x)-0.5 max(x)+0.5]);
    ylim([min(y)-0.5 max(y)+0.5]);

    xlabel('X [m]')
    ylabel('Y [m]')

    title(['Caso ',num2str(caso),' - ',movimiento])

    drawnow

    frame = getframe(gcf);
    writeVideo(video,frame);

end

%% MANTENER EL ÚLTIMO CUADRO 1 SEGUNDO

for i = 1:30

    frame = getframe(gcf);
    writeVideo(video,frame);

end

close(video);

%% RESULTADOS

fprintf('\n');
fprintf('-----------------------------\n');
fprintf('CASO %d\n',caso);
fprintf('Movimiento: %s\n',movimiento);
fprintf('u = %.3f m/s\n',u);
fprintf('w = %.3f rad/s\n',w);
fprintf('x final = %.3f m\n',x(end));
fprintf('y final = %.3f m\n',y(end));
fprintf('theta final = %.3f rad\n',theta(end));
fprintf('-----------------------------\n');
