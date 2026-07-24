# Robot_Diferencial_BuenabadDominguez_Emily
CINEMÁTICA DIRECTA, CINEMÁTICA INVERSA y CONTROL.
Parámetros utilizados

Para la simulación del control cinemático de un robot diferencial se emplearon los siguientes parámetros de operación y configuración:

• Radio de la rueda (r): 0.04 m.
• Distancia entre ruedas (L): 0.20 m.
• Tiempo total de simulación (T): 40 s.
• Paso de integración (Δt): 0.01 s.
• Velocidad angular máxima de las ruedas (θ̇max): 15 rad/s.
• Condiciones iniciales del robot: posición inicial x0 = 0 m, y0 = 0 m y orientación inicial θ0 = π/2 rad.
• Radio de la trayectoria circular (R): 1.2 m.
• Velocidad angular de la trayectoria (ωc): 0.3 rad/s.
• Punto objetivo: (xd, yd) = (4,4) m.

Los parámetros anteriores fueron establecidos para definir las características físicas del robot diferencial, las condiciones iniciales de la simulación y las restricciones de operación. Estos valores permitieron evaluar el desempeño del controlador cinemático tanto en el seguimiento de trayectorias como en la regulación hacia un punto objetivo, considerando el efecto de la saturación en la velocidad angular de las ruedas.

Instrucciones para ejecutar los programas

Para ejecutar correctamente los programas desarrollados en MATLAB se recomienda seguir el procedimiento descrito a continuación:

Primero abre MATLAB y establece como carpeta de trabajo el directorio donde se encuentran almacenados los archivos de la práctica.
Verifica que todos los archivos .m necesarios para la simulación se encuentren en la misma carpeta.
Abre el programa correspondiente a la práctica que desea ejecutar.
Revisa que los parámetros del robot, las condiciones iniciales y las ganancias del controlador sean los establecidos para la simulación.
Ejecuta el programa presionando el botón Run o utilizando la tecla F5.
Esperar a que MATLAB complete la simulación y genere automáticamente las gráficas correspondientes, incluyendo la trayectoria del robot, los errores de posición y orientación, las velocidades lineal y angular, las velocidades de las ruedas y la comparación entre los casos con y sin saturación.
En el programa de control cinemático, al finalizar la simulación se genera automáticamente el archivo control_cinematico.gif, el cual muestra la animación del movimiento del robot desde la posición inicial hasta el punto objetivo.
Si se desea analizar un comportamiento diferente del robot, es posible modificar los parámetros del sistema o del controlador y ejecutar nuevamente el programa para observar los cambios en las gráficas y en la trayectoria obtenida.
