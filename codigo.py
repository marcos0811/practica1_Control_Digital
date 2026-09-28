% =========================================================================
% PRÁCTICA 1: MODELADO E IDENTIFICACIÓN DE UN MOTOR DC
% =========================================================================
clc; clear; close all;

% 1. Crear el modelo del arnés si no existe en memoria
bdclose('harness_dcmotor'); % Cierra si estaba abierto antes
new_system('harness_dcmotor');
open_system('harness_dcmotor');

% 2. Agregar los bloques necesarios
add_block('DCMotor/DC motor and gearbox model', 'harness_dcmotor/DC motor and gearbox model');
add_block('simulink/Sources/Step', 'harness_dcmotor/Step');
add_block('simulink/Sinks/Out1', 'harness_dcmotor/angle_out');

% 3. Configurar la entrada de voltaje (Escalón de 1 V en t = 0)
set_param('harness_dcmotor/Step', 'Time', '0', 'After', '1', 'Before', '0');

% 4. Realizar las conexiones
add_line('harness_dcmotor', 'Step/1', 'DC motor and gearbox model/1');
add_line('harness_dcmotor', 'DC motor and gearbox model/1', 'angle_out/1');

% 5. Ejecutar la simulación durante 10 segundos
simOut = sim('harness_dcmotor', 'StopTime', '10');

disp('¡Simulación completada con éxito!'); 
%
% =========================================================================
% 8. PROCESAMIENTO DE VELOCIDAD Y MÉTODO DEL 63.2 %
% =========================================================================

% Extraer datos del resultado de simulación de forma segura
t = simOut.tout;

% Intentar obtener la salida angle_out según la estructura guardada
if isprop(simOut, 'logsout') && ~isempty(simOut.logsout)
    angle_deg = simOut.logsout.get('angle_out').Values.Data;
elseif isprop(simOut, 'yout') && ~isempty(simOut.yout)
    angle_deg = simOut.yout{1}.Values.Data;
else
    angle_deg = simOut.getElement('angle_out').Values.Data;
end

% Convertir ángulo a radianes y derivar para obtener velocidad (rad/s)
angle_rad = angle_deg;
omega = gradient(angle_rad, t); % Diferenciación numérica

% A) Valor final y Ganancia K (para entrada de 1 V)
y_ss = omega(end); 
K_ident = y_ss / 1.0; 

% B) Punto del 63.2%
y_63 = 0.632 * y_ss;
idx_tau = find(omega >= y_63, 1, 'first');
tau_ident = t(idx_tau);

% Mostrar resultados en la consola de MATLAB
fprintf('\n--- RESULTADOS DE IDENTIFICACIÓN ---');
fprintf('\nGanancia K identificada: %.4f (rad/s)/V (o %.2f deg/s/V)', K_ident, K_ident*(180/pi));
fprintf('\nConstante de tiempo tau: %.4f s\n', tau_ident);

% C) Respuesta teórica de primer orden: w(t) = K * (1 - exp(-t/tau))
omega_model = K_ident * (1 - exp(-t / tau_ident));

% =========================================================================
% 9. GRAFICACIÓN COMPLETA PARA EL INFORME
% =========================================================================
figure('Color', 'w', 'Name', 'Identificación de la Velocidad Angular');

% Subplot 1: Posición Angular
subplot(2,1,1);
plot(t, angle_deg, 'b-', 'LineWidth', 1.5);
grid on;
ylabel('\theta [grados]');
title('Posición Angular (Integrador Puro)');

% Subplot 2: Velocidad Angular + Modelo Identificado + Líneas de 63.2%

subplot(2,1,2);

plot(t, omega, 'r-', 'LineWidth', 1.5, ...
    'DisplayName', 'Velocidad simulada');

hold on;

plot(t, omega_model, 'k--', 'LineWidth', 1.5, ...
    'DisplayName', 'Modelo de primer orden');

% Valor final y nivel del 63.2 %
yline(y_ss, 'k:', ...
    sprintf(' y_{ss} = %.3f rad/s', y_ss), ...
    'LineWidth', 1.2, ...
    'DisplayName', 'Valor final');

yline(y_63, 'b--', ...
    sprintf(' 63.2%% = %.3f rad/s', y_63), ...
    'LineWidth', 1.2, ...
    'DisplayName', '63.2 %');

% Instante de la constante de tiempo
xline(tau_ident, 'm--', ...
    sprintf(' \\tau = %.4f s', tau_ident), ...
    'LineWidth', 1.2, ...
    'DisplayName', '\tau');

grid on;

xlabel('Tiempo [s]');
ylabel('\omega [rad/s]');
title('Identificacion de la velocidad angular mediante el metodo del 63.2%');

legend('Location', 'southeast');
