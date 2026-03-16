% Compare trajectories for several launch angles.
% This function simulates the same projectile launched with different
% initial angles, then compares the resulting ranges, altitudes and
% flight times on a single figure and in a text log.
%
% Generated outputs:
%   - ../images/comparaison_trajectoires_angles.png
%   - ../out/log_compare_angles.txt
%
% Main steps:
%   1. Define physical parameters and drag coefficients
%   2. Simulate the trajectory for each launch angle
%   3. Compute summary quantities
%   4. Save a comparison plot and a log file
function compare_projectile_angles()

    % Création des dossiers de sortie
    if ~exist('../out', 'dir')
        mkdir('../out');
    end
    if ~exist('../images', 'dir')
        mkdir('../images');
    end

    % Ouverture du fichier log
    log_file = fopen('../out/log_compare_angles.txt', 'w');

    % Paramètres physiques
    g = 9.80665;                % gravité (m/s^2)
    ro = 7850;                  % densité acier (kg/m^3)
    r = 0.03;                   % rayon projectile (m)
    m = 4/3 * pi * r^3 * ro;    % masse (kg)

    eta = 1.81e-5;              % viscosité de l'air
    b1 = 6 * pi * eta * r;      % coefficient linéaire
    c = 0.469;                  % coefficient de forme
    ro0 = 1.22;                 % densité de l'air
    b2 = c * 4 * pi * r^2 * ro0 / 2;   % coefficient quadratique

    % Condition initiale commune
    v0 = 300;                   % vitesse initiale (m/s)

    % Angles à comparer
    angles = [20, 30, 45];

    % Figure de comparaison des trajectoires
    figure('Position', [100, 100, 900, 600]);
    hold on;
    grid on;
    title('Comparaison des trajectoires pour différents angles de tir');
    xlabel('Position horizontale (km)');
    ylabel('Position verticale (km)');

    for k = 1:length(angles)
        alpha0 = angles(k);

        % Calcul de la trajectoire
        [t, vx, vy, x, y] = compute_trajectory(v0, alpha0, b1, b2, g, m);

        % Calcul des grandeurs physiques
        [tf, b, h, tu, tc, Q] = compute_quantities_simple(t, vx, vy, x, y, v0, m);

        % Tracé de la trajectoire
        plot(x/1e3, y/1e3, 'LineWidth', 2, 'DisplayName', ...
            ['\alpha_0 = ', num2str(alpha0), '°']);

        % Écriture dans le fichier log
        fprintf(log_file, '=========== Angle = %d degres ===========\n', alpha0);
        fprintf(log_file, 'Temps de vol         : %.4f s\n', tf);
        fprintf(log_file, 'Portee               : %.4f km\n', b/1e3);
        fprintf(log_file, 'Altitude maximale    : %.4f km\n', h/1e3);
        fprintf(log_file, 'Temps de montee      : %.4f s\n', tu);
        fprintf(log_file, 'Temps de descente    : %.4f s\n', tc);
        fprintf(log_file, 'Chaleur produite     : %.4f kJ\n', Q/1e3);
        fprintf(log_file, '=========================================\n\n');

        % Affichage console
        fprintf('Angle = %d° | tf = %.2f s | Portee = %.2f km | hmax = %.2f km\n', ...
            alpha0, tf, b/1e3, h/1e3);
    end

    legend('Location', 'best');
    saveas(gcf, '../images/comparaison_trajectoires_angles.png');

    fclose(log_file);
end


function [t, vx, vy, x, y] = compute_trajectory(v0, alpha0, b1, b2, g, m)
    % Numerically integrates the projectile motion with linear and
    % quadratic drag using an explicit Euler scheme.

    t0 = 0;
    tf = 2 * v0 / g * sind(alpha0);
    N = 1000;
    t = linspace(t0, tf, N);
    dt = t(2) - t(1);

    vx = zeros(1, N);
    vy = zeros(1, N);
    x = zeros(1, N);
    y = zeros(1, N);

    vx(1) = v0 * cosd(alpha0);
    vy(1) = v0 * sind(alpha0);

    for i = 1:N-1
        v = sqrt(vx(i)^2 + vy(i)^2);
        aux = 1 - dt * (b1 + b2 * v) / m;

        vx(i+1) = vx(i) * aux;
        vy(i+1) = vy(i) * aux - g * dt;

        x(i+1) = x(i) + vx(i) * dt;
        y(i+1) = y(i) + vy(i) * dt;

        if y(i+1) < 0
            break;
        end
    end

    t = t(1:i);
    vx = vx(1:i);
    vy = vy(1:i);
    x = x(1:i);
    y = y(1:i);
end


function [tf, b, h, tu, tc, Q] = compute_quantities_simple(t, vx, vy, x, y, v0, m)
    % Extracts flight metrics from the simulated trajectory:
    % final time, range, peak altitude, ascent/descent times and the
    % estimated kinetic energy lost to heat.

    tf = t(end);
    b = x(end);
    h = max(y);

    [~, idx_h] = max(y);
    tu = t(idx_h);
    tc = tf - tu;

    Q = 0.5 * m * (v0^2 - vx(end)^2 - vy(end)^2);
end
