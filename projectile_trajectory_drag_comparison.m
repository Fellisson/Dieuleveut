% Rocket/Projectile simulation:
% comparison with and without aerodynamic drag
% Author: Felic Nimy - modified version

function projectile_trajectory_drag_comparison(~, ~)

    % Create output folders if needed
    if ~exist('out', 'dir')
        mkdir('out');
    end
    if ~exist('images', 'dir')
        mkdir('images');
    end

    % Log file
    log_file = fopen('../out/log_drag_comparison.txt', 'a');

    %% Physical constants
    g = 9.80665;          % gravity (m/s^2)
    rho_mat = 7850;       % material density (kg/m^3)
    r = 0.03;             % rocket/projectile radius (m)

    % Air properties
    eta = 1.81e-5;        % air viscosity (Pa.s)
    Cd = 0.469;           % drag coefficient
    rho_air = 1.22;       % air density (kg/m^3)

    % Drag coefficients
    b1 = 6*pi*eta*r;                      % linear drag coefficient
    b2 = Cd * 4*pi*r^2 * rho_air / 2;     % quadratic drag coefficient

    %% Mass parameters
    m_struct = 0.6 * (4/3)*pi*r^3*rho_mat;   % dry mass
    m_fuel0  = 0.4 * (4/3)*pi*r^3*rho_mat;   % initial fuel mass
    m0 = m_struct + m_fuel0;                 % initial total mass

    %% Launch conditions
    v0 = 300;          % initial speed (m/s)
    alpha0 = 20;       % launch angle (deg)

    %% Propulsion
    T0 = 1500;         % thrust (N)
    mdot = 0.8;        % fuel mass flow rate (kg/s)
    tburn = m_fuel0 / mdot;

    %% Case 1: with drag
    [tD, vxD, vyD, xD, yD, mD, TD] = compute_trajectory( ...
        v0, alpha0, g, m0, m_struct, T0, mdot, tburn, b1, b2, true);

    %% Case 2: without drag
    [tN, vxN, vyN, xN, yN, mN, TN] = compute_trajectory( ...
        v0, alpha0, g, m0, m_struct, T0, mdot, tburn, b1, b2, false);

    %% Compute quantities
    [tfD, bD, hD, tuD, tcD] = compute_quantities(tD, vxD, vyD, xD, yD);
    [tfN, bN, hN, tuN, tcN] = compute_quantities(tN, vxN, vyN, xN, yN);

    %% Plots
    plot_comparison_results(tD, vxD, vyD, xD, yD, mD, TD, ...
                            tN, vxN, vyN, xN, yN, mN, TN);

    %% Animation
    plot_trajectory_animation_comparison(xD, yD, xN, yN);

    %% Display
    disp('================ WITH DRAG ================');
    disp(['Flight Time       : ', num2str(tfD), ' s']);
    disp(['Range             : ', num2str(bD/1e3), ' km']);
    disp(['Maximum Altitude  : ', num2str(hD/1e3), ' km']);
    disp(['Ascent Time       : ', num2str(tuD), ' s']);
    disp(['Descent Time      : ', num2str(tcD), ' s']);

    disp('============== WITHOUT DRAG ==============');
    disp(['Flight Time       : ', num2str(tfN), ' s']);
    disp(['Range             : ', num2str(bN/1e3), ' km']);
    disp(['Maximum Altitude  : ', num2str(hN/1e3), ' km']);
    disp(['Ascent Time       : ', num2str(tuN), ' s']);
    disp(['Descent Time      : ', num2str(tcN), ' s']);
    disp('==========================================');

    %% Save to log file
    fprintf(log_file, '=========== DRAG COMPARISON ===========\n');

    fprintf(log_file, 'WITH DRAG\n');
    fprintf(log_file, 'Flight Time      : %.4f s\n', tfD);
    fprintf(log_file, 'Range            : %.4f km\n', bD/1e3);
    fprintf(log_file, 'Maximum Altitude : %.4f km\n', hD/1e3);
    fprintf(log_file, 'Ascent Time      : %.4f s\n', tuD);
    fprintf(log_file, 'Descent Time     : %.4f s\n', tcD);

    fprintf(log_file, '\nWITHOUT DRAG\n');
    fprintf(log_file, 'Flight Time      : %.4f s\n', tfN);
    fprintf(log_file, 'Range            : %.4f km\n', bN/1e3);
    fprintf(log_file, 'Maximum Altitude : %.4f km\n', hN/1e3);
    fprintf(log_file, 'Ascent Time      : %.4f s\n', tuN);
    fprintf(log_file, 'Descent Time     : %.4f s\n', tcN);
    fprintf(log_file, '=======================================\n\n');

    fclose(log_file);
end


function [t, vx, vy, x, y, m_hist, T_hist] = compute_trajectory( ...
    v0, alpha0, g, m0, m_struct, T0, mdot, tburn, b1, b2, use_drag)

    t0 = 0;
    tf_est = 80;
    N = 5000;

    t = linspace(t0, tf_est, N);
    dt = t(2) - t(1);

    vx = zeros(1, N);
    vy = zeros(1, N);
    x = zeros(1, N);
    y = zeros(1, N);
    m_hist = zeros(1, N);
    T_hist = zeros(1, N);

    vx(1) = v0*cosd(alpha0);
    vy(1) = v0*sind(alpha0);
    m_hist(1) = m0;

    ex = cosd(alpha0);
    ey = sind(alpha0);

    for i = 1:N-1
        ti = t(i);

        % Thrust and fuel consumption
        if ti <= tburn && m_hist(i) > m_struct
            T = T0;
            m_next = max(m_struct, m_hist(i) - mdot*dt);
        else
            T = 0;
            m_next = m_hist(i);
        end

        v = sqrt(vx(i)^2 + vy(i)^2);

        if use_drag
            Fdx = (b1 + b2*v) * vx(i);
            Fdy = (b1 + b2*v) * vy(i);
        else
            Fdx = 0;
            Fdy = 0;
        end

        ax = (T*ex - Fdx) / m_hist(i);
        ay = (T*ey - Fdy) / m_hist(i) - g;

        vx(i+1) = vx(i) + ax*dt;
        vy(i+1) = vy(i) + ay*dt;

        x(i+1) = x(i) + vx(i)*dt;
        y(i+1) = y(i) + vy(i)*dt;

        m_hist(i+1) = m_next;
        T_hist(i) = T;

        if y(i+1) < 0
            y(i+1) = 0;
            T_hist(i+1) = 0;
            break;
        end
    end

    t = t(1:i+1);
    vx = vx(1:i+1);
    vy = vy(1:i+1);
    x = x(1:i+1);
    y = y(1:i+1);
    m_hist = m_hist(1:i+1);
    T_hist = T_hist(1:i+1);
end


function [tf, b, h, tu, tc] = compute_quantities(t, vx, vy, x, y)
    tf = t(end);
    b = x(end);
    h = max(y);
    [~, idx] = max(y);
    tu = t(idx);
    tc = tf - tu;
end


function plot_comparison_results(tD, vxD, vyD, xD, yD, mD, TD, ...
                                 tN, vxN, vyN, xN, yN, ~, TN)

    figure('Position', [100, 100, 1000, 900]);

    % Velocity
    subplot(4,1,1);
    plot(tD, vxD, 'r', 'LineWidth', 1.3); hold on;
    plot(tD, vyD, 'b', 'LineWidth', 1.3);
    plot(tN, vxN, '--r', 'LineWidth', 1.3);
    plot(tN, vyN, '--b', 'LineWidth', 1.3);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Components: with and without drag');
    legend('v_x with drag', 'v_y with drag', ...
           'v_x without drag', 'v_y without drag', ...
           'Location', 'best');
    grid on;

    % Coordinates
    subplot(4,1,2);
    plot(tD, xD/1e3, 'r', 'LineWidth', 1.3); hold on;
    plot(tD, yD/1e3, 'b', 'LineWidth', 1.3);
    plot(tN, xN/1e3, '--r', 'LineWidth', 1.3);
    plot(tN, yN/1e3, '--b', 'LineWidth', 1.3);
    xlabel('Time (s)');
    ylabel('Coordinates (km)');
    title('Coordinates vs Time');
    legend('x with drag', 'y with drag', ...
           'x without drag', 'y without drag', ...
           'Location', 'best');
    grid on;

    % Trajectory
    subplot(4,1,3);
    plot(xD/1e3, yD/1e3, 'k', 'LineWidth', 2); hold on;
    plot(xN/1e3, yN/1e3, '--m', 'LineWidth', 2);
    xlabel('Horizontal Position (km)');
    ylabel('Vertical Position (km)');
    title('Trajectory comparison');
    legend('With drag', 'Without drag', 'Location', 'best');
    grid on;

    % Mass and thrust
    subplot(4,1,4);
    yyaxis left
    plot(tD, mD, 'm', 'LineWidth', 1.5);
    ylabel('Mass (kg)');

    yyaxis right
    plot(tD, TD, 'g', 'LineWidth', 1.5);
    ylabel('Thrust (N)');

    xlabel('Time (s)');
    title('Mass variation and thrust');
    legend('Mass', 'Thrust', 'Location', 'best');
    grid on;

    cd('/images');
    saveas(gcf, 'drag_comparison_results.png');
    cd('../');
end


function plot_trajectory_animation_comparison(xD, yD, xN, yN)

    figure('Position', [120, 120, 900, 600], 'Color', [0.75 0.87 1]);
    hold on;
    grid on;
    xlabel('Horizontal Position (km)');
    ylabel('Vertical Position (km)');
    title('Trajectory Animation: with and without drag');

    plot(xD/1e3, yD/1e3, 'k', 'LineWidth', 2);
    plot(xN/1e3, yN/1e3, '--m', 'LineWidth', 2);

    hD = plot(xD(1)/1e3, yD(1)/1e3, 'o', ...
        'MarkerSize', 7, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
    hN = plot(xN(1)/1e3, yN(1)/1e3, 's', ...
        'MarkerSize', 7, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k');

    legend('With drag trajectory', 'Without drag trajectory', ...
           'With drag projectile', 'Without drag projectile', ...
           'Location', 'best');

    Nmax = max(length(xD), length(xN));

    for i = 2:Nmax
        if i <= length(xD)
            set(hD, 'XData', xD(i)/1e3, 'YData', yD(i)/1e3);
        end
        if i <= length(xN)
            set(hN, 'XData', xN(i)/1e3, 'YData', yN(i)/1e3);
        end

        drawnow;
        pause(1e-3);
    end

    cd('../images');
    saveas(gcf, 'drag_comparison_animation.png');
    cd('../');
end