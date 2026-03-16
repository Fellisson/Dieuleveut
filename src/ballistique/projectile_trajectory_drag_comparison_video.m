% Video generation for the drag/no-drag projectile comparison.
% This function runs two projectile simulations and records an annotated
% side-by-side style comparison in MP4 format.
%
% Output:
%   - out/videos/drag_comparison_animation.mp4
function projectile_trajectory_drag_comparison_video(~, ~)
    clc; close all;
    base_dir = fileparts(mfilename('fullpath'));
    project_dir = fileparts(fileparts(base_dir));
    out_dir = fullfile(project_dir, 'out', 'logs');
    images_dir = fullfile(project_dir, 'out', 'images');
    videos_dir = fullfile(project_dir, 'out', 'videos');

    % Create output folders if needed
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    if ~exist(images_dir, 'dir')
        mkdir(images_dir);
    end
    if ~exist(videos_dir, 'dir')
        mkdir(videos_dir);
    end
    is_batch = is_batch_mode();

    %% Physical constants
    g = 9.80665;          % gravity (m/s^2)
    rho_mat = 7850;       % material density (kg/m^3)
    r = 0.03;             % projectile radius (m)

    % Air properties
    eta = 1.81e-5;        % air viscosity (Pa.s)
    Cd = 0.469;           % drag coefficient
    rho_air = 1.22;       % air density (kg/m^3)

    % Drag coefficients
    b1 = 6*pi*eta*r;
    b2 = Cd * 4*pi*r^2 * rho_air / 2;

    %% Mass parameters
    m_struct = 0.6 * (4/3)*pi*r^3*rho_mat;   % dry mass
    m_fuel0  = 0.4 * (4/3)*pi*r^3*rho_mat;   % fuel mass
    m0 = m_struct + m_fuel0;                 % initial mass

    %% Launch conditions
    v0 = 300;        % initial velocity (m/s)
    alpha0 = 20;     % launch angle (deg)

    %% Propulsion
    T0 = 1500;       % thrust (N)
    mdot = 0.8;      % fuel mass flow rate (kg/s)
    tburn = m_fuel0 / mdot;

    %% Simulations
    [tD, vxD, vyD, xD, yD, ~, ~] = compute_trajectory( ...
        v0, alpha0, g, m0, m_struct, T0, mdot, tburn, b1, b2, true);

    [tN, vxN, vyN, xN, yN, ~, ~] = compute_trajectory( ...
        v0, alpha0, g, m0, m_struct, T0, mdot, tburn, b1, b2, false);

    %% Animation + MP4 recording
    animate_comparison_live_video(tD, vxD, vyD, xD, yD, ...
                                  tN, vxN, vyN, xN, yN, videos_dir, is_batch);
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


function animate_comparison_live_video(tD, vxD, vyD, xD, yD, ...
                                       tN, vxN, vyN, xN, yN, videos_dir, is_batch)

    fig = figure('Position', [100, 100, 1100, 680], 'Color', [0.75 0.87 1], ...
        'Visible', figure_visibility(is_batch));
    hold on; grid on; box on;

    xlabel('Horizontal Position (km)', 'FontSize', 11);
    ylabel('Vertical Position (km)', 'FontSize', 11);
    title('Animation comparative : avec et sans résistance de l''air', 'FontSize', 13);

    xmax = max([xD(:); xN(:)]) / 1e3 * 1.05;
    ymax = max([yD(:); yN(:)]) / 1e3 * 1.10;
    xlim([0 xmax]);
    ylim([0 ymax]);

    % Sol
    plot([0 xmax], [0 0], 'Color', [0.3 0.2 0.1], 'LineWidth', 2);

    % Full trajectories
    plot(xD/1e3, yD/1e3, 'k', 'LineWidth', 2);
    plot(xN/1e3, yN/1e3, '--m', 'LineWidth', 2);

    % Moving markers
    hD = plot(xD(1)/1e3, yD(1)/1e3, 'o', ...
        'MarkerSize', 8, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
    hN = plot(xN(1)/1e3, yN(1)/1e3, 's', ...
        'MarkerSize', 8, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k');

    % Trails
    trailD = plot(NaN, NaN, 'r', 'LineWidth', 2);
    trailN = plot(NaN, NaN, 'b', 'LineWidth', 2);

    legend('Sol', 'Trajectoire avec traînée', 'Trajectoire sans traînée', ...
           'Projectile avec traînée', 'Projectile sans traînée', ...
           'Location', 'best');

    % Live text
    txt1 = text(0.98*xmax, 0.95*ymax, '', ...
        'HorizontalAlignment', 'right', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 10);

    txt2 = text(0.98*xmax, 0.88*ymax, '', ...
        'HorizontalAlignment', 'right', 'Color', 'r', 'FontSize', 10);

    txt3 = text(0.98*xmax, 0.81*ymax, '', ...
        'HorizontalAlignment', 'right', 'Color', 'b', 'FontSize', 10);

    txt4 = text(0.98*xmax, 0.72*ymax, '', ...
        'HorizontalAlignment', 'right', 'Color', [0 0.45 0], 'FontSize', 10, 'FontWeight', 'bold');

    % Video writer
    videoPath = fullfile(videos_dir, 'drag_comparison_animation.mp4');
    vw = VideoWriter(videoPath, 'MPEG-4');
    vw.FrameRate = 30;
    vw.Quality = 100;
    open(vw);

    Nmax = max(length(xD), length(xN));

    % To reduce video size and keep animation fluid
    step = 3;
    if is_batch
        step = max(step, ceil(Nmax / 300));
    end

    for i = 2:step:Nmax

        % With drag
        if i <= length(xD)
            set(hD, 'XData', xD(i)/1e3, 'YData', yD(i)/1e3);
            set(trailD, 'XData', xD(1:i)/1e3, 'YData', yD(1:i)/1e3);
            vD = sqrt(vxD(i)^2 + vyD(i)^2);
            tcurD = tD(i);
            xcurD = xD(i)/1e3;
            ycurD = yD(i)/1e3;
        else
            vD = sqrt(vxD(end)^2 + vyD(end)^2);
            tcurD = tD(end);
            xcurD = xD(end)/1e3;
            ycurD = yD(end)/1e3;
        end

        % Without drag
        if i <= length(xN)
            set(hN, 'XData', xN(i)/1e3, 'YData', yN(i)/1e3);
            set(trailN, 'XData', xN(1:i)/1e3, 'YData', yN(1:i)/1e3);
            vN = sqrt(vxN(i)^2 + vyN(i)^2);
            tcurN = tN(i);
            xcurN = xN(i)/1e3;
            ycurN = yN(i)/1e3;
        else
            vN = sqrt(vxN(end)^2 + vyN(end)^2);
            tcurN = tN(end);
            xcurN = xN(end)/1e3;
            ycurN = yN(end)/1e3;
        end

        tshow = min(tcurD, tcurN);

        % Automatic explanatory message
        if xcurN > xcurD && ycurN >= ycurD
            msg = 'Sans résistance : portée et altitude supérieures';
        elseif xcurN > xcurD
            msg = 'Sans résistance : déplacement horizontal plus grand';
        elseif ycurN > ycurD
            msg = 'Sans résistance : altitude plus élevée';
        else
            msg = 'La traînée freine fortement le projectile';
        end

        set(txt1, 'String', sprintf('Temps courant : %.2f s', tshow));
        set(txt2, 'String', sprintf(['Avec traînée  : x = %.2f km, y = %.2f km, ', ...
                                     '|v| = %.2f m/s'], xcurD, ycurD, vD));
        set(txt3, 'String', sprintf(['Sans traînée : x = %.2f km, y = %.2f km, ', ...
                                     '|v| = %.2f m/s'], xcurN, ycurN, vN));
        set(txt4, 'String', msg);

        drawnow limitrate nocallbacks;

        frame = getframe(fig);
        writeVideo(vw, frame);
    end

    % Final impact markers
    plot(xD(end)/1e3, yD(end)/1e3, 'x', 'Color', 'r', 'LineWidth', 2, 'MarkerSize', 10);
    plot(xN(end)/1e3, yN(end)/1e3, 'x', 'Color', 'b', 'LineWidth', 2, 'MarkerSize', 10);

    drawnow limitrate nocallbacks;
    frame = getframe(fig);
    writeVideo(vw, frame);

    close(vw);
    close(fig);

    fprintf('\nVidéo enregistrée avec succès : %s\n', videoPath);
end

function mode = figure_visibility(is_batch)
    if is_batch
        mode = 'off';
    else
        mode = 'on';
    end
end

function tf = is_batch_mode()
    tf = ~usejava('desktop');
end
