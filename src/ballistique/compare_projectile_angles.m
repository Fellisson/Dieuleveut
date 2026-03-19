% Compare trajectories for several launch angles.
% This function simulates the same projectile launched with different
% initial angles, then compares the resulting ranges, altitudes and
% flight times on a single figure, in a text log and in an MP4 video.
%
% Generated outputs:
%   - out/images/comparaison_trajectoires_angles.png
%   - out/videos/comparaison_trajectoires_angles.mp4
%   - out/logs/log_compare_angles.txt
function compare_projectile_angles()
    base_dir = fileparts(mfilename('fullpath'));
    project_dir = fileparts(fileparts(base_dir));
    out_dir = fullfile(project_dir, 'out', 'logs');
    images_dir = fullfile(project_dir, 'out', 'images');
    videos_dir = fullfile(project_dir, 'out', 'videos');
    is_batch = is_batch_mode();

    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    if ~exist(images_dir, 'dir')
        mkdir(images_dir);
    end
    if ~exist(videos_dir, 'dir')
        mkdir(videos_dir);
    end

    log_file = fopen(fullfile(out_dir, 'log_compare_angles.txt'), 'w');

    g = 9.80665;
    rho_mat = 7850;
    r = 0.03;
    m = 4/3 * pi * r^3 * rho_mat;

    eta = 1.81e-5;
    b1 = 6 * pi * eta * r;
    cd = 0.469;
    rho_air = 1.22;
    b2 = cd * 4 * pi * r^2 * rho_air / 2;

    v0 = 300;
    angles = [20, 30, 45, 60, 75];
    colors = lines(numel(angles));

    traj = cell(1, numel(angles));
    stats = struct('tf', [], 'b', [], 'h', [], 'tu', [], 'tc', [], 'Q', []);

    fig = figure('Position', [100, 100, 900, 600], ...
        'Visible', figure_visibility(is_batch));
    hold on;
    grid on;
    box on;
    title('Comparaison des trajectoires pour differents angles de tir');
    xlabel('Position horizontale (km)');
    ylabel('Position verticale (km)');

    for k = 1:numel(angles)
        alpha0 = angles(k);
        [t, vx, vy, x, y] = compute_trajectory(v0, alpha0, b1, b2, g, m);
        [tf, b, h, tu, tc, Q] = compute_quantities_simple(t, vx, vy, x, y, v0, m);

        traj{k} = struct('t', t, 'vx', vx, 'vy', vy, 'x', x, 'y', y);
        stats(k).tf = tf;
        stats(k).b = b;
        stats(k).h = h;
        stats(k).tu = tu;
        stats(k).tc = tc;
        stats(k).Q = Q;

        plot(x / 1e3, y / 1e3, 'LineWidth', 2, 'Color', colors(k, :), ...
            'DisplayName', sprintf('\\alpha_0 = %d deg', alpha0));

        fprintf(log_file, '=========== Angle = %d degres ===========\n', alpha0);
        fprintf(log_file, 'Temps de vol         : %.4f s\n', tf);
        fprintf(log_file, 'Portee               : %.4f km\n', b / 1e3);
        fprintf(log_file, 'Altitude maximale    : %.4f km\n', h / 1e3);
        fprintf(log_file, 'Temps de montee      : %.4f s\n', tu);
        fprintf(log_file, 'Temps de descente    : %.4f s\n', tc);
        fprintf(log_file, 'Chaleur produite     : %.4f kJ\n', Q / 1e3);
        fprintf(log_file, '=========================================\n\n');

        fprintf('Angle = %d deg | tf = %.2f s | Portee = %.2f km | hmax = %.2f km\n', ...
            alpha0, tf, b / 1e3, h / 1e3);
    end

    legend('Location', 'best');
    saveas(fig, fullfile(images_dir, 'comparaison_trajectoires_angles.png'));
    close(fig);

    animate_angle_comparison(traj, angles, stats, colors, videos_dir, is_batch);

    fclose(log_file);
end

function [t, vx, vy, x, y] = compute_trajectory(v0, alpha0, b1, b2, g, m)
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

        vx(i + 1) = vx(i) * aux;
        vy(i + 1) = vy(i) * aux - g * dt;

        x(i + 1) = x(i) + vx(i) * dt;
        y(i + 1) = y(i) + vy(i) * dt;

        if y(i + 1) < 0
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
    tf = t(end);
    b = x(end);
    h = max(y);

    [~, idx_h] = max(y);
    tu = t(idx_h);
    tc = tf - tu;

    Q = 0.5 * m * (v0^2 - vx(end)^2 - vy(end)^2);
end

function animate_angle_comparison(traj, angles, stats, colors, videos_dir, is_batch)
    fig = figure('Position', [100, 100, 1100, 680], 'Color', [0.74 0.87 1.00], ...
        'Visible', figure_visibility(is_batch));
    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'Position horizontale (km)');
    ylabel(ax, 'Position verticale (km)');
    title(ax, 'Animation comparative des angles de tir');

    xmax = max(cellfun(@(s) max(s.x), traj)) / 1e3 * 1.08;
    ymax = max(cellfun(@(s) max(s.y), traj)) / 1e3 * 1.12;
    xlim(ax, [0 xmax]);
    ylim(ax, [0 ymax]);

    plot(ax, [0 xmax], [0 0], 'Color', [0.35 0.24 0.12], 'LineWidth', 2, ...
        'DisplayName', 'Sol');

    path_handles = gobjects(1, numel(angles));
    marker_handles = gobjects(1, numel(angles));
    legend_handles = gobjects(1, numel(angles) + 1);
    legend_handles(1) = plot(ax, NaN, NaN, 'Color', [0.35 0.24 0.12], 'LineWidth', 2);
    for k = 1:numel(angles)
        plot(ax, traj{k}.x / 1e3, traj{k}.y / 1e3, '--', ...
            'Color', colors(k, :) * 0.65 + 0.35, 'LineWidth', 1.1);
        path_handles(k) = plot(ax, NaN, NaN, 'Color', colors(k, :), 'LineWidth', 2.2);
        marker_handles(k) = plot(ax, NaN, NaN, 'o', 'MarkerSize', 7, ...
            'MarkerFaceColor', colors(k, :), 'MarkerEdgeColor', 'k');
        legend_handles(k + 1) = plot(ax, NaN, NaN, 'Color', colors(k, :), 'LineWidth', 2.2);
    end

    legend(ax, legend_handles, ['Sol', compose('%d deg', angles)], 'Location', 'best');

    txt_time = text(ax, 0.98 * xmax, 0.95 * ymax, '', ...
        'HorizontalAlignment', 'right', 'FontWeight', 'bold');
    txt_info = gobjects(1, numel(angles));
    for k = 1:numel(angles)
        txt_info(k) = text(ax, 0.98 * xmax, (0.88 - 0.07 * (k - 1)) * ymax, '', ...
            'HorizontalAlignment', 'right', 'Color', colors(k, :));
    end

    videoPath = fullfile(videos_dir, 'comparaison_trajectoires_angles.mp4');
    vw = VideoWriter(videoPath, 'MPEG-4');
    vw.FrameRate = 24;
    vw.Quality = 100;
    open(vw);

    min_len = min(cellfun(@(s) numel(s.x), traj));
    step = 1;
    if is_batch
        step = max(1, ceil(min_len / 280));
    end

    for i = 2:step:min_len
        current_time = 0;
        for k = 1:numel(angles)
            set(path_handles(k), 'XData', traj{k}.x(1:i) / 1e3, 'YData', traj{k}.y(1:i) / 1e3);
            set(marker_handles(k), 'XData', traj{k}.x(i) / 1e3, 'YData', traj{k}.y(i) / 1e3);
            current_time = max(current_time, traj{k}.t(i));
            speed = hypot(traj{k}.vx(i), traj{k}.vy(i));
            set(txt_info(k), 'String', sprintf('%d deg : x = %.2f km, y = %.2f km, |v| = %.1f m/s', ...
                angles(k), traj{k}.x(i) / 1e3, traj{k}.y(i) / 1e3, speed));
        end

        set(txt_time, 'String', sprintf('Temps courant : %.2f s', current_time));
        drawnow limitrate nocallbacks;
        writeVideo(vw, getframe(fig));
    end

    for k = 1:numel(angles)
        plot(ax, traj{k}.x(end) / 1e3, traj{k}.y(end) / 1e3, 'x', ...
            'Color', colors(k, :), 'LineWidth', 2, 'MarkerSize', 10);
    end
    drawnow limitrate nocallbacks;
    writeVideo(vw, getframe(fig));

    close(vw);
    close(fig);

    fprintf('Video comparative enregistree : %s\n', videoPath);
    for k = 1:numel(angles)
        fprintf('Angle %d deg -> portee = %.3f km, altitude max = %.3f km\n', ...
            angles(k), stats(k).b / 1e3, stats(k).h / 1e3);
    end
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
