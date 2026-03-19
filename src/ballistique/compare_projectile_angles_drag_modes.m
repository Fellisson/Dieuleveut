% Compare several launch angles with and without aerodynamic drag.
% Generated outputs:
%   - out/images/comparaison_angles_avec_sans_frottement.png
%   - out/videos/comparaison_angles_avec_sans_frottement.mp4
%   - out/logs/log_compare_angles_drag_modes.txt
function compare_projectile_angles_drag_modes()
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

    log_file = fopen(fullfile(out_dir, 'log_compare_angles_drag_modes.txt'), 'w');

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

    drag_traj = cell(1, numel(angles));
    nodrag_traj = cell(1, numel(angles));
    drag_stats = struct('tf', [], 'b', [], 'h', []);
    nodrag_stats = struct('tf', [], 'b', [], 'h', []);

    fig = figure('Position', [100, 100, 1200, 700], ...
        'Visible', figure_visibility(is_batch));
    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    ax1 = nexttile;
    hold(ax1, 'on');
    grid(ax1, 'on');
    box(ax1, 'on');
    title(ax1, 'Avec frottement');
    xlabel(ax1, 'Position horizontale (km)');
    ylabel(ax1, 'Position verticale (km)');

    ax2 = nexttile;
    hold(ax2, 'on');
    grid(ax2, 'on');
    box(ax2, 'on');
    title(ax2, 'Sans frottement');
    xlabel(ax2, 'Position horizontale (km)');
    ylabel(ax2, 'Position verticale (km)');

    for k = 1:numel(angles)
        alpha0 = angles(k);
        [tD, vxD, vyD, xD, yD] = compute_trajectory(v0, alpha0, b1, b2, g, m, true);
        [tN, vxN, vyN, xN, yN] = compute_trajectory(v0, alpha0, b1, b2, g, m, false);

        drag_traj{k} = struct('t', tD, 'vx', vxD, 'vy', vyD, 'x', xD, 'y', yD);
        nodrag_traj{k} = struct('t', tN, 'vx', vxN, 'vy', vyN, 'x', xN, 'y', yN);

        [drag_stats(k).tf, drag_stats(k).b, drag_stats(k).h] = basic_stats(tD, xD, yD);
        [nodrag_stats(k).tf, nodrag_stats(k).b, nodrag_stats(k).h] = basic_stats(tN, xN, yN);

        plot(ax1, xD / 1e3, yD / 1e3, 'LineWidth', 2, 'Color', colors(k, :), ...
            'DisplayName', sprintf('%d deg', alpha0));
        plot(ax2, xN / 1e3, yN / 1e3, 'LineWidth', 2, 'Color', colors(k, :), ...
            'DisplayName', sprintf('%d deg', alpha0));

        fprintf(log_file, '=========== Angle = %d degres ===========\n', alpha0);
        fprintf(log_file, 'AVEC FROTTEMENT   : tf = %.4f s | portee = %.4f km | hmax = %.4f km\n', ...
            drag_stats(k).tf, drag_stats(k).b / 1e3, drag_stats(k).h / 1e3);
        fprintf(log_file, 'SANS FROTTEMENT   : tf = %.4f s | portee = %.4f km | hmax = %.4f km\n', ...
            nodrag_stats(k).tf, nodrag_stats(k).b / 1e3, nodrag_stats(k).h / 1e3);
        fprintf(log_file, '=========================================\n\n');
    end

    legend(ax1, 'Location', 'best');
    legend(ax2, 'Location', 'best');
    saveas(fig, fullfile(images_dir, 'comparaison_angles_avec_sans_frottement.png'));
    close(fig);

    animate_drag_modes(drag_traj, nodrag_traj, angles, colors, videos_dir, is_batch);
    fclose(log_file);
end

function [t, vx, vy, x, y] = compute_trajectory(v0, alpha0, b1, b2, g, m, use_drag)
    t0 = 0;
    tf = 2 * v0 / g * sind(alpha0);
    N = 1400;
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
        drag_scale = 0;
        if use_drag
            drag_scale = b1 + b2 * v;
        end
        aux = 1 - dt * drag_scale / m;

        vx(i + 1) = vx(i) * aux;
        vy(i + 1) = vy(i) * aux - g * dt;

        x(i + 1) = x(i) + vx(i) * dt;
        y(i + 1) = y(i) + vy(i) * dt;

        if y(i + 1) < 0
            y(i + 1) = 0;
            break;
        end
    end

    t = t(1:i+1);
    vx = vx(1:i+1);
    vy = vy(1:i+1);
    x = x(1:i+1);
    y = y(1:i+1);
end

function [tf, b, h] = basic_stats(t, x, y)
    tf = t(end);
    b = x(end);
    h = max(y);
end

function animate_drag_modes(drag_traj, nodrag_traj, angles, colors, videos_dir, is_batch)
    fig = figure('Position', [100, 100, 1200, 700], 'Color', [0.77 0.88 1.00], ...
        'Visible', figure_visibility(is_batch));
    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    ax1 = nexttile;
    hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
    title(ax1, 'Avec frottement');
    xlabel(ax1, 'Position horizontale (km)');
    ylabel(ax1, 'Position verticale (km)');

    ax2 = nexttile;
    hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
    title(ax2, 'Sans frottement');
    xlabel(ax2, 'Position horizontale (km)');
    ylabel(ax2, 'Position verticale (km)');

    xmax = max([cellfun(@(s) max(s.x), drag_traj), cellfun(@(s) max(s.x), nodrag_traj)]) / 1e3 * 1.06;
    ymax = max([cellfun(@(s) max(s.y), drag_traj), cellfun(@(s) max(s.y), nodrag_traj)]) / 1e3 * 1.10;
    xlim(ax1, [0 xmax]); ylim(ax1, [0 ymax]);
    xlim(ax2, [0 xmax]); ylim(ax2, [0 ymax]);

    hDrag = gobjects(1, numel(angles));
    hNoDrag = gobjects(1, numel(angles));
    mDrag = gobjects(1, numel(angles));
    mNoDrag = gobjects(1, numel(angles));
    for k = 1:numel(angles)
        plot(ax1, drag_traj{k}.x / 1e3, drag_traj{k}.y / 1e3, '--', 'Color', colors(k, :) * 0.65 + 0.35);
        plot(ax2, nodrag_traj{k}.x / 1e3, nodrag_traj{k}.y / 1e3, '--', 'Color', colors(k, :) * 0.65 + 0.35);
        hDrag(k) = plot(ax1, NaN, NaN, 'Color', colors(k, :), 'LineWidth', 2);
        hNoDrag(k) = plot(ax2, NaN, NaN, 'Color', colors(k, :), 'LineWidth', 2);
        mDrag(k) = plot(ax1, NaN, NaN, 'o', 'MarkerFaceColor', colors(k, :), 'MarkerEdgeColor', 'k');
        mNoDrag(k) = plot(ax2, NaN, NaN, 'o', 'MarkerFaceColor', colors(k, :), 'MarkerEdgeColor', 'k');
    end
    legend(ax1, compose('%d deg', angles), 'Location', 'best');
    legend(ax2, compose('%d deg', angles), 'Location', 'best');

    videoPath = fullfile(videos_dir, 'comparaison_angles_avec_sans_frottement.mp4');
    vw = VideoWriter(videoPath, 'MPEG-4');
    vw.FrameRate = 24;
    vw.Quality = 100;
    open(vw);

    min_len = min([cellfun(@(s) numel(s.x), drag_traj), cellfun(@(s) numel(s.x), nodrag_traj)]);
    step = 1;
    if is_batch
        step = max(1, ceil(min_len / 260));
    end

    for i = 2:step:min_len
        for k = 1:numel(angles)
            set(hDrag(k), 'XData', drag_traj{k}.x(1:i) / 1e3, 'YData', drag_traj{k}.y(1:i) / 1e3);
            set(mDrag(k), 'XData', drag_traj{k}.x(i) / 1e3, 'YData', drag_traj{k}.y(i) / 1e3);
            set(hNoDrag(k), 'XData', nodrag_traj{k}.x(1:i) / 1e3, 'YData', nodrag_traj{k}.y(1:i) / 1e3);
            set(mNoDrag(k), 'XData', nodrag_traj{k}.x(i) / 1e3, 'YData', nodrag_traj{k}.y(i) / 1e3);
        end
        drawnow limitrate nocallbacks;
        writeVideo(vw, getframe(fig));
    end

    close(vw);
    close(fig);
    fprintf('Video avec/sans frottement enregistree : %s\n', videoPath);
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
