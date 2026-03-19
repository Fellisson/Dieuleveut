% 3D visualization driven by a simple open-loop trajectory.
% This script generates a free-flight motion without PID control and
% renders it in a 3D scene.
%
% Current assumptions:
%   - the simulated motion remains planar in x-z;
%   - the lateral coordinate y is set to zero;
%   - roll and yaw are set to zero;
%   - the simulated angle theta is interpreted as pitch.
%
% Output:
%   - out/videos/animation_3D_visualisation_sans_PID.mp4

clc; clear; close all;

base_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(fileparts(base_dir));
videos_dir = fullfile(project_dir, 'out', 'videos');
is_batch = ~usejava('desktop');

if ~exist(videos_dir, 'dir')
    mkdir(videos_dir);
end

data = simulate_open_loop_data();
t = data.t;
x = data.x;
y = zeros(size(x));
z = data.z;

phi = zeros(size(t));
theta = data.theta;
psi = zeros(size(t));

N = numel(t);

fig = figure('Position', [100 80 1200 800], 'Color', 'w', ...
    'Visible', figure_visibility(is_batch));
ax = axes(fig);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
view(ax, 35, 22);

xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
title(ax, 'Animation 3D sans commande PID');

xlim([min(x) - 2, max(x) + 2]);
ylim([-3, 3]);
zlim([0, max(z) + 3]);

[Xg, Yg] = meshgrid(linspace(min(x) - 2, max(x) + 2, 20), linspace(-3, 3, 20));
Zg = zeros(size(Xg));
surf(ax, Xg, Yg, Zg, 'FaceColor', [0.92 0.92 0.92], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.8);

plot3(ax, x, y, z, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);

hTrace = plot3(ax, NaN, NaN, NaN, 'r', 'LineWidth', 2);
hCenter = plot3(ax, x(1), y(1), z(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
hBody1 = plot3(ax, NaN, NaN, NaN, 'b', 'LineWidth', 3);
hBody2 = plot3(ax, NaN, NaN, NaN, 'k', 'LineWidth', 2);

hXb = quiver3(ax, x(1), y(1), z(1), 0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.8);
hYb = quiver3(ax, x(1), y(1), z(1), 0, 0, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.8);
hZb = quiver3(ax, x(1), y(1), z(1), 0, 0, 0, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.8);

txt = text(ax, max(x), 2.8, max(z) + 2, '', ...
    'HorizontalAlignment', 'right', 'FontWeight', 'bold');

videoPath = fullfile(videos_dir, 'animation_3D_visualisation_sans_PID.mp4');
vw = VideoWriter(videoPath, 'MPEG-4');
vw.FrameRate = 25;
vw.Quality = 100;
open(vw);

L = 0.8;
W = 0.5;
A = 0.9;

xTrace = [];
yTrace = [];
zTrace = [];

step = 1;
if is_batch
    step = max(1, ceil(N / 300));
end

for k = 1:step:N
    R = eulerZYX(psi(k), theta(k), phi(k));

    p1 = R * [-L; 0; 0];
    p2 = R * [ L; 0; 0];
    q1 = R * [0; -W; 0];
    q2 = R * [0;  W; 0];

    c = [x(k); y(k); z(k)];

    P = [c + p1, c + p2];
    Q = [c + q1, c + q2];

    set(hBody1, 'XData', P(1, :), 'YData', P(2, :), 'ZData', P(3, :));
    set(hBody2, 'XData', Q(1, :), 'YData', Q(2, :), 'ZData', Q(3, :));
    set(hCenter, 'XData', x(k), 'YData', y(k), 'ZData', z(k));

    xTrace = [xTrace x(k)];
    yTrace = [yTrace y(k)];
    zTrace = [zTrace z(k)];
    set(hTrace, 'XData', xTrace, 'YData', yTrace, 'ZData', zTrace);

    ex = R * [A; 0; 0];
    ey = R * [0; A; 0];
    ez = R * [0; 0; A];

    set(hXb, 'XData', x(k), 'YData', y(k), 'ZData', z(k), ...
        'UData', ex(1), 'VData', ex(2), 'WData', ex(3));
    set(hYb, 'XData', x(k), 'YData', y(k), 'ZData', z(k), ...
        'UData', ey(1), 'VData', ey(2), 'WData', ey(3));
    set(hZb, 'XData', x(k), 'YData', y(k), 'ZData', z(k), ...
        'UData', ez(1), 'VData', ez(2), 'WData', ez(3));

    set(txt, 'String', sprintf(['t = %.2f s\nx = %.2f m, y = %.2f m, z = %.2f m\n' ...
        '\\phi = %.2f deg  \\theta = %.2f deg  \\psi = %.2f deg'], ...
        t(k), x(k), y(k), z(k), rad2deg(phi(k)), rad2deg(theta(k)), rad2deg(psi(k))));

    drawnow limitrate nocallbacks;
    frame = getframe(fig);
    writeVideo(vw, frame);
end

close(vw);
close(fig);

fprintf('\nVideo enregistree : %s\n', videoPath);

function data = simulate_open_loop_data()
    g = 9.81;
    cx = 0.05;
    cz = 0.08;

    dt = 0.01;
    Tend = 12;
    t = 0:dt:Tend;
    N = length(t);

    x = zeros(1, N);
    vx = zeros(1, N);
    z = zeros(1, N);
    vz = zeros(1, N);
    theta = zeros(1, N);

    vx(1) = 11;
    vz(1) = 13;
    theta(1) = deg2rad(22);

    impact_idx = N;

    for k = 2:N
        tk = t(k - 1);

        % Small prescribed pitch evolution without feedback control.
        if tk < 2.5
            theta(k) = deg2rad(22);
        elseif tk < 5.0
            theta(k) = deg2rad(12);
        elseif tk < 7.5
            theta(k) = deg2rad(4);
        else
            theta(k) = deg2rad(-8);
        end

        ax = -cx * vx(k - 1);
        az = -g - cz * vz(k - 1);

        vx(k) = vx(k - 1) + ax * dt;
        vz(k) = vz(k - 1) + az * dt;

        x(k) = x(k - 1) + vx(k) * dt;
        z(k) = z(k - 1) + vz(k) * dt;

        if z(k) <= 0 && k > 2
            z(k) = 0;
            impact_idx = k;
            break;
        end
    end

    data.t = t(1:impact_idx);
    data.x = x(1:impact_idx);
    data.z = z(1:impact_idx);
    data.theta = theta(1:impact_idx);
end

function R = eulerZYX(psi, theta, phi)
    Rz = [cos(psi) -sin(psi) 0;
          sin(psi)  cos(psi) 0;
          0         0        1];

    Ry = [ cos(theta) 0 sin(theta);
           0          1 0;
          -sin(theta) 0 cos(theta)];

    Rx = [1 0         0;
          0 cos(phi) -sin(phi);
          0 sin(phi)  cos(phi)];

    R = Rz * Ry * Rx;
end

function mode = figure_visibility(is_batch)
    if is_batch
        mode = 'off';
    else
        mode = 'on';
    end
end
