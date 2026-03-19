% Guided ballistic 3D vehicle with PID attitude control.
% The vehicle is stabilized by PID during a short powered phase, then it
% follows a ballistic trajectory under gravity until impact.
%
% Outputs:
%   - out/videos/controle_3D_ballistique_avec_PID.mp4
%   - out/images/controle_3D_ballistique_avec_PID.png

clc; clear; close all;

base_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(fileparts(base_dir));
videos_dir = fullfile(project_dir, 'out', 'videos');
images_dir = fullfile(project_dir, 'out', 'images');
is_batch = ~usejava('desktop');

if ~exist(videos_dir, 'dir')
    mkdir(videos_dir);
end
if ~exist(images_dir, 'dir')
    mkdir(images_dir);
end

%% Parameters
m = 2.0;
g = 9.81;
I = diag([0.12, 0.14, 0.20]);
cv = 0.03;
cw = 0.14;

dt = 0.01;
Tend = 26;
t = 0:dt:Tend;
N = numel(t);

tburn = 4.2;
Tmax = 55;
tilt_max = deg2rad(22);

%% States
x = zeros(1, N); y = zeros(1, N); z = zeros(1, N);
vx = zeros(1, N); vy = zeros(1, N); vz = zeros(1, N);
phi = zeros(1, N); theta = zeros(1, N); psi = zeros(1, N);
p = zeros(1, N); q = zeros(1, N); r = zeros(1, N);

%% Commands
Tcmd = zeros(1, N);
Mx = zeros(1, N); My = zeros(1, N); Mz = zeros(1, N);

%% References
theta_ref = zeros(1, N);
psi_ref = zeros(1, N);

theta_ref(t >= 0.5) = deg2rad(17);
theta_ref(t >= 2.0) = deg2rad(24);
theta_ref(t >= 4.2) = deg2rad(5);

psi_ref(t >= 1.2) = deg2rad(12);
psi_ref(t >= 3.0) = deg2rad(20);
psi_ref(t >= 5.0) = deg2rad(15);

%% Disturbances
wx = zeros(1, N); wy = zeros(1, N); wz = zeros(1, N);
mp = zeros(1, N); mq = zeros(1, N); mr = zeros(1, N);

for k = 1:N
    tk = t(k);
    if tk >= 2.5 && tk < 5.5
        wx(k) = 0.6;
    end
    if tk >= 6 && tk < 9
        wy(k) = -0.5;
    end
    if tk >= 7 && tk < 11
        wz(k) = -0.8;
    end
    if tk >= 3 && tk < 6
        mq(k) = 0.05 * sin(2 * pi * 0.8 * tk);
    end
    if tk >= 8 && tk < 12
        mr(k) = 0.04 * sin(2 * pi * 0.5 * tk);
    end
end

%% PID gains
Kpp = 8.0; Kip = 0.7; Kdp = 2.8;
Kpt = 8.5; Kit = 0.8; Kdt = 3.0;
Kps = 5.5; Kis = 0.4; Kds = 1.8;

ip = 0; itg = 0; is = 0;
ephi_prev = 0; etheta_prev = 0; epsi_prev = 0;

%% Initial impulse
vx(1) = 6;
vy(1) = 0;
vz(1) = 8;

%% Simulation
impact_idx = N;

for k = 2:N
    tk = t(k);

    phi_ref = 0;
    theta_ref_k = clamp(theta_ref(k), -tilt_max, tilt_max);
    psi_ref_k = psi_ref(k);

    if tk <= tburn
        Tcmd(k) = Tmax;
    else
        Tcmd(k) = 0;
    end

    ephi = phi_ref - phi(k - 1);
    etheta = theta_ref_k - theta(k - 1);
    epsi = wrap_to_pi_local(psi_ref_k - psi(k - 1));

    ip = ip + ephi * dt;
    itg = itg + etheta * dt;
    is = is + epsi * dt;

    dephi = (ephi - ephi_prev) / dt;
    detheta = (etheta - etheta_prev) / dt;
    depsi = (epsi - epsi_prev) / dt;

    Mx(k) = clamp(Kpp * ephi + Kip * ip + Kdp * dephi, -2.5, 2.5);
    My(k) = clamp(Kpt * etheta + Kit * itg + Kdt * detheta, -2.8, 2.8);
    Mz(k) = clamp(Kps * epsi + Kis * is + Kds * depsi, -2.0, 2.0);

    ephi_prev = ephi;
    etheta_prev = etheta;
    epsi_prev = epsi;

    R = eulerZYX(psi(k - 1), theta(k - 1), phi(k - 1));
    thrust_world = R * [0; 0; Tcmd(k)];

    vel = [vx(k - 1); vy(k - 1); vz(k - 1)];
    drag = cv * norm(vel) * vel;
    acc = (thrust_world - drag) / m - [0; 0; g] + [wx(k); wy(k); wz(k)] / m;

    omega = [p(k - 1); q(k - 1); r(k - 1)];
    tau = [Mx(k); My(k); Mz(k)] + [mp(k); mq(k); mr(k)];
    omega_dot = I \ (tau - cross(omega, I * omega) - cw * omega);
    euler_dot = euler_rates(phi(k - 1), theta(k - 1), omega);

    vx(k) = vx(k - 1) + acc(1) * dt;
    vy(k) = vy(k - 1) + acc(2) * dt;
    vz(k) = vz(k - 1) + acc(3) * dt;

    x(k) = x(k - 1) + vx(k) * dt;
    y(k) = y(k - 1) + vy(k) * dt;
    z(k) = z(k - 1) + vz(k) * dt;

    p(k) = p(k - 1) + omega_dot(1) * dt;
    q(k) = q(k - 1) + omega_dot(2) * dt;
    r(k) = r(k - 1) + omega_dot(3) * dt;

    phi(k) = phi(k - 1) + euler_dot(1) * dt;
    theta(k) = theta(k - 1) + euler_dot(2) * dt;
    psi(k) = wrap_to_pi_local(psi(k - 1) + euler_dot(3) * dt);

    if z(k) <= 0 && k > 20
        z(k) = 0;
        impact_idx = k;
        break;
    end
end

t = t(1:impact_idx);
x = x(1:impact_idx); y = y(1:impact_idx); z = z(1:impact_idx);
vx = vx(1:impact_idx); vy = vy(1:impact_idx); vz = vz(1:impact_idx);
phi = phi(1:impact_idx); theta = theta(1:impact_idx); psi = psi(1:impact_idx);
Tcmd = Tcmd(1:impact_idx);

%% Figure and video
fig = figure('Position', [80 60 1350 820], 'Color', 'w', ...
    'Visible', figure_visibility(is_batch));

ax3 = subplot(2, 3, [1 2 4 5]);
hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
view(ax3, 36, 24);
xlabel(ax3, 'X (m)');
ylabel(ax3, 'Y (m)');
zlabel(ax3, 'Z (m)');
title(ax3, 'Projectile guide en 3D avec phase balistique');
set(ax3, 'Color', [0.78 0.88 0.98]);

xmin = min(x) - 3; xmax = max(x) + 3;
ymin = min(y) - 3; ymax = max(y) + 3;
zmax = max(z) + 4;

xlim(ax3, [xmin xmax]);
ylim(ax3, [ymin ymax]);
zlim(ax3, [0 zmax]);

[Xg, Yg] = meshgrid(linspace(xmin, xmax, 80), linspace(ymin, ymax, 80));
terrain_shape = 0.18 * sin(0.12 * Xg) .* cos(0.18 * Yg) + ...
    0.12 * exp(-((Xg - mean(x)).^2 + (Yg - mean(y)).^2) / 180);
Zg = max(0, terrain_shape);
surf(ax3, Xg, Yg, Zg, 'FaceColor', [0.42 0.62 0.30], ...
    'EdgeColor', 'none', 'FaceAlpha', 1.0);
colormap(ax3, summer);

% Sky haze layers
[Xs, Ys] = meshgrid(linspace(xmin, xmax, 30), linspace(ymin, ymax, 30));
Zs1 = (zmax + 1.0) * ones(size(Xs));
Zs2 = (zmax + 2.2) * ones(size(Xs));
surf(ax3, Xs, Ys, Zs1, 'FaceColor', [0.82 0.91 1.00], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.18);
surf(ax3, Xs, Ys, Zs2, 'FaceColor', [0.93 0.97 1.00], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.10);

% Stylized clouds
cloud_centers = [xmin + 0.20 * (xmax - xmin), ymin + 0.20 * (ymax - ymin), zmax - 1.2; ...
                 xmin + 0.62 * (xmax - xmin), ymin + 0.68 * (ymax - ymin), zmax - 0.8; ...
                 xmin + 0.78 * (xmax - xmin), ymin + 0.30 * (ymax - ymin), zmax - 1.6];
for ic = 1:size(cloud_centers, 1)
    draw_cloud(ax3, cloud_centers(ic, :), 1.8 + 0.2 * ic);
end

% Stylized trees
tree_positions = [xmin + 2.5, ymin + 1.5; ...
                  xmin + 6.0, ymax - 2.0; ...
                  xmax - 3.5, ymin + 2.2; ...
                  xmax - 6.5, ymax - 1.8; ...
                  mean(x), ymin + 1.0];
for itree = 1:size(tree_positions, 1)
    draw_tree(ax3, tree_positions(itree, 1), tree_positions(itree, 2), 0);
end

plot3(ax3, x, y, z, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2);
hTrace = plot3(ax3, NaN, NaN, NaN, 'r', 'LineWidth', 2);
hBody1 = plot3(ax3, NaN, NaN, NaN, 'b', 'LineWidth', 3);
hBody2 = plot3(ax3, NaN, NaN, NaN, 'k', 'LineWidth', 2);
hCenter = plot3(ax3, x(1), y(1), z(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
hThrust = quiver3(ax3, x(1), y(1), z(1), 0, 0, 0, 0, ...
    'Color', [0 0.5 0], 'LineWidth', 2, 'MaxHeadSize', 0.8);
hImpact = plot3(ax3, x(end), y(end), z(end), 'x', 'Color', [0.8 0.2 0.1], ...
    'LineWidth', 2, 'MarkerSize', 10, 'Visible', 'off');

txt = text(ax3, xmax - 0.5, ymax - 0.5, zmax - 0.3, '', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', 'FontWeight', 'bold');

axAlt = subplot(2, 3, 3);
hold(axAlt, 'on'); grid(axAlt, 'on'); box(axAlt, 'on');
hAlt = plot(axAlt, NaN, NaN, 'b', 'LineWidth', 1.8);
hSpeed = plot(axAlt, NaN, NaN, 'r', 'LineWidth', 1.4);
xlabel(axAlt, 'Temps (s)');
ylabel(axAlt, 'Altitude / Vitesse');
title(axAlt, 'Evolution verticale');
legend(axAlt, 'Altitude z', 'Vitesse |v|', 'Location', 'best');

axAtt = subplot(2, 3, 6);
hold(axAtt, 'on'); grid(axAtt, 'on'); box(axAtt, 'on');
hPhi = plot(axAtt, NaN, NaN, 'm', 'LineWidth', 1.6);
hTheta = plot(axAtt, NaN, NaN, 'c', 'LineWidth', 1.6);
hPsi = plot(axAtt, NaN, NaN, 'k', 'LineWidth', 1.6);
xlabel(axAtt, 'Temps (s)');
ylabel(axAtt, 'Angle (deg)');
title(axAtt, 'Attitude pendant le vol');
legend(axAtt, 'phi', 'theta', 'psi', 'Location', 'best');

videoPath = fullfile(videos_dir, 'controle_3D_ballistique_avec_PID.mp4');
tempVideoPath = fullfile(videos_dir, 'controle_3D_ballistique_avec_PID_tmp.mp4');
if exist(tempVideoPath, 'file')
    delete(tempVideoPath);
end
vw = VideoWriter(tempVideoPath, 'MPEG-4');
vw.FrameRate = 24;
vw.Quality = 85;
open(vw);

L = 0.9;
W = 0.65;
thrust_scale = 0.05;

xTrace = [];
yTrace = [];
zTrace = [];
step = 2;
if is_batch
    step = max(step, ceil(numel(t) / 140));
end

for k = 1:step:numel(t)
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

    thrust_vec = R * [0; 0; thrust_scale * Tcmd(k)];
    set(hThrust, 'XData', x(k), 'YData', y(k), 'ZData', z(k), ...
        'UData', thrust_vec(1), 'VData', thrust_vec(2), 'WData', thrust_vec(3));

    speed = sqrt(vx(k)^2 + vy(k)^2 + vz(k)^2);
    phase = 'Phase propulsee';
    if t(k) > tburn
        phase = 'Phase balistique';
    end
    if k == numel(t)
        set(hImpact, 'Visible', 'on');
        phase = 'Impact au sol';
    end

    set(txt, 'String', sprintf(['t = %.2f s\nx = %.2f  y = %.2f  z = %.2f m\n' ...
        'phi = %.1f deg  theta = %.1f deg  psi = %.1f deg\n' ...
        '|v| = %.2f m/s | %s'], ...
        t(k), x(k), y(k), z(k), rad2deg(phi(k)), rad2deg(theta(k)), rad2deg(psi(k)), speed, phase));

    set(hAlt, 'XData', t(1:k), 'YData', z(1:k));
    set(hSpeed, 'XData', t(1:k), 'YData', sqrt(vx(1:k).^2 + vy(1:k).^2 + vz(1:k).^2));

    set(hPhi, 'XData', t(1:k), 'YData', rad2deg(phi(1:k)));
    set(hTheta, 'XData', t(1:k), 'YData', rad2deg(theta(1:k)));
    set(hPsi, 'XData', t(1:k), 'YData', rad2deg(psi(1:k)));

    drawnow limitrate nocallbacks;
    frame = getframe(fig);
    writeVideo(vw, frame);
end

saveas(fig, fullfile(images_dir, 'controle_3D_ballistique_avec_PID.png'));
close(vw);
close(fig);

if exist(videoPath, 'file')
    delete(videoPath);
end
if exist(tempVideoPath, 'file')
    movefile(tempVideoPath, videoPath, 'f');
end

fprintf('\nVideo enregistree : %s\n', videoPath);

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

function eul_dot = euler_rates(phi, theta, omega)
    ctheta = max(cos(theta), 1e-3);
    T = [1, sin(phi) * tan(theta), cos(phi) * tan(theta);
         0, cos(phi),             -sin(phi);
         0, sin(phi) / ctheta,     cos(phi) / ctheta];
    eul_dot = T * omega;
end

function y = clamp(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

function angle = wrap_to_pi_local(angle)
    angle = mod(angle + pi, 2 * pi) - pi;
end

function mode = figure_visibility(is_batch)
    if is_batch
        mode = 'off';
    else
        mode = 'on';
    end
end

function draw_cloud(ax, center, scale)
    [sx, sy, sz] = sphere(12);
    offsets = [0 0 0; 0.9 0.2 0.1; -0.8 0.1 -0.05; 0.25 0.55 0.12];
    radii = [1.0, 0.75, 0.70, 0.62] * scale;
    for i = 1:size(offsets, 1)
        surf(ax, center(1) + offsets(i, 1) * scale + sx * radii(i), ...
            center(2) + offsets(i, 2) * scale + sy * 0.55 * radii(i), ...
            center(3) + offsets(i, 3) * scale + sz * 0.35 * radii(i), ...
            'FaceColor', [1 1 1], 'EdgeColor', 'none', 'FaceAlpha', 0.42);
    end
end

function draw_tree(ax, xt, yt, zt)
    [ct_x, ct_y, ct_z] = cylinder(0.16, 10);
    trunk_h = 1.3;
    surf(ax, xt + ct_x, yt + ct_y, zt + ct_z * trunk_h, ...
        'FaceColor', [0.45 0.28 0.12], 'EdgeColor', 'none');

    [sx, sy, sz] = sphere(10);
    surf(ax, xt + sx * 0.85, yt + sy * 0.85, zt + trunk_h + 0.9 + sz * 1.0, ...
        'FaceColor', [0.16 0.48 0.16], 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    surf(ax, xt + 0.35 + sx * 0.55, yt - 0.15 + sy * 0.55, ...
        zt + trunk_h + 1.2 + sz * 0.65, ...
        'FaceColor', [0.20 0.56 0.20], 'EdgeColor', 'none', 'FaceAlpha', 0.92);
end
