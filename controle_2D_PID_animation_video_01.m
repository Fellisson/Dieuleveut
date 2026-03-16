% 2D vehicle control with PID and MP4 export.
% This script simulates a simplified 2D vehicle with:
%   - horizontal and vertical translation;
%   - angular motion;
%   - altitude and angle PID control loops;
%   - wind disturbances;
%   - animated visualization and video recording.
%
% Output:
%   - videos/controle_2D_PID_animation.mp4
clc; clear; close all;

%% =========================
% PARAMETRES PHYSIQUES
%% =========================
m = 1.5;          % masse (kg)
J = 0.08;         % inertie (kg.m^2)
g = 9.81;         % gravité (m/s^2)

cx = 0.25;        % amortissement horizontal
cz = 0.90;        % amortissement vertical
cth = 0.15;       % amortissement angulaire

L = 1.2;          % longueur visuelle du véhicule

%% =========================
% SIMULATION
%% =========================
dt = 0.01;
Tend = 22;
t = 0:dt:Tend;
N = length(t);

%% =========================
% ETATS
%% =========================
x = zeros(1,N);        % position horizontale
vx = zeros(1,N);       % vitesse horizontale

z = zeros(1,N);        % altitude
vz = zeros(1,N);       % vitesse verticale

theta = zeros(1,N);    % angle (rad)
omega = zeros(1,N);    % vitesse angulaire

%% =========================
% COMMANDES
%% =========================
T = zeros(1,N);        % poussée
M = zeros(1,N);        % couple

%% =========================
% CONSIGNES
%% =========================
z_ref = zeros(1,N);
theta_ref = zeros(1,N);

z_ref(t >= 1)  = 8;                 % monter à 8 m
z_ref(t >= 12) = 12;                % puis à 12 m

theta_ref(t >= 6)  = deg2rad(10);   % inclinaison +10°
theta_ref(t >= 14) = deg2rad(-8);   % inclinaison -8°
theta_ref(t >= 18) = deg2rad(0);    % retour à 0°

%% =========================
% PID ALTITUDE
%% =========================
Kpz = 18;
Kiz = 4;
Kdz = 10;

%% =========================
% PID ANGLE
%% =========================
Kpt = 12;
Kit = 1.5;
Kdt = 4.5;

%% =========================
% VARIABLES PID
%% =========================
ez = zeros(1,N);
eth = zeros(1,N);

int_ez = 0;
int_eth = 0;

ez_prev = 0;
eth_prev = 0;

%% =========================
% PERTURBATIONS DE VENT
%% =========================
wx = zeros(1,N);     % vent horizontal
wz = zeros(1,N);     % vent vertical
wth = zeros(1,N);    % vent angulaire

for k = 1:N
    tk = t(k);

    % Rafale horizontale à droite
    if tk >= 4 && tk < 8
        wx(k) = 2.0;
    end

    % Rafale verticale
    if tk >= 9 && tk < 12
        wz(k) = 2.5;
    end

    % Vent angulaire oscillant
    if tk >= 13 && tk < 18
        wth(k) = 0.05*sin(2*pi*0.8*tk);
    end

    % Rafale horizontale opposée
    if tk >= 18 && tk < 20
        wx(k) = -2.8;
    end
end

%% =========================
% SATURATIONS
%% =========================
Tmin = 0;
Tmax = 45;

Mmin = -6;
Mmax = 6;

%% =========================
% BOUCLE DE SIMULATION
%% =========================
for k = 2:N

    % ----- PID altitude
    ez(k) = z_ref(k) - z(k-1);
    int_ez = int_ez + ez(k)*dt;
    dez = (ez(k) - ez_prev)/dt;

    Teq = m*g / max(cos(theta(k-1)), 0.2);   % compensation poids simple
    T(k) = Teq + Kpz*ez(k) + Kiz*int_ez + Kdz*dez;
    T(k) = min(max(T(k), Tmin), Tmax);

    if T(k) == Tmin || T(k) == Tmax
        int_ez = int_ez - ez(k)*dt;
    end
    ez_prev = ez(k);

    % ----- PID angle
    eth(k) = theta_ref(k) - theta(k-1);
    int_eth = int_eth + eth(k)*dt;
    deth = (eth(k) - eth_prev)/dt;

    M(k) = Kpt*eth(k) + Kit*int_eth + Kdt*deth;
    M(k) = min(max(M(k), Mmin), Mmax);

    if M(k) == Mmin || M(k) == Mmax
        int_eth = int_eth - eth(k)*dt;
    end
    eth_prev = eth(k);

    % ----- Dynamique translationnelle 2D
    ax = (T(k)*sin(theta(k-1)) - cx*vx(k-1) + wx(k)) / m;
    az = (T(k)*cos(theta(k-1)) - m*g - cz*vz(k-1) + wz(k)) / m;

    % ----- Dynamique angulaire
    alpha = (M(k) - cth*omega(k-1) + wth(k)) / J;

    % ----- Intégration Euler
    vx(k) = vx(k-1) + ax*dt;
    x(k) = x(k-1) + vx(k)*dt;

    vz(k) = vz(k-1) + az*dt;
    z(k) = z(k-1) + vz(k)*dt;
    if z(k) < 0
        z(k) = 0;
        vz(k) = 0;
    end

    omega(k) = omega(k-1) + alpha*dt;
    theta(k) = theta(k-1) + omega(k)*dt;
end

%% =========================
% FIGURE
%% =========================
fig = figure('Position',[70 50 1300 800],'Color','w');

% ---------- Animation principale
subplot(2,3,[1 2 4 5]);
hold on; grid on; box on;
title('Animation 2D du système contrôlé');
xlabel('Position horizontale x (m)');
ylabel('Altitude z (m)');

xmin = min(x) - 3;
xmax = max(x) + 3;
zmax = max([z_ref z]) + 4;

xlim([xmin xmax]);
ylim([0 zmax]);

% Sol
plot([xmin xmax], [0 0], 'k', 'LineWidth', 2);

% Consignes d'altitude
plot([xmin xmax], [8 8], '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1);
plot([xmin xmax], [12 12], '--', 'Color', [0.85 0.85 0.85], 'LineWidth', 1);

% Trace
hTrace = plot(NaN, NaN, 'r--', 'LineWidth', 1.5);

% Corps véhicule
hBody = plot([0 0], [0 0], 'b', 'LineWidth', 4);

% Centre
hCenter = plot(0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7);

% Flamme
hFlame = plot([0 0], [0 0], 'm', 'LineWidth', 3);

% Vecteur poussée
hThrust = quiver(0, 0, 0, 0, 0, 'Color', [0 0.5 0], 'LineWidth', 2, 'MaxHeadSize', 1.5);

% Vent horizontal visible
nWind = 6;
windY = linspace(1, zmax-1, nWind);
hWind = gobjects(1, nWind);
for i = 1:nWind
    hWind(i) = quiver(xmin + 1, windY(i), 0, 0, 0, ...
        'Color', [0.2 0.7 1], 'LineWidth', 1.5, 'MaxHeadSize', 1.2);
end

% Textes info
txt1 = text(xmax-0.2, zmax-0.5, '', 'HorizontalAlignment','right', 'FontWeight','bold');
txt2 = text(xmax-0.2, zmax-1.3, '', 'HorizontalAlignment','right', 'Color','b');
txt3 = text(xmax-0.2, zmax-2.1, '', 'HorizontalAlignment','right', 'Color','r');
txt4 = text(xmax-0.2, zmax-2.9, '', 'HorizontalAlignment','right', 'Color','m');
txt5 = text(xmax-0.2, zmax-3.7, '', 'HorizontalAlignment','right', 'Color',[0 0.5 0], 'FontWeight','bold');

% ---------- Altitude
subplot(2,3,3);
hold on; grid on; box on;
plot(t, z_ref, '--k', 'LineWidth', 1.5);
hAlt = plot(NaN, NaN, 'b', 'LineWidth', 1.8);
xlabel('Temps (s)');
ylabel('Altitude (m)');
title('Réponse en altitude');
legend('Consigne', 'Altitude', 'Location', 'best');

% ---------- Angle
subplot(2,3,6);
hold on; grid on; box on;
plot(t, rad2deg(theta_ref), '--k', 'LineWidth', 1.5);
hAng = plot(NaN, NaN, 'r', 'LineWidth', 1.8);
xlabel('Temps (s)');
ylabel('\theta (deg)');
title('Réponse angulaire');
legend('Consigne', 'Angle', 'Location', 'best');

%% =========================
% VIDEO MP4
%% =========================
if ~exist('videos', 'dir')
    mkdir('videos');
end

videoPath = fullfile('videos', 'controle_2D_PID_animation.mp4');
vw = VideoWriter(videoPath, 'MPEG-4');
vw.FrameRate = 30;
vw.Quality = 100;
open(vw);

%% =========================
% ANIMATION
%% =========================
step = 4;
xTrace = [];
zTrace = [];

for k = 1:step:N

    xc = x(k);
    zc = z(k);

    % Corps incliné
    x1 = xc - (L/2)*sin(theta(k));
    x2 = xc + (L/2)*sin(theta(k));

    z1 = zc - (L/2)*cos(theta(k));
    z2 = zc + (L/2)*cos(theta(k));

    subplot(2,3,[1 2 4 5]);

    % Corps
    set(hBody, 'XData', [x1 x2], 'YData', [z1 z2]);
    set(hCenter, 'XData', xc, 'YData', zc);

    % Flamme
    flameLength = 0.4 + 0.8*(T(k)/Tmax);
    xf1 = xc + 0.15*sin(theta(k));
    zf1 = zc + 0.15*cos(theta(k));
    xf2 = xc - flameLength*sin(theta(k));
    zf2 = zc - flameLength*cos(theta(k));
    set(hFlame, 'XData', [xf1 xf2], 'YData', [zf1 zf2]);

    % Vecteur poussée
    thrustScale = 0.12;
    uT = thrustScale*T(k)*sin(theta(k));
    vT = thrustScale*T(k)*cos(theta(k));
    set(hThrust, 'XData', xc, 'YData', zc, 'UData', uT, 'VData', vT);

    % Trace
    xTrace = [xTrace xc];
    zTrace = [zTrace zc];
    set(hTrace, 'XData', xTrace, 'YData', zTrace);

    % Vent visible
    meanWind = wx(k);
    windScale = 0.25;
    for i = 1:nWind
        set(hWind(i), ...
            'XData', xmin + 1, ...
            'YData', windY(i), ...
            'UData', windScale*meanWind, ...
            'VData', 0);
    end

    % Texte phase
    if t(k) < 1
        phase = 'Décollage initial';
    elseif t(k) >= 4 && t(k) < 8
        phase = 'Rafale horizontale';
    elseif t(k) >= 9 && t(k) < 12
        phase = 'Rafale verticale';
    elseif t(k) >= 13 && t(k) < 18
        phase = 'Vent angulaire oscillant';
    elseif t(k) >= 18 && t(k) < 20
        phase = 'Rafale opposée';
    else
        phase = 'Suivi PID normal';
    end

    set(txt1, 'String', sprintf('Temps = %.2f s', t(k)));
    set(txt2, 'String', sprintf('x = %.2f m, z = %.2f m', x(k), z(k)));
    set(txt3, 'String', sprintf('v_x = %.2f m/s, v_z = %.2f m/s', vx(k), vz(k)));
    set(txt4, 'String', sprintf('\\theta = %.2f deg | T = %.2f N | M = %.2f N.m', ...
        rad2deg(theta(k)), T(k), M(k)));
    set(txt5, 'String', sprintf('Vent: w_x = %.2f, w_z = %.2f, w_\\theta = %.3f | %s', ...
        wx(k), wz(k), wth(k), phase));

    % Courbes
    subplot(2,3,3);
    set(hAlt, 'XData', t(1:k), 'YData', z(1:k));

    subplot(2,3,6);
    set(hAng, 'XData', t(1:k), 'YData', rad2deg(theta(1:k)));

    drawnow;
    frame = getframe(fig);
    writeVideo(vw, frame);
end

close(vw);

fprintf('\nVidéo enregistrée : %s\n', videoPath);
