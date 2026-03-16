% Vertical vehicle control with altitude and angle PID loops.
% This script simulates a simplified vehicle moving in the vertical plane.
% Two PID controllers are used:
%   - one for altitude tracking;
%   - one for attitude control.
%
% The simulation includes actuator saturation, simple anti-windup,
% vertical and angular disturbances, and a live animation of the response.
clc; clear; close all;

%% Paramètres physiques
m = 1.5;           % masse (kg)
J = 0.08;          % inertie (kg.m^2)
g = 9.81;          % gravité (m/s^2)

cz = 0.9;          % amortissement vertical
cth = 0.15;        % amortissement angulaire

L = 1.2;           % longueur visuelle du véhicule pour l'animation

%% Simulation
dt = 0.01;
Tend = 20;
t = 0:dt:Tend;
N = length(t);

%% Etats
z = zeros(1,N);        % altitude
vz = zeros(1,N);       % vitesse verticale
theta = zeros(1,N);    % angle (rad)
omega = zeros(1,N);    % vitesse angulaire

%% Commandes
T = zeros(1,N);        % poussée
M = zeros(1,N);        % couple

%% Consignes
z_ref = zeros(1,N);
theta_ref = zeros(1,N);

z_ref(t >= 1) = 10;                  % consigne altitude = 10 m
theta_ref(t >= 8) = deg2rad(8);      % angle = 8°
theta_ref(t >= 14) = deg2rad(-5);    % angle = -5°

%% PID altitude
Kpz = 18;
Kiz = 4;
Kdz = 10;

%% PID angle
Kpt = 12;
Kit = 1.5;
Kdt = 4.5;

%% Variables PID
ez = zeros(1,N);
eth = zeros(1,N);

int_ez = 0;
int_eth = 0;

ez_prev = 0;
eth_prev = 0;

%% Perturbations de vent
dz = zeros(1,N);       % vent vertical
dth = zeros(1,N);      % vent angulaire

for k = 1:N
    if t(k) >= 5 && t(k) < 9
        dz(k) = 2.5;
    end
    if t(k) >= 10 && t(k) < 16
        dth(k) = 0.04*sin(2*pi*0.8*t(k));
    end
end

%% Saturations
Tmin = 0;
Tmax = 40;
Mmin = -5;
Mmax = 5;

%% Boucle de simulation
for k = 2:N

    % PID altitude
    ez(k) = z_ref(k) - z(k-1);
    int_ez = int_ez + ez(k)*dt;
    dez = (ez(k) - ez_prev)/dt;

    Teq = m*g;
    T(k) = Teq + Kpz*ez(k) + Kiz*int_ez + Kdz*dez;
    T(k) = min(max(T(k), Tmin), Tmax);

    if T(k) == Tmin || T(k) == Tmax
        int_ez = int_ez - ez(k)*dt;
    end

    ez_prev = ez(k);

    % PID angle
    eth(k) = theta_ref(k) - theta(k-1);
    int_eth = int_eth + eth(k)*dt;
    deth = (eth(k) - eth_prev)/dt;

    M(k) = Kpt*eth(k) + Kit*int_eth + Kdt*deth;
    M(k) = min(max(M(k), Mmin), Mmax);

    if M(k) == Mmin || M(k) == Mmax
        int_eth = int_eth - eth(k)*dt;
    end

    eth_prev = eth(k);

    % Dynamique
    az = (T(k)*cos(theta(k-1)) - m*g - cz*vz(k-1) + dz(k)) / m;
    alpha = (M(k) - cth*omega(k-1) + dth(k)) / J;

    % Euler
    vz(k) = vz(k-1) + az*dt;
    z(k) = z(k-1) + vz(k)*dt;

    omega(k) = omega(k-1) + alpha*dt;
    theta(k) = theta(k-1) + omega(k)*dt;
end

%% Figure avec animation
fig = figure('Position',[100 80 1100 750],'Color','w');

% Sous-figure animation
subplot(2,2,[1 3]);
hold on; grid on; box on;
title('Animation du système contrôlé');
xlabel('Position horizontale visuelle');
ylabel('Altitude z (m)');

xlim([-3 3]);
ylim([0 max([z_ref z])+3]);

% Sol
plot([-3 3], [0 0], 'k', 'LineWidth', 2);

% Consigne altitude
hRef = plot([-3 3], [z_ref(end) z_ref(end)], '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);

% Corps du véhicule
hBody = plot([0 0],[0 0],'b','LineWidth',4);

% Centre du véhicule
hCenter = plot(0,0,'ro','MarkerFaceColor','r','MarkerSize',7);

% Flamme/poussée
hFlame = plot([0 0],[0 0],'m','LineWidth',3);

% Trace altitude
hTrace = plot(NaN, NaN, 'r--', 'LineWidth', 1.2);

% Textes
txt1 = text(2.8, max([z_ref z])+2.2, '', 'HorizontalAlignment','right', 'FontWeight','bold');
txt2 = text(2.8, max([z_ref z])+1.4, '', 'HorizontalAlignment','right', 'Color','b');
txt3 = text(2.8, max([z_ref z])+0.6, '', 'HorizontalAlignment','right', 'Color','r');
txt4 = text(2.8, max([z_ref z])-0.2, '', 'HorizontalAlignment','right', 'Color',[0 0.5 0]);

% Sous-figure altitude
subplot(2,2,2);
hold on; grid on; box on;
plot(t, z_ref, '--k', 'LineWidth', 1.5);
hAlt = plot(NaN, NaN, 'b', 'LineWidth', 1.8);
xlabel('Temps (s)');
ylabel('Altitude (m)');
title('Réponse en altitude');
legend('Consigne', 'Altitude', 'Location', 'best');

% Sous-figure angle
subplot(2,2,4);
hold on; grid on; box on;
plot(t, rad2deg(theta_ref), '--k', 'LineWidth', 1.5);
hAng = plot(NaN, NaN, 'r', 'LineWidth', 1.8);
xlabel('Temps (s)');
ylabel('\theta (deg)');
title('Réponse angulaire');
legend('Consigne', 'Angle', 'Location', 'best');

%% Animation
step = 5;  % accélère un peu l’animation

zTrace = [];
xTrace = [];

for k = 1:step:N

    % Position du centre
    xc = 0;
    zc = z(k);

    % Extrémités du corps incliné
    x1 = xc - (L/2)*sin(theta(k));
    x2 = xc + (L/2)*sin(theta(k));

    z1 = zc - (L/2)*cos(theta(k));
    z2 = zc + (L/2)*cos(theta(k));

    % Mise à jour corps
    subplot(2,2,[1 3]);
    set(hBody, 'XData', [x1 x2], 'YData', [z1 z2]);
    set(hCenter, 'XData', xc, 'YData', zc);

    % Flamme de poussée
    flameLength = 0.3 + 0.5*(T(k)/Tmax);
    xf1 = xc + 0.15*sin(theta(k));
    zf1 = zc + 0.15*cos(theta(k));
    xf2 = xc - flameLength*sin(theta(k));
    zf2 = zc - flameLength*cos(theta(k));
    set(hFlame, 'XData', [xf1 xf2], 'YData', [zf1 zf2]);

    % Trace
    zTrace = [zTrace zc];
    xTrace = [xTrace xc];
    set(hTrace, 'XData', xTrace, 'YData', zTrace);

    % Texte phase
    if t(k) < 1
        phase = 'Décollage initial';
    elseif t(k) >= 5 && t(k) < 9
        phase = 'Perturbation : rafale verticale';
    elseif t(k) >= 10 && t(k) < 16
        phase = 'Perturbation : vent angulaire';
    else
        phase = 'Suivi PID normal';
    end

    set(txt1, 'String', sprintf('Temps = %.2f s', t(k)));
    set(txt2, 'String', sprintf('Altitude = %.2f m | Consigne = %.2f m', z(k), z_ref(k)));
    set(txt3, 'String', sprintf('\\theta = %.2f deg | Consigne = %.2f deg', rad2deg(theta(k)), rad2deg(theta_ref(k))));
    set(txt4, 'String', sprintf('Poussée = %.2f N | Couple = %.2f N.m | %s', T(k), M(k), phase));

    % Mise à jour courbes temporelles
    subplot(2,2,2);
    set(hAlt, 'XData', t(1:k), 'YData', z(1:k));

    subplot(2,2,4);
    set(hAng, 'XData', t(1:k), 'YData', rad2deg(theta(1:k)));

    drawnow;
    pause(0.01);
end
