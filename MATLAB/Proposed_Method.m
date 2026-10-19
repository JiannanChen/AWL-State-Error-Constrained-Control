
dt = 0.005;
T  = 100;
t  = (0:dt:T).';
N  = length(t);


gamma0 = 0.0;
X(1,:) = [0.2, 1.3, 0.1, gamma0];

%% 4. State and error constraints from the paper
B.Fl = [-2.0, -2.0, -pi];
B.Fh = [ 2.0,  2.0,  pi];

B.Hl = [-1.0, -1.0, -1.0];
B.Hh = [ 1.0,  1.0,  1.0];

% Delta parameters from the paper:
% Delta_l1 = -25, Delta_l2 = -25, Delta_l3 = -pi/2
% Delta_h1 = 25, Delta_h2 = 25, Delta_h3 = pi/2
B.Dl = [-25, -25, -pi/2];
B.Dh = [ 25,  25,  pi/2];

%% 5. Control gains from the paper
K.k1 = 0.15;
K.k2 = 0.10;
K.k3 = 0.20;

%% 6. Data allocation
Xd        = zeros(N, 3);     % [x1d, y1d, theta_fd]
XdDot     = zeros(N, 3);     % [x1d_dot, y1d_dot, theta_fd_dot]
E         = zeros(N, 3);     % [e1, e2, e3]
Phi       = zeros(N, 3);     % [phi1, phi2, phi3]
Eta1      = zeros(N, 3);     % [eta11, eta21, eta31]
Eta2      = zeros(N, 3);     % [eta12, eta22, eta32]
U         = zeros(N, 3);     % [ux, uy, u_theta]
VFD       = zeros(N, 1);
GammaDeg  = zeros(N, 1);

thetaFdPrev = NaN;

%% 7. Main simulation loop
for k = 1:N-1
    tk = t(k);

    %% 7.1 Desired trajectory from the current paper
    [xd_pos, xd_vel] = refSignalCurrentPaper(tk, ref);

    %% 7.2 Outer-loop constrained controls ux and uy
    [ux, phi1, e1, eta11, eta12] = scalarConstrainedControl( ...
        X(k,1), xd_pos(1), xd_vel(1), ...
        B.Hl(1), B.Hh(1), B.Fl(1), B.Fh(1), B.Dl(1), B.Dh(1), K.k1);

    [uy, phi2, e2, eta21, eta22] = scalarConstrainedControl( ...
        X(k,2), xd_pos(2), xd_vel(2), ...
        B.Hl(2), B.Hh(2), B.Fl(2), B.Fh(2), B.Dl(2), B.Dh(2), K.k2);

    vfd = sqrt(ux^2 + uy^2);

    %% 7.4 Inner-loop heading controller
    gamma = X(k,4);
    denVehicle = p.lf*cos(gamma) + p.lr;
    
    f = vfd*sin(gamma)/denVehicle;
    g = p.lr/denVehicle;

    uTheta = (((-K.k3*phi3 - eta32)/eta31) - f)/g;


%% 10. Figures
fig_x1 = figure('Name','x1 state and reference','Color','w');
plot(t, X(:,1), 'b--', 'LineWidth', 1.7); hold on;
plot(t, Xd(:,1), 'r-', 'LineWidth', 1);
plot(t, B.Fh(1)*ones(N,1), 'k-', 'LineWidth', 1.2);
plot(t, B.Fl(1)*ones(N,1), 'k-', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('x_1 (m)');
title('x_1 state, reference, and bounds');
legend('x_1','x_{1d}','upper bound','lower bound','Location','best');
setBoundedYLim(B.Fl(1), B.Fh(1));
if saveFigFlag, saveFigure(fig_x1, outDir, 'state_x1_reference_bounds_paper_strict'); end

fig_y1 = figure('Name','y1 state and reference','Color','w');
plot(t, X(:,2), 'b--', 'LineWidth', 1.7); hold on;
plot(t, Xd(:,2), 'r-', 'LineWidth', 1);
plot(t, B.Fh(2)*ones(N,1), 'k-', 'LineWidth', 1.2);
plot(t, B.Fl(2)*ones(N,1), 'k-', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('y_1 (m)');
title('y_1 state, reference, and bounds');
legend('y_1','y_{1d}','upper bound','lower bound','Location','best');
setBoundedYLim(B.Fl(2), B.Fh(2));
if saveFigFlag, saveFigure(fig_y1, outDir, 'state_y1_reference_bounds_paper_strict'); end

fig_theta = figure('Name','theta_f state and reference','Color','w');


thetaFPlot  = atan2(sin(X(:,3)),  cos(X(:,3)));
thetaFdPlot = atan2(sin(Xd(:,3)), cos(Xd(:,3)));

plot(t, thetaFPlot, 'b--', 'LineWidth', 1.7); hold on;
plot(t, thetaFdPlot, 'r-', 'LineWidth', 1);
plot(t, B.Fh(3)*ones(N,1), 'k-', 'LineWidth', 1.2);
plot(t, B.Fl(3)*ones(N,1), 'k-', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('\theta_f (rad)');
title('\theta_f state, reference, and bounds');
legend('\theta_f','\theta_{fd}','upper bound','lower bound','Location','best');
setBoundedYLim(B.Fl(3), B.Fh(3));
if saveFigFlag, saveFigure(fig_theta, outDir, 'state_theta_reference_bounds_paper_strict'); end

fig_e1 = figure('Name','tracking error e1','Color','w');
plot(t, E(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, B.Hh(1)*ones(N,1), 'k-', 'LineWidth', 1.2);
plot(t, B.Hl(1)*ones(N,1), 'k-', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('e_1 (m)');
title('tracking error e_1 and bounds');
legend('e_1','upper bound','lower bound','Location','best');
setBoundedYLim(B.Hl(1), B.Hh(1));
if saveFigFlag, saveFigure(fig_e1, outDir, 'tracking_error_e1_bounds_paper_strict'); end

fig_e2 = figure('Name','tracking error e2','Color','w');
plot(t, E(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(t, B.Hh(2)*ones(N,1), 'k-', 'LineWidth', 1.2);
plot(t, B.Hl(2)*ones(N,1), 'k-', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('e_2 (m)');
title('tracking error e_2 and bounds');
legend('e_2','upper bound','lower bound','Location','best');
setBoundedYLim(B.Hl(2), B.Hh(2));
if saveFigFlag, saveFigure(fig_e2, outDir, 'tracking_error_e2_bounds_paper_strict'); end

fig_e3 = figure('Name','tracking error e3','Color','w');
plot(t, E(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot(t, B.Hh(3)*ones(N,1), 'k-', 'LineWidth', 1.2);
plot(t, B.Hl(3)*ones(N,1), 'k-', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('e_3 (rad)');
title('tracking error e_3 and bounds');
legend('e_3','upper bound','lower bound','Location','best');
setBoundedYLim(B.Hl(3), B.Hh(3));
if saveFigFlag, saveFigure(fig_e3, outDir, 'tracking_error_e3_bounds_paper_strict'); end

fig_ux = figure('Name','control input ux','Color','w');
plot(t, U(:,1), 'b-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('u_x (m/s)');
title('control input u_x');
legend('u_x','Location','best');
if saveFigFlag, saveFigure(fig_ux, outDir, 'control_input_ux_paper_strict'); end

fig_uy = figure('Name','control input uy','Color','w');
plot(t, U(:,2), 'b-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('u_y (m/s)');
title('control input u_y');
legend('u_y','Location','best');
if saveFigFlag, saveFigure(fig_uy, outDir, 'control_input_uy_paper_strict'); end

fig_utheta = figure('Name','control input u_theta','Color','w');
plot(t, U(:,3), 'b-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('u_\theta (rad/s)');
title('control input u_\theta, no saturation');
legend('u_\theta','Location','best');
if saveFigFlag, saveFigure(fig_utheta, outDir, 'control_input_utheta_paper_strict'); end

fig_path = figure('Name','path tracking','Color','w');
plot(Xd(:,1), Xd(:,2), 'b--', 'LineWidth', 1.7); hold on;
plot(X(:,1), X(:,2), 'r-', 'LineWidth', 1);
axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');
title('path tracking');
legend('reference path','real path','Location','best');
if saveFigFlag, saveFigure(fig_path, outDir, 'path_tracking_paper_strict'); end

fig_gamma = figure('Name','articulation angle gamma','Color','w');
plot(t, GammaDeg, 'b-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('\gamma (deg)');
title('articulation angle \gamma, no clipping');
legend('\gamma','Location','best');
if saveFigFlag, saveFigure(fig_gamma, outDir, 'diagnosis_gamma_paper_strict'); end

fig_vfd = figure('Name','desired front-body speed vfd','Color','w');
plot(t, VFD, 'b-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('v_{fd} (m/s)');
title('desired front-body speed v_{fd}; ideal plant uses v_f = v_{fd}');
legend('v_{fd}','Location','best');
if saveFigFlag, saveFigure(fig_vfd, outDir, 'diagnosis_vfd_current_paper'); end

fprintf('\nSimulation finished. Figures are saved in:\n%s\n', outDir);

%% ====================== Local functions ======================

function [xd_pos, xd_vel] = refSignalCurrentPaper(t, ref)
    w = ref.omega;
    xd_pos = [sin(w*t), cos(w*t)];
    xd_vel = [w*cos(w*t), -w*sin(w*t)];
end

function [u, phi, e, eta1, eta2] = scalarConstrainedControl(x, xd, xd_dot, Hl, Hh, Fl, Fh, Dl, Dh, k)
    [phi, eta1, eta2, e] = barrierTerms(x, xd, xd_dot, Hl, Hh, Fl, Fh, Dl, Dh);
    u = (-k*phi - eta2)/eta1;
end

function [phi, eta1, eta2, e] = barrierTerms(x, xd, xd_dot, Hl, Hh, Fl, Fh, Dl, Dh)
    e = x - xd;

    dHh = Hh - e;
    dHl = Hl - e;
    dFh = Fh - x;
    dFl = Fl - x;

    phi = 0.25*(Hh/dHh + Hl/dHl + (Fh+Dh)/dFh + (Fl+Dl)/dFl)*e;

    eta1 = 0.25*( ...
        Hh^2/(dHh^2) + ...
        Hl^2/(dHl^2) + ...
        (Fl+Dl)*(Fl-xd)/(dFl^2) + ...
        (Fh+Dh)*(Fh-xd)/(dFh^2) );

    eta2 = -0.25*( ...
        Hh^2*xd_dot/(dHh^2) + ...
        Hl^2*xd_dot/(dHl^2) + ...
        (Fl+Dl)*xd_dot/dFl + ...
        (Fh+Dh)*xd_dot/dFh );
end

function [phi, eta1, eta2, e] = barrierTermsAngle(x, xd, xd_dot, Hl, Hh, Fl, Fh, Dl, Dh)
    % Periodic heading error: use the shortest angular difference.
    e = wrapToPi_local(x - xd);

    dHh = Hh - e;
    dHl = Hl - e;
    dFh = Fh - x;
    dFl = Fl - x;

    phi = 0.25*(Hh/dHh + Hl/dHl + (Fh+Dh)/dFh + (Fl+Dl)/dFl)*e;

    eta1 = 0.25*( ...
        Hh^2/(dHh^2) + ...
        Hl^2/(dHl^2) + ...
        (Fl+Dl)*(Fl-xd)/(dFl^2) + ...
        (Fh+Dh)*(Fh-xd)/(dFh^2) );

    eta2 = -0.25*( ...
        Hh^2*xd_dot/(dHh^2) + ...
        Hl^2*xd_dot/(dHl^2) + ...
        (Fl+Dl)*xd_dot/dFl + ...
        (Fh+Dh)*xd_dot/dFh );
end

function a = wrapToPi_local(a)
    a = mod(a + pi, 2*pi) - pi;
end

function r = deg2rad_local(d)
    r = d*pi/180;
end

function d = rad2deg_local(r)
    d = r*180/pi;
end

function setBoundedYLim(lowerBound, upperBound)
    axisPadding = 0.08*(upperBound - lowerBound);
    ylim([lowerBound - axisPadding, upperBound + axisPadding]);
end

function saveFigure(figHandle, outDir, baseName)
    pngName = fullfile(outDir, [baseName, '.png']);
    figName = fullfile(outDir, [baseName, '.fig']);

    try
        exportgraphics(figHandle, pngName, 'Resolution', 300);
    catch
        saveas(figHandle, pngName);
    end

    try
        savefig(figHandle, figName);
    catch
        saveas(figHandle, figName);
    end
end