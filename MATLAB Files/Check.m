%CHECK - Comprehensive diagnostic tool for four-bar design
%   Run this BEFORE main.m to validate your design and identify issues
%
%   This script will:
%   - Visualize your mechanism at key positions
%   - Check if end effector reaches the target
%   - Validate torque requirements
%   - Test Grashof condition
%   - Predict optimization success
clc;
clear;
close all;
fourbar_add_path();

fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║   FOUR-BAR LINKAGE COMPREHENSIVE DIAGNOSTIC CHECK             ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

%% STEP 1: Define your design (COPY FROM main.m STEP 2)
fprintf('STEP 1: Loading Design Parameters...\n');

% Frame dimensions (FIXED)
framewidth = 0.42;  % 420 mm
frameheight = 0.48; % 480 mm
evport_x = framewidth + 0.1524; % 572.4 mm from origin
evport_y = 0.240; % 240 mm from ground

% YOUR DESIGN PARAMETERS - ADJUST THESE
x1 = 0.2805; 
y1 = 0.132;
gamma = deg2rad(-27.2);
delta = deg2rad(25.5);

pos = [x1; y1; gamma; delta];

L1 = 0.068; % crank length - starts motion
L2 = 0.075; % coupler length - carries end effector
L3 = 0.085; % follower length
L4 = 0.087; % ground link length
ree = 0.2493; % end effector length - reaches to charging port

L = [L1; L2; L3; L4; ree];

fprintf('  ✓ Design loaded\n');
fprintf('    Mounting: (%.0f mm, %.0f mm)\n', x1*1000, y1*1000);
fprintf('    Gamma: %.1f°, Delta: %.1f°\n', rad2deg(gamma), rad2deg(delta));
fprintf('    Links (mm): L1=%.0f, L2=%.0f, L3=%.0f, L4=%.0f, ree=%.0f\n\n', ...
    L1*1000, L2*1000, L3*1000, L4*1000, ree*1000);

%% STEP 2: Check valid input range
fprintf('STEP 2: Checking Input Range...\n');

[th1bnds, linkage_type] = inputrange(pos, L);
range_span_deg = rad2deg(th1bnds(2) - th1bnds(1));

fprintf('  Valid range: [%.2f, %.2f] rad = [%.1f, %.1f]°\n', ...
    th1bnds(1), th1bnds(2), rad2deg(th1bnds(1)), rad2deg(th1bnds(2)));
fprintf('  Range span: %.1f°\n', range_span_deg);
fprintf('  Linkage type from bounds: %s\n', linkage_type);

if range_span_deg < 60
    fprintf('  ⚠ WARNING: Very small range (<60°) - may not reach target!\n');
elseif range_span_deg < 90
    fprintf('  ⚠ CAUTION: Small range (<90°) - verify target reachability\n');
elseif range_span_deg > 180
    fprintf('  ✓ EXCELLENT: Large range (>180°) gives great flexibility\n');
else
    fprintf('  ✓ GOOD: Moderate range (90-180°)\n');
end
fprintf('\n');

%% STEP 3: Check Grashof condition
fprintf('STEP 3: Grashof Analysis...\n');

Lsum = [L1, L2, L3, L4];
[s, s_idx] = min(Lsum);
[l, l_idx] = max(Lsum);
p_q = sum(Lsum) - s - l;
grashof_sum = s + l;

link_names = {'Crank (L1)', 'Coupler (L2)', 'Follower (L3)', 'Ground (L4)'};

fprintf('  Shortest link: %.0f mm (%s)\n', s*1000, link_names{s_idx});
fprintf('  Longest link:  %.0f mm (%s)\n', l*1000, link_names{l_idx});
fprintf('  s + l = %.0f mm\n', grashof_sum*1000);
fprintf('  p + q = %.0f mm\n', p_q*1000);

grashof_satisfied = grashof_sum <= p_q;

if grashof_satisfied
    fprintf('  ✓ Grashof SATISFIED (s+l ≤ p+q)\n');
    if s_idx == 1
        fprintf('  Type: CRANK-ROCKER (crank can rotate fully)\n');
        fprintf('  → Good for this application!\n');
    elseif s_idx == 4
        fprintf('  Type: DRAG-LINK (both rotate fully)\n');
        fprintf('  → May work, but unusual for this application\n');
    else
        fprintf('  Type: ROCKER-ROCKER or other configuration\n');
        fprintf('  → Verify motion meets requirements\n');
    end
else
    fprintf('  ✗ Grashof NOT satisfied (s+l > p+q)\n');
    fprintf('  Type: DOUBLE-ROCKER (both links rock back/forth)\n');
    fprintf('  → This may limit your range of motion\n');
end
fprintf('\n');

%% STEP 4: Visualize at key positions
fprintf('STEP 4: Visualizing Mechanism at Key Positions...\n');


th1_start = deg2rad(194);
th1_mid = mean(th1bnds);
th1_end = deg2rad(42);
test_angles = [th1_start, th1_mid, th1_end];
test_names = {'START (retracted)', 'MIDDLE (transition)', 'END (extended)'};

figure('Position', [50, 50, 1600, 500], 'Name', 'Mechanism Positions');

distances = zeros(1, 3);
ee_positions = zeros(3, 2);

for i = 1:3
    th1 = test_angles(i);
    
    subplot(1, 3, i);
    hold on;
    
    % Draw frame (blue rectangle)
    rectangle('Position', [0, 0, framewidth, frameheight], ...
        'EdgeColor', 'b', 'LineWidth', 2.5);
    
    % Draw slit (magenta thick line)
    params = sharedparameters(pos, L);
    slit = params.slit;
    plot(slit(1, :), slit(2, :), 'm-', 'LineWidth', 4);
    text(framewidth - 0.05, mean(slit(2,:)) - 0.04, 'SLIT', ...
        'Color', 'm', 'FontSize', 10, 'FontWeight', 'bold');
    
    % Draw EV charging port (large red X)
    plot(evport_x, evport_y, 'rx', 'MarkerSize', 20, 'LineWidth', 4);
    text(evport_x + 0.02, evport_y + 0.03, 'TARGET', ...
        'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');
    
    % Draw four-bar linkage
    [crank, coupler, follower, ground] = rodpositions(th1, pos, L);
    
    % Crank (red)
    plot(crank(1,:), crank(2,:), 'r-', 'LineWidth', 3);
    plot(crank(1,:), crank(2,:), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % Coupler (green)
    plot(coupler(1,:), coupler(2,:), 'g-', 'LineWidth', 3);
    plot(coupler(1,:), coupler(2,:), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    
    % Follower (blue)
    plot(follower(1,:), follower(2,:), 'b-', 'LineWidth', 3);
    plot(follower(1,:), follower(2,:), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    % Ground (black)
    plot(ground(1,:), ground(2,:), 'k-', 'LineWidth', 3);
    plot(ground(1,:), ground(2,:), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    
    % Draw end effector (CYAN - thick and visible!)
    ee = endeffector(th1, pos, L);
    plot([ee(1,1), ee(1,2)], [ee(2,1), ee(2,2)], ...
        'c-', 'LineWidth', 5);
    plot(ee(1,2), ee(2,2), 'co', 'MarkerSize', 12, 'MarkerFaceColor', 'c');
    
    % Draw line from EE tip to target
    plot([ee(1,2), evport_x], [ee(2,2), evport_y], ...
        'k:', 'LineWidth', 1.5);
    
    % Calculate distance to target
    dist = norm([ee(1,2) - evport_x; ee(2,2) - evport_y]);
    distances(i) = dist;
    ee_positions(i, :) = [ee(1,2), ee(2,2)];
    
    hold off;
    axis equal;
    grid on;
    xlim([-0.05, evport_x + 0.1]);
    ylim([-0.05, frameheight + 0.05]);
    
    title_str = sprintf('%s\n\\theta_1 = %.1f°\nEE: (%.0f, %.0f) mm\nDist to target: %.1f mm', ...
        test_names{i}, rad2deg(th1), ee(1,2)*1000, ee(2,2)*1000, dist*1000);
    title(title_str, 'FontSize', 10);
    xlabel('x (m)');
    ylabel('y (m)');
    
    % Add legend
    if i == 1
        legend('Frame', 'Slit', 'Target', 'Crank', '', 'Coupler', '', ...
            'Follower', '', 'Ground', '', 'End Effector', '', ...
            'Location', 'northwest', 'FontSize', 8);
    end
end

fprintf('  Position Analysis:\n');
for i = 1:3
    fprintf('    %s:\n', test_names{i});
    fprintf('      End effector: (%.0f mm, %.0f mm)\n', ...
        ee_positions(i,1)*1000, ee_positions(i,2)*1000);
    fprintf('      Distance to target: %.1f mm', distances(i)*1000);
    
    if i == 3  % End position
        if distances(i) < 0.01  % Within 10mm
            fprintf(' ✓ EXCELLENT!\n');
        elseif distances(i) < 0.02  % Within 20mm
            fprintf(' ✓ GOOD (optimizer should fix)\n');
        elseif distances(i) < 0.05  % Within 50mm
            fprintf(' ⚠ OK (needs optimization)\n');
        else
            fprintf(' ✗ TOO FAR! (may not converge)\n');
        end
    else
        if ee_positions(i,1) < framewidth
            fprintf(' (inside frame) ✓\n');
        else
            fprintf(' (outside frame)\n');
        end
    end
end
fprintf('\n');

%% STEP 5: Check torque requirements
fprintf('STEP 5: Torque Analysis...\n');

params = sharedparameters(pos, L);
n_check = 50;

tau_vals = zeros(1, n_check);
angles = linspace(th1_start, th1_end, n_check);

for i = 1:n_check
    tau_vals(i) = inputtorque(angles(i), pos, L, params);
end

max_tau = max(abs(tau_vals));
servo_max = params.servotorque;
utilization = 100 * max_tau / servo_max;

fprintf('  Max torque required: %.4f Nm\n', max_tau);
fprintf('  Servo max torque: %.4f Nm\n', servo_max);
fprintf('  Utilization: %.1f%%\n', utilization);

% Plot torque profile
figure('Name', 'Torque Profile');
hold on;
plot(rad2deg(angles), tau_vals, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Required Torque');
yline(servo_max, 'r--', 'LineWidth', 2, 'DisplayName', 'Servo Max');
yline(-servo_max, 'r--', 'LineWidth', 2, 'DisplayName', 'Servo Min');
hold off;
grid on;
xlabel('Input Angle θ_1 (degrees)');
ylabel('Input Torque τ (Nm)');
title('Torque Requirements vs Input Angle');
legend('Location', 'best');

if max_tau > servo_max
    fprintf('  ✗ FAILS: Torque exceeds servo limits!\n');
    fprintf('  → Action: Reduce masses in sharedparameters.m\n');
    fprintf('  → Or: Use shorter crank (L1) for better mechanical advantage\n');
    torque_ok = false;
elseif max_tau > 0.95 * servo_max
    fprintf('  ⚠ MARGINAL: Very close to servo limit (>95%%)\n');
    fprintf('  → Recommend: Reduce masses slightly for safety margin\n');
    torque_ok = false;
elseif max_tau > 0.85 * servo_max
    fprintf('  ⚠ ACCEPTABLE: Close to limit but should work (>85%%)\n');
    torque_ok = true;
else
    fprintf('  ✓ EXCELLENT: Comfortable margin below servo limit\n');
    torque_ok = true;
end

fprintf('\n  Link masses (from sharedparameters.m):\n');
fprintf('    Crank:   %.1f g\n', params.m(1)*1000);
fprintf('    Coupler: %.1f g\n', params.m(2)*1000);
fprintf('    Follower: %.1f g\n', params.m(3)*1000);
fprintf('    TOTAL:   %.1f g\n', sum(params.m)*1000);

if sum(params.m)*1000 > 200
    fprintf('  ⚠ Total mass >200g may be heavy - consider more aggressive lightweighting\n');
elseif sum(params.m)*1000 > 100
    fprintf('  ✓ Total mass reasonable for MDF construction\n');
else
    fprintf('  ✓ Very light - good for low torque requirements\n');
end
fprintf('\n');

%% STEP 6: Convergence prediction
fprintf('STEP 6: Convergence Prediction...\n');

score = 0;
max_score = 10;

% Check 1: Range adequacy (2 points)
if range_span_deg > 120
    score = score + 2;
    fprintf('  ✓ [2/2] Range: Excellent (>120°)\n');
elseif range_span_deg > 90
    score = score + 1;
    fprintf('  ○ [1/2] Range: Adequate (90-120°)\n');
else
    fprintf('  ✗ [0/2] Range: Too small (<90°)\n');
end

% Check 2: End position accuracy (3 points)
end_dist_mm = distances(3) * 1000;
if end_dist_mm < 20
    score = score + 3;
    fprintf('  ✓ [3/3] Target: Excellent (<20mm)\n');
elseif end_dist_mm < 50
    score = score + 2;
    fprintf('  ○ [2/3] Target: Good (<50mm)\n');
elseif end_dist_mm < 100
    score = score + 1;
    fprintf('  ○ [1/3] Target: Marginal (<100mm)\n');
else
    fprintf('  ✗ [0/3] Target: Too far (>100mm)\n');
end

% Check 3: Torque adequacy (3 points)
if torque_ok && utilization < 85
    score = score + 3;
    fprintf('  ✓ [3/3] Torque: Excellent (<85%% utilization)\n');
elseif torque_ok
    score = score + 2;
    fprintf('  ○ [2/3] Torque: Adequate (85-95%%)\n');
elseif utilization < 110
    score = score + 1;
    fprintf('  ○ [1/3] Torque: Marginal (95-110%%)\n');
else
    fprintf('  ✗ [0/3] Torque: Exceeds limits significantly\n');
end

% Check 4: Grashof (1 point)
if grashof_satisfied && s_idx == 1
    score = score + 1;
    fprintf('  ✓ [1/1] Grashof: Crank-rocker (ideal)\n');
elseif grashof_satisfied
    score = score + 0.5;
    fprintf('  ○ [0.5/1] Grashof: Satisfied but not crank-rocker\n');
else
    fprintf('  ✗ [0/1] Grashof: Not satisfied\n');
end

% Check 5: Start position inside frame (1 point)
if ee_positions(1,1) < framewidth - 0.02
    score = score + 1;
    fprintf('  ✓ [1/1] Start: Well inside frame\n');
elseif ee_positions(1,1) < framewidth
    score = score + 0.5;
    fprintf('  ○ [0.5/1] Start: Just inside frame\n');
else
    fprintf('  ✗ [0/1] Start: Outside frame\n');
end

fprintf('\n  OVERALL SCORE: %.1f / %.0f\n', score, max_score);

if score >= 9
    fprintf('  ✓✓✓ EXCELLENT: Very likely to converge!\n');
    fprintf('  → Proceed to main.m with confidence\n');
elseif score >= 7
    fprintf('  ✓✓ GOOD: Should converge with optimization\n');
    fprintf('  → Proceed to main.m\n');
elseif score >= 5
    fprintf('  ○ FAIR: May converge, but could need adjustments\n');
    fprintf('  → Try running main.m, be ready to iterate\n');
elseif score >= 3
    fprintf('  ⚠ POOR: Unlikely to converge without changes\n');
    fprintf('  → Recommend adjusting design parameters first\n');
else
    fprintf('  ✗ VERY POOR: Will not converge\n');
    fprintf('  → Must revise design before attempting optimization\n');
end

fprintf('\n');

%% STEP 6.5: Detailed Constraint Analysis
fprintf('STEP 6.5: Constraint Analysis...\n');

% Create x0 vector for constraint checking
th1start_check = th1_start;
th1end_check = th1_end;
x0_check = [th1start_check; th1end_check; pos; L];

% Evaluate constraints
try
    [c, ceq] = constraints(x0_check);
    
    fprintf('\n  ═══════════════════════════════════════════════════════\n');
    fprintf('  INEQUALITY CONSTRAINTS (must be ≤ 0 to satisfy)\n');
    fprintf('  ═══════════════════════════════════════════════════════\n');
    
    % Constraint descriptions
    c_names = {
        'c(1):  th1start ≥ lower input limit';
        'c(2):  th1start ≤ upper input limit';
        'c(3):  th1end ≥ lower input limit';
        'c(4):  th1end ≤ upper input limit';
        'c(5):  Angular range within servo capability';
        'c(6):  4-bar left edge inside frame';
        'c(7):  4-bar right edge inside frame';
        'c(8):  4-bar bottom edge inside frame';
        'c(9):  4-bar top edge inside frame';
        'c(10): End effector left edge inside frame';
        'c(11): End effector right edge inside frame';
        'c(12): End effector bottom edge inside frame';
        'c(13): End effector top edge inside frame';
        'c(14): End effector above slit lower bound';
        'c(15): End effector below slit upper bound';
        'c(16): End effector crosses frame boundary';
        'c(17): Min torque ≥ -servo limit';
        'c(18): Max torque ≤ +servo limit';
        'c(19): Transmission angle ≥ 40°';
        'c(20): Transmission angle ≤ 140°'
    };
    
    num_satisfied = 0;
    num_violated = 0;
    critical_violations = {};
    
    for i = 1:length(c)
        if c(i) <= 1e-6
            status = '✓ PASS';
            num_satisfied = num_satisfied + 1;
            status_color = '';
        elseif c(i) <= 0.01
            status = '⚠ MARGINAL';
            num_violated = num_violated + 1;
            status_color = '';
        else
            status = '✗ FAIL';
            num_violated = num_violated + 1;
            status_color = ' ← CRITICAL';
            critical_violations{end+1} = sprintf('c(%d)', i);
        end
        
        fprintf('  %-45s: %10.6f %s%s\n', c_names{i}, c(i), status, status_color);
    end
    
    fprintf('\n  ═══════════════════════════════════════════════════════\n');
    fprintf('  EQUALITY CONSTRAINTS (must be = 0 to satisfy)\n');
    fprintf('  ═══════════════════════════════════════════════════════\n');
    
    ceq_names = {
        'ceq(1): X-position at target (572.4 mm)';
        'ceq(2): Y-position at target (240 mm)'
    };
    
    for i = 1:length(ceq)
        if abs(ceq(i)) < 1e-6
            status = '✓ EXACT';
        elseif abs(ceq(i)) < 0.001  % Within 1mm
            status = '✓ CLOSE';
        elseif abs(ceq(i)) < 0.01   % Within 10mm
            status = '○ NEAR';
        else
            status = '✗ FAR';
        end
        
        % Convert to mm for easier interpretation
        error_mm = ceq(i) * 1000;
        
        fprintf('  %-40s: %10.6f m (%+.1f mm) %s\n', ...
            ceq_names{i}, ceq(i), error_mm, status);
    end
    
    fprintf('\n  ═══════════════════════════════════════════════════════\n');
    fprintf('  CONSTRAINT SUMMARY\n');
    fprintf('  ═══════════════════════════════════════════════════════\n');
    fprintf('  Inequality constraints:  %d/%d satisfied\n', num_satisfied, length(c));
    fprintf('  Equality constraints:    ');
    if abs(ceq(1)) < 0.01 && abs(ceq(2)) < 0.01
        fprintf('Both within 10mm ✓\n');
    else
        fprintf('Not satisfied ✗\n');
    end
    
    % Overall feasibility
    is_feasible = (num_violated == 0) && (abs(ceq(1)) < 0.01) && (abs(ceq(2)) < 0.01);
    
    fprintf('\n  INITIAL DESIGN FEASIBILITY: ');
    if is_feasible
        fprintf('✓ FEASIBLE (all constraints satisfied!)\n');
        fprintf('  → Your initial design is already excellent!\n');
        fprintf('  → Optimization may improve slightly but not necessary\n');
    elseif num_violated <= 3
        fprintf('○ NEARLY FEASIBLE (%d minor violations)\n', num_violated);
        fprintf('  → Optimization should easily fix these\n');
    elseif num_violated <= 5
        fprintf('⚠ MODERATELY FEASIBLE (%d violations)\n', num_violated);
        fprintf('  → Optimization needed to satisfy all constraints\n');
    else
        fprintf('✗ NOT FEASIBLE (%d violations)\n', num_violated);
        fprintf('  → Significant design changes needed before optimization\n');
    end
    
    % Highlight critical violations
    if ~isempty(critical_violations)
        fprintf('\n  CRITICAL VIOLATIONS THAT NEED ATTENTION:\n');
        for i = 1:length(critical_violations)
            fprintf('    • %s\n', critical_violations{i});
        end
    end
    
    fprintf('\n');
    
catch ME
    fprintf('  ✗ ERROR evaluating constraints: %s\n', ME.message);
    fprintf('  → Your design may have fundamental issues\n');
    fprintf('  → Check that link lengths form valid four-bar\n\n');
end
%% STEP 7: Recommendations
fprintf('STEP 7: Recommendations...\n');

issues_found = false;

if end_dist_mm > 50
    fprintf('  ⚠ End effector misses target by %.0f mm\n', end_dist_mm);
    fprintf('     → Try: Increase L2 (coupler: %.0f→%.0f mm) or ree (EE: %.0f→%.0f mm)\n', ...
        L2*1000, (L2+0.02)*1000, ree*1000, (ree+0.03)*1000);
    fprintf('     → Or: Adjust delta angle (currently %.1f°)\n', rad2deg(delta));
    issues_found = true;
end

if ~torque_ok
    fprintf('  ⚠ Torques exceed or approach servo limits\n');
    fprintf('     → Reduce lightweighting_factor in sharedparameters.m (try 0.3 or 0.25)\n');
    fprintf('     → Or: Use shorter L1 crank (%.0f→%.0f mm) for better mechanical advantage\n', ...
        L1*1000, max(L1*0.8, 0.05)*1000);
    issues_found = true;
end

if ee_positions(1,1) > framewidth
    fprintf('  ⚠ Start position outside frame!\n');
    fprintf('     → Adjust th1_start closer to th1_min\n');
    fprintf('     → Or: Move mounting point (x1: %.0f→%.0f mm)\n', ...
        x1*1000, (x1-0.02)*1000);
    issues_found = true;
end

if range_span_deg < 90
    fprintf('  ⚠ Small input range may limit motion\n');
    fprintf('     → Try different link length ratios\n');
    fprintf('     → Current: L1/L4=%.2f, aim for 0.4-0.8\n', L1/L4);
    issues_found = true;
end

if ~issues_found
    fprintf('  ✓ No major issues detected!\n');
    fprintf('  → Your design looks good - proceed to optimization\n');
end

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║   END OF DIAGNOSTIC CHECK                                     ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n');

if score >= 7
    fprintf('\n✓ Ready to run main.m\n\n');
else
    fprintf('\n⚠ Consider revising design parameters before running main.m\n\n');
end