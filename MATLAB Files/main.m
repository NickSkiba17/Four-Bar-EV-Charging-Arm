%MAIN A step-by-step guide to optimize your design
%
%  See also EXAMPLE_FUNCTION_CALLS

%% STEP 1: Start fresh!
%   TODO: insert code to clear the workspace, close figures, and load the
%   four-bar library.

% <------------INSERT YOUR CODE HERE------------>
clc;
clear;
close all;
fourbar_add_path();
%% STEP 2: Set up pos and L
%   TODO: insert code to correctly define pos and L.

% <------------INSERT YOUR CODE HERE------------>
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
%% STEP 3: Set up your initial condition x0
%   TODO: insert code that defines th1start and th1end
%
%   NOTE: th1start and th1end must lie between the values of th1bnds; if
%   possible, you should choose a start position that is inside the frame
%   and an end position that is outside the frame.  th1start does not have
%   to be less than th1end, it can be larger.
%
%   HINT: don't just blindly use the values of th1bnds.  The torques
%   requried at these values are not going to be pretty.  You would need
%   infinite torque at the input to move the mechanism, i.e., your
%   mechanical advantage is zero.  At which joint is the output torque
%   located?  A useful reference is https://roymech.org/Useful_Tables/Mechanics/Linkages.html#Mechanical%20Advantage

th1bnds = inputrange(pos, L);

fprintf('Valid input range: [%0.2f, %0.2f] radians or [%0.1f, %0.1f] degrees\n', ...
    th1bnds(1), th1bnds(2), rad2deg(th1bnds(1)), rad2deg(th1bnds(2)));

th1start = deg2rad(194);% <------------INSERT YOUR CODE HERE------------>;
th1end = deg2rad(42);% <------------INSERT YOUR CODE HERE------------>;

fprintf('Using input angles: th1start = %0.2f rad (%0.1f deg), th1end = %0.2f rad (%0.1f deg)\n', ...
    th1start, rad2deg(th1start), th1end, rad2deg(th1end));

x0 = [th1start; th1end; pos; L];

%% STEP 4: Animate and finalize your x0
%   TODO: insert code that animates your initial design using the
%   parameters for x0.  Add a title to your animation.
%
%   HINT: you'll eventually get tired of watching the animation as you do
%   some guess-and-check work.  Reduce the total animation time to a
%   shorter duraction than the default 5 seconds.

% <------------INSERT YOUR CODE HERE------------>
figure('Name', 'Initial Design Animation');
title(sprintf('Initial Four-Bar Design\n$\\theta_1$: [%0.1f$^\\circ$, %0.1f$^\\circ$]', ...
    rad2deg(th1start), rad2deg(th1end)), 'Interpreter', 'latex');
xlabel('x (m)', 'Interpreter', 'latex');
ylabel('y (m)', 'Interpreter', 'latex');
axis equal;
grid on;

totaltime = 5; % seconds
animate([th1start, th1end], pos, L, totaltime);
%% STEP 5: Plot torque
%   TODO: insert code that uses the Matlab built-in functions zeros and
%   linspace to create a 1 x n vector of zeros for the variable tau and a
%   linear spacing of points between th1start and th1end for theta1.  The
%   value of n must come from the struct returned from sharedparameters.
% 
%   Then write a loop that calls inputtorque and returns the return value
%   in the array tau as a function of input angle in theta1.  The loop must
%   iterature over all n values of theta1.
% 
%   Finally, plot theta1 vs tau.  Make sure to label the axes and give the
%   plot a descriptive title.

% params is a struct; you will need to access the value of n, which is a
% field in the struct params.  Assign the value of the field called n to
% the variable n in the script.  Search online if you don't know how to
% access fields in a struct.
params = sharedparameters(pos, L);
n = params.n;% <------------INSERT YOUR CODE HERE------------>;

% use zeros and linspace to define tau and theta1
tau = zeros(1,n);% <------------INSERT YOUR CODE HERE------------>;
theta1 = linspace(th1start, th1end, n);% <------------INSERT YOUR CODE HERE------------>;

% write a loop using an index i from 1 to n that stores the input torque
% returned from inputtorque into the vector tau for every value of the
% input angles in theta1.

% <------------INSERT YOUR CODE HERE------------>
for i = 1:n
    tau(i) = inputtorque(theta1(i), pos, L, params);
end

figure('Name', 'Input Torque Analysis');
hold on;


% plot the results, don't forget to label your plot!
% you can get the value of taumax from the params struct
taumax = params.servotorque;% <------------INSERT YOUR CODE HERE------------>;
plot(theta1([1, n]), [taumax, taumax], theta1([1, n]), -[taumax, taumax])
plot(theta1([1, n]), -[taumax, taumax], theta1([1, n]), -[taumax, taumax])

% <------------INSERT YOUR CODE HERE------------>
plot(theta1, tau, 'b-', 'LineWidth', 2, 'DisplayName', 'Required Torque');

hold off;
grid on;
xlabel('Input Angle $\theta_1$ (rad)', 'Interpreter', 'latex');
ylabel('Input Torque $\tau$ (Nm)', 'Interpreter', 'latex');
title('Input Torque vs Crank Angle', 'Interpreter', 'latex');
legend('Location', 'best');

% Check if torques exceed servo capability
if max(abs(tau)) > taumax
    warning('WARNING: Required torques exceed servo capability! Reduce link masses or change geometry.');
    fprintf('Max required torque: %.3f Nm\n', max(abs(tau)));
    fprintf('Servo max torque: %.3f Nm\n', taumax);
else
    fprintf('SUCCESS: All torques are within servo capability!\n');
    fprintf('Max required torque: %.3f Nm (%.1f%% of servo max)\n', ...
        max(abs(tau)), 100*max(abs(tau))/taumax);
end

%% STEP 6: Can you build a mechanism with values of m and rcom?
%   TODO: display the values of mass and rcom in the params struct.  If the
%   torques are too high for the motor, then go into sharedparameters.m and
%   change the values of m and rcom on lines 18 and 22, respectively, to
%   your target values.  For example, you can probably get away with very
%   light (e.g., lightweighted) crank and follower links, but will need a
%   slightly heavier and rigid coupler since it has to carry the
%   weight of the end effector as well.  You have experience working with
%   your frame to intuitively know how much 700-800 g feels like.  Use this
%   as a reference point for your links.  Hint: Your links should each be
%   much lighter than your frame.  Feel free to use the electronic scale in
%   the lab to further hone your intuition for how much things way.  For
%   example, can you accurately ballpark how much your phone weighs by just
%   holding it?
%
%   In the end, you will need to treat m and rcom as functional
%   specifications that are particular to your design.  You do not need to
%   exactly hit these values in your CAD design and prototype, but you
%   should be "close enough."  This is a fuzzy, widely used engineering
%   term that really just means that you will need to depend on your
%   intuition, experience, and ability to test your hypothesis on a mock-up
%   or actual prototype to determine if the values will work.
%
%   Why don't we care about Icom in our analysis?  Hint: We assume
%   that the servo moves the mechanism at a constant velocity.

% <------------INSERT YOUR CODE HERE------------>
fprintf('\n=== Link Properties ===\n');
fprintf('Crank (Link 1):\n');
fprintf('  Mass: %.4f kg (%.1f g)\n', params.m(1), params.m(1)*1000);
fprintf('  COM distance: %.4f m (%.1f mm)\n', params.rcom(1), params.rcom(1)*1000);
fprintf('  Moment of inertia: %.6f kg·m²\n\n', params.Icom(1));

fprintf('Coupler (Link 2):\n');
fprintf('  Mass: %.4f kg (%.1f g)\n', params.m(2), params.m(2)*1000);
fprintf('  COM distance: %.4f m (%.1f mm)\n', params.rcom(2), params.rcom(2)*1000);
fprintf('  Moment of inertia: %.6f kg·m²\n\n', params.Icom(2));

fprintf('Follower (Link 3):\n');
fprintf('  Mass: %.4f kg (%.1f g)\n', params.m(3), params.m(3)*1000);
fprintf('  COM distance: %.4f m (%.1f mm)\n', params.rcom(3), params.rcom(3)*1000);
fprintf('  Moment of inertia: %.6f kg·m²\n\n', params.Icom(3));

fprintf('NOTE: These masses are estimates based on MDF density of %.0f kg/m³\n', 750);
fprintf('      Adjust in sharedparameters.m if needed (lines 18-22)\n');
fprintf('      Your frame weighs ~700-800g, so each link should be much lighter!\n');

%% DIAGNOSTIC: Check initial design target distance
fprintf('\n=== Initial Design Target Check ===\n');
params = sharedparameters(pos, L);
evpos = params.evpos;

% Check end effector position at th1end
ee_initial = endeffector(th1end, pos, L);
dist_to_target = norm(ee_initial(:,2) - evpos);

fprintf('End effector at th1end:\n');
fprintf('  Position: (%.1f mm, %.1f mm)\n', ee_initial(1,2)*1000, ee_initial(2,2)*1000);
fprintf('  Target:   (%.1f mm, %.1f mm)\n', evpos(1)*1000, evpos(2)*1000);
fprintf('  Distance: %.2f mm\n', dist_to_target*1000);

if dist_to_target < 0.01  % Within 10mm
    fprintf('  ✓ EXCELLENT - Very close to target!\n');
    fprintf('  → Consider using initial design without optimization\n');
elseif dist_to_target < 0.05  % Within 50mm
    fprintf('  ✓ GOOD - Optimization might improve slightly\n');
else
    fprintf('  ⚠ FAR - Optimization needed\n');
end

% Prompt user
fprintf('\nProceed with optimization? (Press Ctrl+C to stop, Enter to continue)\n');
pause;

%% STEP 7: Are you satisfied with the results of STEPS 4-5?
% YES: Go to STEP 8.

% NO: Redo STEPS 2-6 to iterate on your design.  You'll need some intuition
% as to what might be satisfactory.  First, try completing/running the code
% in STEPS 8-10 and see if your guess converges.  If it doesn't, try the
% steps below.
% 
% Most convergence issues are due to the torque values being too large or
% undefined while the solver searches for an optimal solution.  Try sending
% an x0 with a torque profile that does not tend towards inifinity.  Below
% are other things to try.
%
% Is the motion of the end effector inside and outside of the frame?  A
% little inside or a little outside is typically OK, but end effector
% motions that are all inside the frame or outside the frame can sometimes
% lead to convergence issues.
% 
% Are you happy with the trajectory, but feel as is the mechanism is too
% far from the opening?  You can actually change the offset position of the
% four-bar: x1, y1, and gamma, without changing any aspect of the motion
% relative to the input joint.  Play around with different values of x1,
% y1, and gamma until you find an offset and orientation you are satisfied
% with.
%
% Are the torque values reasonable?  If you see values of 10^15, then
% you're risking having your mechanism locking at those positions.  Choose
% a new range of values for th1start and th1end and rerun your code.  You
% might also have to adjust m, rcom, and your link lengths.

%% STEP 8: Learn about fmincon
%   TODO: open and look through optimize.m in the optimization/ folder.
%   Line 81 shows that the function optimize is really just a wrapper
%   function around fmincon. Read the fmincon documentation.  Focus on the
%   headers: Syntax, Description, Input Arguments (skim most of it and skip
%   options and problem), and Output Arguments (x, fval, and exitflag
%   only).  You can skim everything else.  Answer the following questions:
%
%      Q5: Why is fmincon called with 4 empty braces?
%
%      Q6: Rewrite the following nonlinear constraint: x^2 <= 15, so that
%      it can be expressed as a constraint in the function handle nonlcon.
%
%      Q7: Open constraints.m in the optimization/ folder.  From the
%      documentation, you should be able to understand the variables c and
%      ceq.  How does fmincon ensure that an optimal solution has input
%      torques that the servo can generate? Write your answer in terms of
%      the line number(s) in the code.
%
%      Q8: Using your answer to Q7, rewrite the constraint mathematically
%      as a single line using input torque tau, servomax, and <= or >=
%      operators.
%
%      Q9: For what values of exitflag is xopt optimal after calling the
%      optimize function?  What should you do if exitflag is negative?
%
%      Q10: What parameters in your design does fmincon optimize in
%      optimize.m?  Additionally, and at a high-level, summarize what the
%      constraints gaurantee if xopt is feasible.
%
%      BONUS POINT:
%      QB2: Open costfunction.m in the optimization/ folder.  Why subtract
%      the offset and rotate the resulting vector in lines 46 and 49?

%% STEP 9: Optimize your parameters
%   TODO: insert code that will optimize your design.  Store the optimized
%   parameters in a variable xopt.  Then output the values of the
%   constraints.  Convince yourself that both the inequality and equality
%   constraints have been numerically satisfied.
%
%   The primary goal is to find lengths that satisfy the constraints
%   defined by the functional specifications.  Once it converges, evaluate
%   whether you can build a four-bar mechanism with the values of xopt.  If
%   not, make tweaks to lb and ub in optimize.m to constrain the range of
%   values that you want to work with.  You can exert further control on
%   the optimization by adding constraints in constraints.m.  For example,
%   you might want the crank and follower to always be the same lengths.
%   You would express this constraint in constraint.m in a manner similar
%   to what you did for Q6.  Yes, you can use the function to specify
%   linear constraints as well.


% <------------INSERT YOUR CODE HERE------------>
[xopt, fval, exitflag, output] = optimize(x0);

% Extract constraint values
[c, ceq] = constraints(xopt);

% Display results
fprintf('\n=== Optimization Results ===\n');
if exitflag > 0
    fprintf('SUCCESS! Found a feasible solution (exitflag = %d)\n', exitflag);
    fprintf('Solution is likely locally optimal.\n\n');
else
    fprintf('FAILED to find a feasible solution (exitflag = %d)\n', exitflag);
    fprintf('Try adjusting x0 or bounds in optimize.m\n\n');
end

fprintf('Cost function value (fval): %.6f\n\n', fval);

% Display parameter changes
fprintf('Parameter Comparison (Initial → Optimized):\n');
fprintf('  th1start:  %.3f rad (%.1f°) → %.3f rad (%.1f°)\n', ...
    x0(1), rad2deg(x0(1)), xopt(1), rad2deg(xopt(1)));
fprintf('  th1end:    %.3f rad (%.1f°) → %.3f rad (%.1f°)\n', ...
    x0(2), rad2deg(x0(2)), xopt(2), rad2deg(xopt(2)));
fprintf('  x1:        %.3f m → %.3f m\n', x0(3), xopt(3));
fprintf('  y1:        %.3f m → %.3f m\n', x0(4), xopt(4));
fprintf('  gamma:     %.3f rad (%.1f°) → %.3f rad (%.1f°)\n', ...
    x0(5), rad2deg(x0(5)), xopt(5), rad2deg(xopt(5)));
fprintf('  delta:     %.3f rad (%.1f°) → %.3f rad (%.1f°)\n', ...
    x0(6), rad2deg(x0(6)), xopt(6), rad2deg(xopt(6)));
fprintf('  L1 (crank):    %.3f m → %.3f m\n', x0(7), xopt(7));
fprintf('  L2 (coupler):  %.3f m → %.3f m\n', x0(8), xopt(8));
fprintf('  L3 (follower): %.3f m → %.3f m\n', x0(9), xopt(9));
fprintf('  L4 (ground):   %.3f m → %.3f m\n', x0(10), xopt(10));
fprintf('  ree (end eff): %.3f m → %.3f m\n\n', x0(11), xopt(11));

% Check constraints
fprintf('Inequality Constraints (must all be ≤ 0):\n');
for i = 1:length(c)
    if c(i) <= 0
        status = '✓';
    else
        status = '✗';
    end
    fprintf('  c(%2d) = %10.6f %s\n', i, c(i), status);
end

fprintf('\nEquality Constraints (must all be = 0):\n');
for i = 1:length(ceq)
    if abs(ceq(i)) < 1e-6
        status = '✓';
    else
        status = '✗';
    end
    fprintf('  ceq(%d) = %10.6f %s\n', i, ceq(i), status);
end
%% STEP 10: Animate and plot the torques of your optimal solution
%   TODO: insert code that store torques in a variable tauopt and angles in
%   a new variable theta1opt.

% <------------INSERT YOUR CODE HERE------------>
th1start_opt = xopt(1);
th1end_opt = xopt(2);
pos_opt = xopt(3:6);
L_opt = xopt(7:11);

% Get parameters for optimized design
params_opt = sharedparameters(pos_opt, L_opt);
n_opt = params_opt.n;

% Compute torques for optimized design
tauopt = zeros(1, n_opt);
theta1opt = linspace(th1start_opt, th1end_opt, n_opt);

for i = 1:n_opt
    tauopt(i) = inputtorque(theta1opt(i), pos_opt, L_opt, params_opt);
end

% Plot comparison of initial vs optimized torques
figure('Name', 'Torque Comparison');
hold on;

% Plot servo limits
taumax = params_opt.servotorque;
plot([theta1(1), theta1opt(end)], [taumax, taumax], 'r--', 'LineWidth', 1.5, 'DisplayName', 'Servo Limit');
plot([theta1(1), theta1opt(end)], -[taumax, taumax], 'r--', 'LineWidth', 1.5);

% Plot torque profiles
plot(theta1, tau, 'b-', 'LineWidth', 2, 'DisplayName', 'Initial Design');
plot(theta1opt, tauopt, 'g-', 'LineWidth', 2, 'DisplayName', 'Optimized Design');

hold off;
grid on;
xlabel('Input Angle $\theta_1$ (rad)', 'Interpreter', 'latex');
ylabel('Input Torque $\tau$ (Nm)', 'Interpreter', 'latex');
title('Torque Comparison: Initial vs Optimized', 'Interpreter', 'latex');
legend('Location', 'best');

fprintf('\n=== Optimized Torque Analysis ===\n');
fprintf('Max required torque: %.3f Nm (%.1f%% of servo max)\n', ...
    max(abs(tauopt)), 100*max(abs(tauopt))/taumax);

% Animate optimized design
figure('Name', 'Optimized Design Animation');
sgtitle('Comparison: Initial vs Optimized Design', 'Interpreter', 'latex');

subplot(1, 2, 1);
title(sprintf('Initial Design\n$\\theta_1$: [%0.1f$^\\circ$, %0.1f$^\\circ$]', ...
    rad2deg(x0(1)), rad2deg(x0(2))), 'Interpreter', 'latex');
xlabel('x (m)');
ylabel('y (m)');
axis equal;
grid on;
animate(x0(1:2), x0(3:6), x0(7:11), 1);

subplot(1, 2, 2);
title(sprintf('Optimized Design\n$\\theta_1$: [%0.1f$^\\circ$, %0.1f$^\\circ$]', ...
    rad2deg(xopt(1)), rad2deg(xopt(2))), 'Interpreter', 'latex');
xlabel('x (m)');
ylabel('y (m)');
axis equal;
grid on;
animate(xopt(1:2), xopt(3:6), xopt(7:11), 1);
%% STEP 11: Build a prototype?
%   TODO: if xopt looks good to you, then CAD and fabricate your robotic
%   arm charger.

%extra not needed code
fprintf('\n=== Ready to Fabricate? ===\n');
if exitflag > 0
    fprintf('✓ Your optimized design meets all constraints!\n');
    fprintf('\nNext steps:\n');
    fprintf('1. Export xopt parameters to CAD software\n');
    fprintf('2. Design link shapes with lightweighting features\n');
    fprintf('3. Check link masses match params.m assumptions\n');
    fprintf('4. Fabricate from 1/4" MDF\n');
    fprintf('5. Assemble and test!\n');
    
    % Save optimized parameters
    save('optimized_design.mat', 'xopt', 'x0', 'exitflag', 'fval');
    fprintf('\nOptimized parameters saved to: optimized_design.mat\n');
else
    fprintf('✗ Design did not converge. Try:\n');
    fprintf('  - Adjusting initial link lengths (pos, L)\n');
    fprintf('  - Changing th1start and th1end\n');
    fprintf('  - Modifying bounds in optimize.m\n');
    fprintf('  - Reducing link masses in sharedparameters.m\n');
end

fprintf('\n=== Script Complete ===\n');