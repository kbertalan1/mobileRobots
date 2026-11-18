P  = cc_params('Track', 'oval');
T  = cc_track(P);
CM = cc_car_model(T, P);
[poses, info] = cc_lap_plan(CM);

%{
  leg 1 -> (2.90, 1.00):  13 poses,  1.48 m,    36 expansions, arrives at  +56.0 deg
  leg 2 -> (1.60, 1.50):  14 poses,  1.66 m,    25 expansions, arrives at +179.5 deg
  leg 3 -> (0.30, 1.00):  14 poses,  1.60 m,    65 expansions, arrives at +235.5 deg
  leg 4 -> (1.60, 0.50):  14 poses,  1.67 m,    59 expansions, arrives at +348.0 deg
%}
%%

run('experiments/experiment_oval_lap.m')

%{
============ 2. Drive ============

  route length   6.05 m
  tightest bend  R = 0.104 m
  speed          0.90 .. 2.14 m/s
  LAP TIME       5.48 s
  top speed      1.44 m/s

  To watch it:  cc_animate_lap(T, log, 'Route', XY)
  When this works, go to the real track:
      run('experiments/experiment_cc_lap.m')
%}
%%

XY = cc_resample_path(poses(:,1:2), 0.04);
vFlat = repmat(cc_top_speed(P), size(XY,1), 1); % max speed
log = cc_simulate(T, [XY, vFlat]);
fprintf('crashed: %d at (%.2f, %.2f) after %.2f s\n', ...
    log.crashed, log.crashAt(1), log.crashAt(2), log.t(end));

% crash 
%%
for saf = [0.00 0.02 0.03 0.04 0.05 0.07 0.08 0.10 0.12]
    P  = cc_params('Track', 'oval', 'safety', saf);
    T  = cc_track(P);
    CM = cc_car_model(T, P);
    try
        [poses, info] = cc_lap_plan(CM, 'Verbose', false);
    catch err
        fprintf('safety %.2f : %s\n', saf, err.message(1:min(40,end)));
        continue
    end
    XY   = cc_resample_path(poses(:,1:2), 0.04);
    vRef = cc_speed_profile(XY, P);
    log  = cc_simulate(T, [XY, vRef]);
    if log.crashed
        fprintf('safety %.2f : CRASH at (%.2f, %.2f)\n', saf, log.crashAt(1), log.crashAt(2));
    else
        fprintf('safety %.2f : %.2f s, clearance %4.1f mm, %3d expansions\n', ...
            saf, log.lapTime, 1000*cc_clearance(T, log), info.expansions);
    end
end

%{
safety 0.00 : Leg 4 could not be planned: from (0.33, 
safety 0.02 : 5.36 s, clearance 25.0 mm, 178 expansions
safety 0.03 : 5.44 s, clearance 14.9 mm, 167 expansions
safety 0.04 : 5.48 s, clearance 41.6 mm, 194 expansions
safety 0.05 : 5.48 s, clearance 41.6 mm, 185 expansions
safety 0.07 : 5.64 s, clearance 45.7 mm, 143 expansions
safety 0.08 : 5.64 s, clearance 52.8 mm, 161 expansions
safety 0.10 : 5.52 s, clearance 68.5 mm, 160 expansions
safety 0.12 : 5.36 s, clearance 84.2 mm, 131 expansions
%}
%%
for saf = [0.00 0.02 0.05]
    P = cc_params('Track','oval','safety',saf);
    CM = cc_car_model(cc_track(P), P);
    pose = P.start;
    fprintf('safety %.2f:', saf);
    goals = [P.checkpoints; P.start(1), P.start(2)];
    for k = 1:size(goals,1)
        lp = cc_search_car(CM, pose, goals(k,:), P.cpTol);
        if isempty(lp), fprintf('  leg %d FAILS\n', k); break, end
        pose = lp(end,:);
        fprintf('  CP%d at %+6.1f deg', k, mod(rad2deg(pose(3)),360));
    end
    fprintf('\n');
end

%{
safety 0.00:  CP1 at  +56.0 deg  CP2 at +179.0 deg  CP3 at +212.0 deg  leg 4 FAILS
safety 0.02:  CP1 at  +56.0 deg  CP2 at +179.0 deg  CP3 at +222.9 deg  CP4 at +357.4 deg
safety 0.05:  CP1 at  +56.0 deg  CP2 at +179.5 deg  CP3 at +235.5 deg  CP4 at +348.0 deg
%}
%%
safeties = 0.02:0.02:0.4; % Dense sweep with 2 mm resolution
valid_results = []; % Store successful runs: [safety, lapTime, clearance]

fprintf('Running sweep (%d values). This might take a minute...\n', length(safeties));

for saf = safeties
    P  = cc_params('Track', 'oval', 'safety', saf);
    T  = cc_track(P);
    CM = cc_car_model(T, P);

    try
        % Verbose=false to keep the console clean
        [poses, ~] = cc_lap_plan(CM, 'Verbose', false);
    catch
        % If no route is found (e.g., car gets stuck), skip to the next value
        continue; 
    end

    XY   = cc_resample_path(poses(:,1:2), 0.04);
    vRef = cc_speed_profile(XY, P);
    log  = cc_simulate(T, [XY, vRef]);

    if ~log.crashed
        % If the car finished the lap safely, save the data
        valid_results(end+1, :) = [saf, log.lapTime, 1000*cc_clearance(T, log)]; %#ok<SAGROW>
    end
end

% --- Plot the results ---
if ~isempty(valid_results)
    figure('Name', 'Safety Margin Analysis', 'Position', [150 150 900 450]);

    % Left axis: Lap Time (we want to minimize this)
    yyaxis left
    plot(valid_results(:,1), valid_results(:,2), '-o', 'LineWidth', 1.5, 'MarkerSize', 4);
    ylabel('Lap Time (s) [Lower is better]');

    % Right axis: Clearance from the wall (we want to maximize this)
    yyaxis right
    plot(valid_results(:,1), valid_results(:,3), '-x', 'LineWidth', 1.5, 'MarkerSize', 4);
    ylabel('Clearance (mm) [Higher is better]');

    xlabel('P.safety margin (m)');
    title('Impact of Safety Margin on Lap Time and Clearance');
    grid on;

    % Print the best lap time
    [minTime, idx] = min(valid_results(:,2));
    fprintf('\nFastest lap: %.2f s (Safety = %.3f m, Clearance = %.1f mm)\n', ...
        minTime, valid_results(idx, 1), valid_results(idx, 3));
else
    disp('No successful laps were found.');
end