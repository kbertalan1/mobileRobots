densities = [0.15, 0.25];
seeds = 1:8;
methods = {'dijkstra', 'dfs', 'bfs', 'greedy', 'astar'};

rows = {};
for density = densities
    for seed = seeds
        [M, s, t] = maze_obstacles(30, 40, density, seed);

        % Dijkstra provides a reference optimal cost on this graph.
        [~, cref] = search_student_grid(M, s, t, 'dijkstra');

        for k = 1:numel(methods)
            [p, c, st] = search_student_grid(M, s, t, methods{k});
            rows(end+1,:) = {density, seed, methods{k}, ...
                c, numel(p)-1, st.expansions, ...
                1000*st.searchTime, c/cref}; 
        end
    end
end

results = cell2table(rows, 'VariableNames', ...
    {'Density','Seed','Method','Cost','Steps','Expansions', ...
    'TimeMs','CostRatio'});


for density = densities
    for k = 1:numel(methods)
        idx = results.Density == density & strcmp(results.Method, methods{k});
        fprintf('%.2f %-9s ratio %.3f, expansions %.1f, median %.3f ms\n', ...
            density, methods{k}, mean(results.CostRatio(idx)), ...
            mean(results.Expansions(idx)), median(results.TimeMs(idx)));
    end
end

% --- Experiment 3 Results Summary ---
% Optimality: Dijkstra, BFS, and A* always find the optimal path (Cost Ratio = 1.0). 
% DFS and Greedy find routes that are ~15-35% longer.
% Efficiency: A* is the best optimal algorithm, expanding significantly 
% fewer nodes than Dijkstra or BFS.
% Density effect: Higher obstacle density (0.25) makes Greedy struggle more 
% (cost ratio jumps to 1.35) because simple heuristics fail when forced 
% to make large detours.

% Table
%0.15 dijkstra  ratio 1.000, expansions 908.1, median 17.133 ms
%0.15 dfs       ratio 1.219, expansions 84.4, median 1.099 ms
%0.15 bfs       ratio 1.000, expansions 908.1, median 9.217 ms
%0.15 greedy    ratio 1.145, expansions 79.6, median 2.688 ms
%0.15 astar     ratio 1.000, expansions 707.5, median 15.867 ms
%0.25 dijkstra  ratio 1.000, expansions 786.1, median 12.896 ms
%0.25 dfs       ratio 1.191, expansions 126.5, median 1.254 ms
%0.25 bfs       ratio 1.000, expansions 786.1, median 7.744 ms
%0.25 greedy    ratio 1.348, expansions 122.0, median 3.229 ms
%0.25 astar     ratio 1.000, expansions 404.9, median 8.400 ms