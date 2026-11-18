M2 = maze_random(10, 15, 42);
s2 = sub2ind(size(M2), 2, 2);
t2 = sub2ind(size(M2), 20, 30);
methods = {'dfs', 'bfs', 'greedy', 'dijkstra', 'astar'};

figure('Name', 'Perfect maze', 'Position', [100 100 1500 430]);
for k = 1:numel(methods)
    method = methods{k};
    [p, c, st] = search_student_grid(M2, s2, t2, method, 'Conn', 4);

    subplot(1, numel(methods), k)
    plot_grid(M2, 'Expanded', st.order, 'Path', p, 'Start', s2, 'Goal', t2, ...
        'Title', sprintf('%s: cost %.0f', method, c));
end



% --- Part B: 8-connected movement (BFS vs Dijkstra) ---
% Ensure the obstacle map is generated first
[M, s, t] = maze_obstacles(30, 40, 0.25, 7);

[pB, cB, stB] = search_student_grid(M, s, t, 'bfs', 'Conn', 8);
[pD, cD, stD] = search_student_grid(M, s, t, 'dijkstra', 'Conn', 8);

figure('Name', 'Part B: 8-connected (BFS vs Dijkstra)', 'Position', [200 200 1000 450]);
subplot(1, 2, 1)
plot_grid(M, 'Expanded', stB.order, 'Path', pB, 'Start', s, 'Goal', t, ...
    'Title', sprintf('BFS: %d steps, cost %.3f', numel(pB)-1, cB));
subplot(1, 2, 2)
plot_grid(M, 'Expanded', stD.order, 'Path', pD, 'Start', s, 'Goal', t, ...
    'Title', sprintf('Dijkstra: %d steps, cost %.3f', numel(pD)-1, cD));


% --- Part C: Deliberately invalid A* heuristic (Octile vs Manhattan) ---
[pO, cO, stO] = search_student_grid(M, s, t, 'astar', 'Conn', 8, 'Heuristic', 'octile');
[pM, cM, stM] = search_student_grid(M, s, t, 'astar', 'Conn', 8, 'Heuristic', 'manhattan');

figure('Name', 'Part C: Octile vs Manhattan heuristic', 'Position', [300 300 1000 450]);
subplot(1, 2, 1)
plot_grid(M, 'Expanded', stO.order, 'Path', pO, 'Start', s, 'Goal', t, ...
    'Title', sprintf('Octile heuristic (A*)\nCost %.3f, Expansions: %d', cO, stO.expansions));
subplot(1, 2, 2)
plot_grid(M, 'Expanded', stM.order, 'Path', pM, 'Start', s, 'Goal', t, ...
    'Title', sprintf('Manhattan heuristic (A*)\nCost %.3f, Expansions: %d', cM, stM.expansions));