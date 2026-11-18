[M, s, t] = maze_obstacles(30, 40, 0.25, 7);
methods = {'dfs', 'bfs', 'greedy', 'dijkstra', 'astar'};

figure('Position', [100 100 1500 430]);
for k = 1:numel(methods)
    method = methods{k};
    [p, c, st] = search_student_grid(M, s, t, method, 'Conn', 4);

    subplot(1, numel(methods), k)
    plot_grid(M, 'Expanded', st.order, 'Path', p, ...
        'Start', s, 'Goal', t, ...
        'Title', sprintf('%s: cost %.0f, %d expanded', ...
        method, c, st.expansions));
    fprintf('%-9s cost %8.3f, expansions %5d, search %8.3f ms\n', ...
        method, c, st.expansions, 1000*st.searchTime);
end