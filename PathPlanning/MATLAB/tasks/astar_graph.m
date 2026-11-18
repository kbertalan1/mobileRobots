function [path, cost, stats] = astar_graph(W, s, t, h)
%ASTAR_GRAPH  TASK 5 -- A*.
%
%   Copy your finished dijkstra_graph in here and change ONLY THE KEY.
%
%       Dijkstra   key = g(v)
%       A*         key = g(v) + h(v)
%
%   Careful: the key appears at TWO places -- once when the start is
%   pushed, once inside the loop. Both have to change.
%
%   Target:  A - B - E - F, cost 3, 4 expansions.
%            Same path as Dijkstra, one expansion less.
%
%   Check:   check_my_code('astar')

% error('task:todo', 'Task 5 not started -- delete this line and begin.');
n      = size(W, 1);
g      = inf(n, 1);       % no route known yet
g(s)   = 0;               % the start costs nothing
par    = zeros(n, 1);
closed = false(n, 1);
order  = zeros(1, 0);

OPEN = open_new('minkey');
OPEN = open_push(OPEN, s, g(s) + h(s));

while ~open_isempty(OPEN)
    [OPEN, u] = open_pop(OPEN);
    if closed(u)
        continue
    end
    closed(u)      = true;
    order(end + 1) = u;   %#ok<AGROW>
    trace_step(numel(order), u);
    if u == t
        break
    end

    [nb, w] = graph_neighbors(W, u);
    for k = 1:numel(nb)
        v = nb(k);
        if closed(v)
            continue
        end

        % TODO 1 -- What does the route to v cost if we go through u?
        %           Available: g(u) and w(k).
        candidate = g(u) + W(u, v);

        % TODO 2 -- Is that better than what we already knew about v?
        if candidate < g(v)
            
            % TODO 3 -- If cheaper: update records and push to priority queue
            g(v) = candidate;       % Save the new best cost
            par(v) = u;             % Save the new parent (where we came from)
            OPEN = open_push(OPEN, v, candidate + h(v)); % Push to OPEN with the new priority
        end
    end
end

[path, cost]     = reconstruct_path(par, s, t, W);
stats.expansions = numel(order);
stats.order      = order;
stats.g          = g;
end


%{
G = graph_example();
[p, c, st] = astar_graph(G.W, G.s, G.t, G.h);
plot_graph(G, 'Expanded', st.order, 'Path', p, 'Title', 'My A*');
check_my_code('astar')
%}