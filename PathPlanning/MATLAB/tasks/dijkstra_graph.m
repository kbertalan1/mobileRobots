function [path, cost, stats] = dijkstra_graph(W, s, t)
%DIJKSTRA_GRAPH  TASK 4 -- Dijkstra.
%
%   From here on there is arithmetic. Two things come in:
%
%     g(v)         what the best route to v found so far costs
%     relaxation   is going through u cheaper than what we knew?
%
%   That is why the seen array is gone: a node MAY enter OPEN again when
%   a cheaper route to it turns up. And that is why the line
%   "if closed(u)" now really matters -- the same node can sit in OPEN
%   several times and only the first one out counts.
%
%   Target:  A - B - E - F, cost 3, 5 expansions.
%            D stays in OPEN with g = 6 and is never taken out.
%
%   Check:   check_my_code('dijkstra')

%error('task:todo', 'Task 4 not started -- delete this line and begin.');

n      = size(W, 1);
g      = inf(n, 1);       % no route known yet
g(s)   = 0;               % the start costs nothing
par    = zeros(n, 1);
closed = false(n, 1);
order  = zeros(1, 0);

OPEN = open_new('minkey');
OPEN = open_push(OPEN, s, g(s));

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
            OPEN = open_push(OPEN, v, candidate); % Push to OPEN with the new priority
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
[p, c, st] = dijkstra_graph(G.W, G.s, G.t);
plot_graph(G, 'Expanded', st.order, 'Path', p, 'Values', st.g, 'Label', 'g', 'Title', 'My Dijkstra');
check_my_code('dijkstra')
%}

%{
  DIJKSTRA'S ALGORITHM - CONCEPTUAL OVERVIEW

  Dijkstra finds the absolute cheapest path from start to finish by 
  factoring in the actual weight (cost, distance, or energy) of each 
  step, rather than just counting the number of edges like BFS does.

  The algorithm relies on three main tools:
  1. Priority Queue (Min-Key): Always explores the node with the 
     lowest total accumulated cost (g-value) next.
  2. Relaxation: The core mechanism. If the algorithm discovers a new, 
     cheaper route to a node it already knows about, it "relaxes" (updates) 
     the node's stored cost and parent to reflect this better path.
  3. Parent Array & Cost Tracker (g): Leaves "breadcrumbs" for the 
     best route and keeps a ledger of the lowest cost to reach every node.

  Because it evaluates true accumulated costs, Dijkstra is essential for 
  systems where movement has variable penalties—such as routing traces on 
  a PCB where certain paths have higher impedance, or calculating the most 
  energy-efficient trajectory for an autonomous vehicle across uneven terrain.
%}