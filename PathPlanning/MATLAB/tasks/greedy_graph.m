function [path, cost, stats] = greedy_graph(W, s, t, h)
%GREEDY_GRAPH  TASK 3 -- greedy best-first search.
%
%   Same pattern again. This time the order is decided by a number: the
%   estimate h(v) of how far it still is from v to the goal.
%
%   Copy your bfs_graph and change two things:
%     - the container has to sort by a key
%     - the key has to be handed over when pushing:
%           open_push(OPEN, v, <key>)
%
%   Nothing else. No g, no relaxation -- greedy only ever looks ahead.
%
%   h is a vector with one estimate per node, h(t) = 0.
%
%   Target:  A - D - F, cost 7, 3 expansions.
%
%   Check:   check_my_code('greedy')
%
%   Worth thinking about: greedy needs the same number of expansions as
%   depth-first search and finds the same path. Is that a coincidence?

%error('task:todo', 'Task 3 not started -- delete this line and begin.');
n      = size(W, 1);
par    = zeros(n, 1);     % par(v): which node did we come from to reach v
seen   = false(n, 1);     % already put into OPEN once?
closed = false(n, 1);     % neighbours already looked at?
order  = zeros(1, 0);     % which nodes came out, in order

% OPEN - a stack storing node which are discovered but not pushed yet
% seen - a node that has been pushed to the OPEN stack
% closed - a node that has been already processed
% par - stores the parent of each node 
% order - stores the order of the expansion

%           'lifo' = last in, first out
%           'fifo' = first in, first out
%           (Do TODO 1 first. The wrong one is deliberately here.)
OPEN = open_new('minkey');

seen(s) = true;
OPEN = open_push(OPEN, s, h(s));

while ~open_isempty(OPEN)
    [OPEN, u] = open_pop(OPEN);

    % A stale entry? Skip it. (In this pattern the line never fires,
    % because seen already prevents duplicates. It is here so the loop
    % stays unchanged when you move to task 4, where it is essential.)
    if closed(u)
        continue
    end
    closed(u)      = true;
    order(end + 1) = u;   %#ok<AGROW>
    trace_step(numel(order), u);

    % TODO 3 -- When are we done? Replace false by the condition.
    if u == t
        break
    end

    nb = graph_neighbors(W, u);
    for k = 1:numel(nb)
        v = nb(k);
        if seen(v) || closed(v)
            continue
        end

        % TODO 1 -- Two lines are missing here:
        %             - remember that we reached v coming from u
        %             - put v into OPEN  (open_push)
        par(v) = u;
        %OPEN   = open_push(OPEN, v);
        OPEN = open_push(OPEN, v, h(v));
        seen(v) = true;
    end
end

[path, cost]     = reconstruct_path(par, s, t, W);
stats.expansions = numel(order);
stats.order      = order;
end



%{
G = graph_example();
[p, c, st] = greedy_graph(G.W, G.s, G.t, G.h);
plot_graph(G, 'Expanded', st.order, 'Path', p, 'Title', 'My Greedy');
check_my_code('greedy')
%}

%{
  GREEDY BEST-FIRST SEARCH - CONCEPTUAL OVERVIEW

  Greedy Best-First Search tries to reach the target as quickly as 
  possible by always moving toward the node that *looks* closest to 
  the goal, completely ignoring how much it cost to get there.

  The algorithm relies on three main tools:
  1. Priority Queue (Min-Key): Always explores the node with the 
     smallest heuristic value (h-value) next.
  2. Heuristic Function (h): A mathematical estimate (like a straight-line 
     distance) of how far a node is from the target. 
  3. First-Visit Locking: Like BFS and DFS, it does not re-evaluate or 
     "relax" paths. Once it visits a node, it locks its parent in.

  Because it ignores accumulated travel costs (edge weights), Greedy is 
  "blind" to expensive terrain. It is extremely fast and expands very few 
  nodes, but it does not guarantee the cheapest route. It is best used 
  when computing speed is much more important than finding the absolute 
  perfect path.
%}