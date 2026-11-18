function [path, cost, stats] = bfs_graph(W, s, t)
%BFS_GRAPH  TASK 2 -- breadth-first search.
%
%   Copy your finished dfs_graph in here, rename the function in the
%   first line, and change EXACTLY ONE WORD.
%
%   If you change more than one word you have misunderstood the task --
%   and that one word is the central claim of the whole lecture.
%
%   Target:  A - D - F, 2 steps, cost 7, 6 expansions.
%
%   Check:   check_my_code('bfs')
%
%   Watch the result: breadth-first search finds the path with the
%   FEWEST STEPS. That does not make it the cheapest -- here it costs 7
%   while a route for 3 exists.


%DFS_GRAPH  TASK 1 -- depth-first search.
%
%   The skeleton below is complete and runs. Three places are missing,
%   all marked TODO. Do them in order and RUN IT after each one.
%
%   How to run it:
%       G = graph_example();
%       [p, c, st] = dfs_graph(G.W, G.s, G.t)
%
%   Target:  A - D - F, cost 7, 3 expansions.
%
%   Check:   check_my_code('dfs')
%
%   See also OPEN_NEW, GRAPH_NEIGHBORS, RECONSTRUCT_PATH.


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


% TODO 2 -- Which container belongs to depth-first search?
%           'lifo' = last in, first out
%           'fifo' = first in, first out
%           (Do TODO 1 first. The wrong one is deliberately here.)
OPEN = open_new('fifo');

seen(s) = true;
OPEN    = open_push(OPEN, s);

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
        OPEN   = open_push(OPEN, v);
        seen(v) = true;
    end
end

[path, cost]     = reconstruct_path(par, s, t, W);
stats.expansions = numel(order);
stats.order      = order;
end



%{
  BTH-FIRST SEARCH (BFS) - CONCEPTUAL OVERVIEW
BFS explores a graph evenly in all directions, expanding layer by 
  layer. Like a ripple spreading outward from a pebble dropped in a 
  pond, it checks all immediate neighbors first, then moves on to 
  their neighbors, and so on.

  The algorithm relies on three main tools:
  1. FIFO Queue (First-In, First-Out): Prioritizes the oldest 
     discovered paths. By evaluating nodes in the exact order they 
     were found, it forces the search to widen before it deepens.
  2. Visited List: Marks explored nodes to prevent infinite loops.
  3. Parent Array: Leaves "breadcrumbs" to retrace the final route.

  Because it expands in uniform waves, BFS is mathematically 
  guaranteed to find the path with the fewest possible steps (edges). 
  However, it ignores edge weights—meaning it does not account for 
  travel time or energy costs—making it best suited for unweighted 
  grids or minimizing hop-counts in a communication network.
 
%}