
%% File: pathcr.m
% Author: Abhishek Tyagi
% Version: 2.4
% Maze-based trajectory planner using Dijkstra's algorithm for Veg drone
%
% Fixes vs 2.3:
%   * the node picked each iteration now ignores already-visited cells
%     (2.3 re-picked the start cell forever and exited after one expansion,
%      so it never produced a path)
%   * diagonal moves cost sqrt(2) and may not cut obstacle corners
%   * unreachable goal / blocked start or goal raise an error instead of
%     silently returning a 1-point "path"
%   * time_vec is exactly dt apart; plotting is optional

function [x_path, y_path, time_vec] = pathcr(map_img, start_pt, goal_pt, dt, do_plot)
    % Inputs:
    %   map_img  - 2D binary image (0=free, 1=obstacle)
    %   start_pt - [row, col]
    %   goal_pt  - [row, col]
    %   dt       - seconds per waypoint (default 0.1)
    %   do_plot  - true to draw the path (default true)
    % Outputs:
    %   x_path, y_path - waypoints (x = column, y = row)
    %   time_vec       - timestamps for each point
    if nargin < 4 || isempty(dt), dt = 0.1; end
    if nargin < 5, do_plot = true; end

    map = logical(map_img);
    [nrows, ncols] = size(map);
    start_pt = double(start_pt(:)'); goal_pt = double(goal_pt(:)');
    in_map = @(p) p(1) >= 1 && p(1) <= nrows && p(2) >= 1 && p(2) <= ncols;
    if ~in_map(start_pt) || ~in_map(goal_pt), error('pathcr:bounds', 'start/goal outside map'); end
    if map(start_pt(1), start_pt(2)), error('pathcr:blocked', 'start is inside an obstacle'); end
    if map(goal_pt(1), goal_pt(2)),   error('pathcr:blocked', 'goal is inside an obstacle'); end

    cost = inf(nrows, ncols);
    visited = false(nrows, ncols);
    parent = zeros(nrows, ncols, 2);
    cost(start_pt(1), start_pt(2)) = 0;

    while true
        open_cost = cost;
        open_cost(visited) = inf;              % only consider unvisited cells
        [c_min, idx] = min(open_cost(:));
        if isinf(c_min), break; end            % nothing reachable left
        [r, c] = ind2sub([nrows, ncols], idx);
        visited(r, c) = true;
        if r == goal_pt(1) && c == goal_pt(2), break; end

        for dr = -1:1
            for dc = -1:1
                if dr == 0 && dc == 0, continue; end
                nr = r + dr; nc = c + dc;
                if nr < 1 || nr > nrows || nc < 1 || nc > ncols, continue; end
                if map(nr, nc) || visited(nr, nc), continue; end
                if dr ~= 0 && dc ~= 0 && (map(r, nc) || map(nr, c))
                    continue;                  % no squeezing diagonally past an obstacle corner
                end
                new_cost = cost(r, c) + hypot(dr, dc);
                if new_cost < cost(nr, nc)
                    cost(nr, nc) = new_cost;
                    parent(nr, nc, :) = [r, c];
                end
            end
        end
    end

    if ~visited(goal_pt(1), goal_pt(2))
        error('pathcr:unreachable', 'goal is not reachable from start');
    end

    path = goal_pt;
    while ~isequal(path(1, :), start_pt)
        prev = reshape(parent(path(1, 1), path(1, 2), :), 1, 2);
        path = [prev; path]; %#ok<AGROW>
    end

    % Convert to X-Y (col-row) and time
    y_path = path(:, 1);   % row => y
    x_path = path(:, 2);   % col => x
    time_vec = (0:numel(x_path) - 1)' * dt;

    if do_plot
        figure;
        imshow(~map); hold on;
        plot(x_path, y_path, 'r-', 'LineWidth', 2);
        plot(start_pt(2), start_pt(1), 'go', goal_pt(2), goal_pt(1), 'bx', 'LineWidth', 2);
        title(sprintf('Planned Dijkstra Path (length %.1f cells)', cost(goal_pt(1), goal_pt(2))));
    end
end
