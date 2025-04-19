
%% File: pathcr.m
% Author: Abhishek Tyagi
% Version: 2.3
% Maze-based trajectory planner using Dijkstra's algorithm for Veg drone

function [x_path, y_path, time_vec] = pathcr(map_img, start_pt, goal_pt)
    % Inputs:
    %   map_img - 2D binary image (0=free, 1=obstacle)
    %   start_pt - [row, col]
    %   goal_pt - [row, col]
    % Outputs:
    %   x_path, y_path - time series trajectory (waypoints)
    %   time_vec - timestamps for each point

    map = logical(map_img);
    [nrows, ncols] = size(map);
    cost = inf(nrows, ncols);
    visited = false(nrows, ncols);
    parent = zeros(nrows, ncols, 2);

    cost(start_pt(1), start_pt(2)) = 0;

    while true
        [~, idx] = min(cost(:));
        [r, c] = ind2sub(size(cost), idx);
        if visited(r, c) || isinf(cost(r, c)), break; end
        visited(r, c) = true;
        if isequal([r, c], goal_pt), break; end

        for dr = -1:1
            for dc = -1:1
                if dr == 0 && dc == 0, continue; end
                nr = r + dr;
                nc = c + dc;
                if nr < 1 || nr > nrows || nc < 1 || nc > ncols || map(nr, nc)
                    continue;
                end
                new_cost = cost(r, c) + 1;
                if new_cost < cost(nr, nc)
                    cost(nr, nc) = new_cost;
                    parent(nr, nc, :) = [r, c];
                end
            end
        end
    end

    path = goal_pt;
    while ~isequal(path(1,:), start_pt)
        prev = squeeze(parent(path(1,1), path(1,2), :))';
        if all(prev == 0), break; end
        path = [prev; path];
    end

    % Convert to X-Y (col-row) and time
    y_path = path(:,1); % row => y
    x_path = path(:,2); % col => x
    time_vec = linspace(0, length(x_path)*0.1, length(x_path)); % 0.1s per step

    % Optional: plot
    figure;
    imshow(~map); hold on;
    plot(x_path, y_path, 'r-', 'LineWidth', 2);
    title('Planned Dijkstra Path');
end
