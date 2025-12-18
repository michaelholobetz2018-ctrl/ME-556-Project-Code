

% figure();
% rotate3d on;
% hold on;
% axis equal;
% grid on;
% view(3); % Set default 3D view

% Initialize path storage
path_coords = [];
path_pose = [];
node_plot = tree{end}; 
radius_link = 0.01;

% for m = 1:length(node_plot.pose)
%     for n = 1:length(node_plot.pose{m})
%         scatter3(node_plot.pose{m}(1,n), node_plot.pose{m}(2,n), node_plot.pose{m}(3,n), 5, [0 0 0], 'filled');
%     end
% end
% for m = 1:length(node_plot.pose)
%     % Extract the coordinate matrix for the current pose
%     current_pose = node_plot.pose{m};
% 
%     % plot3(X, Y, Z) connects coordinates in the order they appear in the vectors
%     % current_pose(1,:) = X, current_pose(2,:) = Y, current_pose(3,:) = Z
%     plot3(current_pose(1,:), current_pose(2,:), current_pose(3,:), 'r-', 'LineWidth', 1);
% end

while ~isempty(node_plot)
    % Append current node coordinates (ensure it is a row vector)
    path_coords = [path_coords; node_plot.coordinates(:)'];
    path_pose = [path_pose; node_plot.pose];
    % Move up to the parent
    %if isfield(node_plot, 'parent') && ~isempty(node_plot.parent)
    if ~isempty(node_plot.parent)
        node_plot = node_plot.parent;
    else
        node_plot = []; % Terminate loop
    end
end
[X_unit, Y_unit, Z_unit] = sphere; 
r = 0.2;
%%%%%%%
% for t = 1:length(path_pose)
%     figure();
%     rotate3d on;
%     hold on;
%     axis equal;
%     grid on;
%     view(3);
%     % 1. Plot the path as a continuous line
%     plot3(path_coords(:,1), path_coords(:,2), path_coords(:,3), 'b-', 'LineWidth', 2);
% 
%     % 2. Plot all ancestor nodes in red
%     scatter3(path_coords(2:end,1), path_coords(2:end,2), path_coords(2:end,3), 20, [1 0 0], 'filled');
% 
%     % 3. Plot the leaf (start) node in black
%     scatter3(path_coords(1,1), path_coords(1,2), path_coords(1,3), 30, [0 0 0], 'filled');
% 
%     % 4. Plot the root (end) node in green
%     scatter3(path_coords(end,1), path_coords(end,2), path_coords(end,3), 30, [0 1 0], 'filled');
% 
%     %plot objects
%     for i = 1:length(object_pos)
%         X = X_unit * r + object_pos(1,i);
%         Y = Y_unit * r + object_pos(2,i);
%         Z = Z_unit * r + object_pos(3,i);
%         h = surf(X, Y, Z);
%         set(h, 'FaceColor', 'red', 'EdgeColor', 'black');
%         hold on
%     end
%     pose = path_pose(t);
%     for h = 1:length(pose)
%         for k = 1:length(pose{h})
%             scatter3(pose{h}(1,k), pose{h}(2,k), pose{h}(3,k), 5, [0 0 0], 'filled');
%         end
%     end
%     for m = 1:length(pose)
%         % Extract the coordinate matrix for the current pose
%         current_pose = pose{m};
% 
%         % plot3(X, Y, Z) connects coordinates in the order they appear in the vectors
%         % current_pose(1,:) = X, current_pose(2,:) = Y, current_pose(3,:) = Z
%         plot3(pose(1,:), pose(2,:), pose(3,:), 'r-', 'LineWidth', 1);
%     end
%     xlabel('X'); ylabel('Y'); zlabel('Z');
%     title('Path from node ',length(path_pose)-t, 'to Root');
% end
%%%%%%%%%

for t = 1:length(path_pose)
    figure(t); % Opens a separate figure for each step (or use clf to animate)
    hold on; grid on; axis equal; view(3);
    rotate3d on;

    % 1. Plot the static path and nodes
    plot3(path_coords(:,1), path_coords(:,2), path_coords(:,3), 'b-', 'LineWidth', 1.5);
    scatter3(path_coords(:,1), path_coords(:,2), path_coords(:,3), 20, 'r', 'filled');

    % 2. Plot obstacles
    for i = 1:size(object_pos, 2)
        X = X_unit * r + object_pos(1,i);
        Y = Y_unit * r + object_pos(2,i);
        Z = Z_unit * r + object_pos(3,i);
        surf(X, Y, Z, 'FaceColor', 'r', 'EdgeColor', 'black', 'FaceAlpha', 0.5);
    end

    % 3. Plot the Robot Configuration (Pose) at step t
    % path_pose is a cell array where each element is a cell array of link matrices
    current_full_pose = path_pose(t,:); 
    
    for m = 1:length(current_full_pose)
        for k = 1:length(current_full_pose{m})
            link_coords = current_full_pose{m}(:,k); % Matrix of [3 x N]
        
            % Plot the joints/points of the link
            scatter3(link_coords(1,1), link_coords(2,1), link_coords(3,1), 10, 'k', 'filled');
        
            % Plot the link segment (The fix is here: link_coords replaces 'pose')
            %plot3(link_coords(1,m), link_coords(2,m), link_coords(3,m), 'k-', 'LineWidth', 2);
        end
    end

    xlabel('X'); ylabel('Y'); zlabel('Z');
    % Fixed title formatting
    title(sprintf('Path Step: %d | Nodes remaining to root: %d', t, length(path_pose)-t));
end

