% Define the parameters for the desired sphere
grid_size_lower = -1;
grid_size_upper = 1;
zgrid_lower = 0;
zgrid_upper = 2;
Num_obj = 10;

rad = 0.2;
coord = grid_size_lower:0.01:grid_size_upper;
coordZ = 0:0.01:2*grid_size_upper;
x_grid = grid_size_lower:0.25:grid_size_upper;
y_grid = grid_size_lower:0.25:grid_size_upper;
z_grid = 0:0.25:2*grid_size_upper;


radius_sphere = zeros(1,Num_obj);
X_list = zeros(1,Num_obj);
Y_list = zeros(1,Num_obj);
Z_list = zeros(1,Num_obj);
[X_plot, Y_plot, Z_plot] = meshgrid(x_grid, y_grid, z_grid);
%%%%%Disallow for object creation within the limits of the robot arm
obj_XYZ = zeros(3,length(x_grid)*length(y_grid)*length(z_grid));
num_pts = 0;
for k = 1:length(x_grid)
    for l = 1:length(x_grid)
        for m = 1:length(x_grid)
            num_pts = num_pts+1;
            obj_XYZ(:,num_pts) = [X_plot(m,l,k), Y_plot(m,l,k), Z_plot(m,l,k)]';
            
        end
    end
end
object_init_pos = {};
start_position = [0 0 2];
rad_link = 0.01;
[P_coord] = RobotLinks(start_position, rad_link);
for p = 1:length(obj_XYZ)
    add_pt = true;
    while add_pt == true
        for m = 1:size(P_coord,2)
            for n = 1:length(P_coord{m})
                robot2object = norm(P_coord{m}(:,n)-obj_XYZ(:,p));
                if robot2object <= (rad+rad_link) || robot2object == 0
                    add_pt = false;
                    break
                end
            end
            if robot2object <= (rad+rad_link) || robot2object == 0
                add_pt = false;
                break
            end
        end
        if add_pt == true
            object_init_pos = [object_init_pos, obj_XYZ(:,p)];
            break
        else
            add_pt = false;
        end
    end
end

figure;
xlim([grid_size_lower, grid_size_upper]);
ylim([grid_size_lower, grid_size_upper]);
zlim([0, 2*grid_size_upper]);
grid on;
hold on; % Hold on to the figure so all spheres are plotted in the same axes
% Use '.' marker style for small dots, size 1, gray color

object_pos = zeros(3,Num_obj);
for i = 1:Num_obj
    % Generate the coordinates for a unit sphere
    [X_unit, Y_unit, Z_unit] = sphere(20); % Added resolution for smoother spheres

    radius = rad;

    object = object_init_pos{randi([1, length(object_init_pos)])};
    %object_pos(:,i) = [center_x, center_y, center_z]';
    object_pos(:,i) = [object(1), object(2), object(3)]';

    radius_sphere(i) = radius;
    X_list(i) = object(1);
    Y_list(i) = object(2);
    Z_list(i) = object(3);
    % Scale and translate the coordinates
    X = X_unit * radius + object(1);
    Y = Y_unit * radius + object(2);
    Z = Z_unit * radius + object(3);

    % Plot the current sphere object inside the loop
    h = surf(X, Y, Z);
    set(h,'EdgeColor', 'black', 'FaceColor', [1 0 0])
    hold on
    axis equal
end 

%Remove points inside spherical obstructions

% for k = 1:length(x_grid)
%     for l = 1:length(x_grid)
%         for m = 1:length(x_grid)
%             for i = 1:length(X_list)
%                 radius_pt_remove = sqrt((X_plot(m,l,k)-X_list(i))^2+(Y_plot(m,l,k)-Y_list(i))^2+(Z_plot(m,l,k)-Z_list(i))^2);
%                 if radius_pt_remove <= rad  
%                     X_plot(m,l,k) = 0;
%                     Y_plot(m,l,k) = 0;
%                     Z_plot(m,l,k) = 0;
%                 end
%             end
%         end
%     end
% end
% 
i = true;
while i == true
    goalnode = object_init_pos{randi([1, length(object_init_pos)])};
    for j = 1:length(object_pos)
        if goalnode == object_pos(:,j)
            continue
        else
            break
        end
    end
    if goalnode == object_pos(:,j)
        continue
    else
        break
    end
end

robot_pos = start_position;
max_iter = 3000;

%optimal_pose = {};
step_size = 0.15;
rad_around_node = 0.4;
[Robot_link_create, Link_check] = CollisionDetectLinks(object_pos, rad, P_coord);

start_node = RRTNode(robot_pos, [], 0, P_coord);
tree = {start_node};
iterations = 0;
for i = 1:max_iter
    %rand_coord = [x_grid(randi([1, length(x_grid)])),y_grid(randi([1, length(y_grid)])),z_grid(randi([1, length(z_grid)]))];
    rX = rand(); % For x
    rY = rand(); % For y
    rZ = rand(); % For z
    rand_coord = [(grid_size_lower + (grid_size_upper - grid_size_lower) * rX),(grid_size_lower + (grid_size_upper - grid_size_lower) * rY),(zgrid_lower + (zgrid_upper - zgrid_lower) * rZ)];

    [x_nearest, nearest_idx] = Nearest(tree, rand_coord);
    newCoords = Steer(x_nearest.coordinates, rand_coord, step_size);

    [P_coord, rad_link] = RobotLinks(newCoords, rad_link);
    [Robot_link_create, Link_check] = CollisionDetectLinks(object_pos, rad, P_coord);
    

    for count = 1:3
        if Link_check == true
            break
        else
            [P_coord, rad_link] = RobotLinks(newCoords, rad_link);
            [Robot_link_create, Link_check] = CollisionDetectLinks(object_pos, rad, P_coord);
        end
    end
    [Valid] = is_collision_free(x_nearest.coordinates, newCoords, object_pos, rad, rad_link);
   
    if Valid == true && Link_check == true
       
        % Create a temporary new node object
        tempNode = RRTNode(newCoords, [], 0, P_coord); 

        % 5. Find all nodes in the neighborhood of the new point
        X_near_indices = find_near_nodes(tree, newCoords, rad_around_node);

        % 6. Select the best parent and add to the tree
        [parent_node, min_cost] = select_best_parent(tree, tempNode, X_near_indices, object_pos, rad, rad_link);
        
        if ~isempty(parent_node)
            tempNode.parent = parent_node;
            tempNode.cost = min_cost;
            tree{end+1} = tempNode; % Add the new Node object to the tree

            % 7. Rewire the tree (optimize paths through the new node)
            rewire_tree(tree, tempNode, X_near_indices, object_pos, rad, rad_link);
        end
        % Check if goal is reached, etc.
        %if norm(P_coord{length(P_coord)}(:,end)-goalnode) <= 0.2
        if norm(newCoords'-goalnode) <= 0.25
            break
        end
    end
    iterations = iterations + 1;
    fprintf('The current iteration number is %d\\n', i);
end        
    



% 3. Plotting
% axis equal
%scatter3(nodes(1,:), nodes(2,:), nodes(3,:), 1, [0 0 0], 'filled');


% xlabel('X');
% ylabel('Y');
% zlabel('Z');

% This is the key command to ensure non-distorted objects:
% axis equal











function [P_coord, rad_link] = RobotLinks(robot_pos, rad_link)
    [T,dhparams] = Sample(robot_pos);

    P_coord = {};
    Link_len = zeros(3,1);
    origin = [0 0 0];
    for i = 1:(size(dhparams, 1)-1)
        if i == 1
            origin_t = origin';
            A = T(:,i);
            B = T(:,i+1);
            Link1 = sqrt((A(1)-origin_t(1))^2+(A(2)-origin_t(2))^2+(A(3)-origin_t(3))^2);
            Link_len(i) = sqrt((B(1)-A(1))^2+(B(2)-A(2))^2+(B(3)-A(3))^2);
            Link1_t = linspace(0, 1, floor(Link1/0.03)); 
            t = linspace(0, 1, floor(Link_len(i)/0.03));
            P_origin = origin_t + Link1_t .* (A - origin_t);
            P = A + t .* (B - A);
            P_coord = [P_coord, P_origin];
            P_coord = [P_coord, P];

        else
            A = T(:,i);
            B = T(:,i+1);
            Link_len(i) = sqrt((B(1)-A(1))^2+(B(2)-A(2))^2+(B(3)-A(3))^2);
            t = linspace(0, 1, floor(Link_len(i)/0.03)); 
            P = A + t .* (B - A);
            P_coord = [P_coord, P];
        end
    end
end


function [robot_link_pos, Link_check] = CollisionDetectLinks(object_pos, obj_rad, P_coord)

rad_link = 0.01; 

robot_link_pos = []; 

% --- Step 1: Process and collect all robot link points into a single matrix ---
for i = 1:length(P_coord)
    
    robot_link_pos = horzcat(robot_link_pos, P_coord{i}); 
    
end

Link_check = true; 

% Iterate through every robot link point
for i = 1:size(robot_link_pos, 2)
    % Current robot point is a column vector
    current_robot_point = robot_link_pos(:, i); 
    
    % Iterate through every object position
    for j = 1:size(object_pos, 2)
        % Current object position is a column vector
        current_object_pos = object_pos(:, j); 
        
        distance = norm(current_robot_point - current_object_pos);
        
        if distance < (rad_link + obj_rad)
            % Collision detected
            Link_check = false;
            % Immediately return once a collision is found to save computation
            return; 
        end
    end
end


end



function [T,dhparams] = Sample(robot_pos)
    
    dhparams = [
    0,    pi/2,  0.75,  0; % Link 1 (base to joint 2)
    0.3,     0,    0,  0; % Link 2 (joint 2 to joint 3)
    0,    pi/2,  0.1,  0; % Link 3 (joint 3 to joint 4)
    0.1,     0,    0,  0; % Link 4 (joint 4 to joint 5)
    0,   -pi/2, -0.75,  0; % Link 5 (joint 5 to joint 6)
    0.75,     0,    0,  0;
    ];
    robot = rigidBodyTree;
    currentBody = robot.Base;
    BODIES = cell(size(dhparams, 1),1);

    for i = 1:size(dhparams, 1)
        bodyName = ['body', num2str(i)];
        jointName = ['joint', num2str(i)];
        BODIES{i} = ['body', num2str(i)];
        body = rigidBody(bodyName);
        joint = rigidBodyJoint(jointName, 'revolute');
        setFixedTransform(joint, dhparams(i, :), 'dh');
        body.Joint = joint;
        addBody(robot, body, currentBody.Name);
        currentBody = body;
    end

    eeBody = rigidBody('end_effector');
    eeJoint = rigidBodyJoint('fixef', 'fixed'); 
    offsetTform = trvec2tform([0, 0, 0]); 
    setFixedTransform(eeJoint, offsetTform); 
    eeBody.Joint = eeJoint;
    addBody(robot, eeBody, currentBody.Name);

    % Inverse Kinematics Implementation

    ik = inverseKinematics('RigidBodyTree', robot);
    q0 = randomConfiguration(robot);
    %%%%%q0 = randomConfiguration(robot);%%%%%% 

    % Define target pose as a homogeneous transformation matrix
    % Target position [X, Y, Z]
    robot_position = robot_pos;
    StartPose = trvec2tform(robot_position); 

    % Note: Use the string name 'end_effector'
    % Define weights (6 elements: Orientation XYZ, Position XYZ)
    weights = [10 10 10 1000 1000 1000]; 

    ee_bounds = constraintCartesianBounds('end_effector'); 
    % Set the bounds: [minX, maxX; minY, maxY; minZ, maxZ]
    ee_bounds.Bounds = [-inf, inf; 
                        -inf, inf; 
                           0, inf]; 

    % Solve the IK problem
    [config, solInfo] = ik('end_effector', StartPose, weights, q0);


    % Display results and visualize the solution
    disp('Calculated Joint Configuration:');
    disp(config);
    disp('Solver Status:'); 
    disp(solInfo.Status);

    % figure;
    % show(robot, config);
    % hold on
    % title('Robot at Target Inverse Kinematics Pose');

    T = zeros(3,size(dhparams, 1));

    for i = 1:size(dhparams, 1)
        Transform = [getTransform(robot, config, BODIES{i})];
        T(:,i) = Transform(1:3,4);
    end


end


function [nearest_node, nearest_idx] = Nearest(tree, point)
    minDist = inf;
    nearest_idx = 0;
    nearest_node = [];
    
    for i = 1:length(tree)
        node = tree{i}; % Access the Node object
        dist = norm(node.coordinates - point); % Use the coordinates property
        if dist < minDist
            minDist = dist;
            nearest_idx = i;
            nearest_node = node;
        end
    end
end

function newCoords = Steer(nearCoords, randCoords, stepSize)
    % newCoords = steer(nearCoords, randCoords, stepSize)
    %
    % Calculates a new set of coordinates (newCoords) by moving from 
    % nearCoords towards randCoords, but limited by the stepSize.

    % Calculate the vector from the nearest node to the random point
    directionVector = randCoords - nearCoords;
    
    % Calculate the total distance to the random point
    distance = norm(directionVector);
    
    % Determine the length of the step to take.
    % If the random point is closer than the step size, move all the way there.
    % Otherwise, move exactly 'stepSize' distance.
    if distance < stepSize
        step = distance;
    else
        step = stepSize;
    end
    
    % Normalize the direction vector (turn it into a unit vector)
    unitDirection = directionVector / distance;
    
    % Calculate the new coordinates by stepping from the near coordinates
    newCoords = nearCoords + unitDirection * step;
end

function near_indices = find_near_nodes(tree, newCoords, radius)
    % near_indices = find_near_nodes(tree, newCoords, radius)
    % 
    % Finds the indices of all nodes in the 'tree' (cell array of Node objects)
    % that are within the specified 'radius' of the 'newCoords'.
    
    near_indices = [];
    
    for i = 1:length(tree)
        node = tree{i}; % Access the Node object
        
        % Calculate the Euclidean distance between the existing node and the new point
        distance = norm(node.coordinates - newCoords);
        
        % If the node is within the search radius, add its index to the list
        if distance <= radius
            near_indices = [near_indices, i]; % Append the index
        end
    end
end


function isValid = is_collision_free(startCoords, endCoords, object_pos, rad, rad_link)
    % Checks if the line segment between startCoords and endCoords is collision-free
    
    isValid = true; % Assume valid until proven otherwise
    
    % Define the number of steps to check along the segment (granularity)
    % A higher number increases accuracy but slows down the algorithm
    numSteps = 40; 
    
    for i = 0:numSteps
        % Interpolate points along the line segment
        t = i / numSteps;
        pointToCheck = startCoords + t * (endCoords - startCoords);
        
        % Check if this point is inside any of the obstacles
        for j = 1:length(object_pos)
            % obsX = object_pos(1, j);
            % obsY = object_pos(2, j);
            % obsZ = object_pos(3, j);

            % Calculate distance from the current point to the obstacle center
            %distance = norm(pointToCheck - [obsX, obsY, obsZ]);
            distance = norm(pointToCheck - object_pos(:,j)');
            
            if distance <= (rad_link+rad)
                % Collision detected
                isValid = false;
                return; % Exit early if a collision is found
            end
        end
    end
end

function [best_parent, min_cost] = select_best_parent(tree, newNode, near_indices, object_pos, rad, rad_link)
    min_cost = inf;
    best_parent = [];

    for i = near_indices
        potential_parent = tree{i};
        % Calculate cost to go from potential parent to newNode
        cost_to_parent = norm(potential_parent.coordinates - newNode.coordinates);
        current_cost = potential_parent.cost + cost_to_parent;
        
        % Check collision and if this path is cheaper
        if current_cost < min_cost && is_collision_free(potential_parent.coordinates, newNode.coordinates, object_pos, rad, rad_link)
            min_cost = current_cost;
            best_parent = potential_parent;
        end
    end
end


function rewire_tree(tree, newNode, near_indices, object_pos, rad, rad_link)
    for i = near_indices
        nearby_node = tree{i}; 
        % Calculate cost if passing through the new node to reach nearby_node
        cost_through_new = newNode.cost + norm(newNode.coordinates - nearby_node.coordinates);
        
        % If the path through newNode is better than the current path:
        if cost_through_new < nearby_node.cost && is_collision_free(newNode.coordinates, nearby_node.coordinates, object_pos, rad, rad_link)
            nearby_node.parent = newNode;             % Update the parent handle
            nearby_node.cost = cost_through_new; % Update the cost
            % Note: MATLAB automatically updates the object in the 'tree' cell array
        end
    end
end