classdef RRTNode < handle
    %RRTNode Represents a node in the RRT* search tree.

    properties
        coordinates % Node position/state (e.g., [x, y] or [x, y, theta])
        parent      % Handle to the parent RRTNode object
        cost        % Cost to reach this node from the start
        pose
    end

    methods
        function obj = RRTNode(coord, parentNode, nodeCost, PoseatNode)
            %RRTNode Constructor for the RRTNode class.
            if nargin > 0
                obj.coordinates = coord;
                obj.parent = parentNode;
                obj.cost = nodeCost;
                obj.pose = PoseatNode;
            end
        end

        function dist = distanceFrom(obj, otherNode)
            % Calculates the Euclidean distance from this node to another.
            % You can customize this for different state spaces (e.g., Dubins, Reeds-Shepp)
            dist = norm(obj.coordinates - otherNode.coordinates);
        end
    end
end