classdef SweepAndPrune < BroadPhaseSolver
    % The sweep and prune algorithm parses the set of manifolds where
    % collision is possible using a primitive AABB interval check through 
    % successive dimensions.
    
    properties
        AABBs = AABB.empty;
    end
    methods 
        function [this] = SweepAndPrune()
            % CONSTRUCTOR - Create an instance of the 'Sweep and Prune'
            % broad-phase approach to collision resolution.

            % Call the parent
            [this] = this@BroadPhaseSolver();
        end
        function [manifolds] = ResolveManifolds(this,colliders)
            % This function resolves the set of active intervals/manifold
            % between the provided collider set where a manifold is 
            % defined as:
            % Collision => colliderA,colliderB

            % Sanity check
            assert(isa(colliders,"Collider"),"Expecting a list of 'collider' objects.");
            
            % Property initialisation
            this.AABBs = AABB.empty;
            manifolds = Manifold.empty;
            numberOfColliders = numel(colliders);

            % Move through all colliders and construct/transform their
            % AABBs to be representive for this frame.
            for i = 1:numel(colliders)
                % Transform the AABB  for i
                this.AABBs(i) = colliders(i).GetTransformedAABB();
                % Ensure the result retains the parent association
                this.AABBs(i).Parent = colliders(i);
            end
           
            % 1. We want to move through the set of colliders and compair
            % dimensional intervals
            % 2. We want to move through the set of intervals with the
            % next next dimension check, remove those don't overlap
            % 3. We want to move through the set of intervals with final
            % dimension, remove those that don't overlap.

            %% 1. Sweep (along the x-axis)
            pairs = [];           
            for i = 1:numberOfColliders
                collider_i = colliders(i);
                for j = i+1:numberOfColliders
                    collider_j = colliders(j);

                    % Check if assessing self-collision
                    if collider_i.Cid == collider_j.Cid
                        continue
                    end
                    
                    % Compare x-bounds
                    if ~Interval.TestIntersection(this.AABBs(i).xInterval,this.AABBs(j).xInterval)
                        continue
                    end
                    
                    % Append the new intersecting manifold
                    pairs = vertcat(pairs,[this.AABBs(i),this.AABBs(j)]);
                end
            end

            % Check for pairings
            numberOfPairs = size(pairs,1);
            if numberOfPairs == 0
                return;
            end

            %% 2. Parse y-axis
            logicalIndices = false(numberOfPairs,1);
            for i = 1:numberOfPairs
                % Evaluate the intersection, logically select if true.
                if Interval.TestIntersection(pairs(i,1).yInterval,pairs(i,2).yInterval)
                   logicalIndices(i) = true; 
                end
            end
            % Prune
            pairs = pairs(logicalIndices,:);

            % Check for pairings
            numberOfPairs = size(pairs,1);
            if numberOfPairs == 0
                return;
            end

            %% 3. Parse z-axis
            logicalIndices = false(numberOfPairs,1);
            for i = 1:numberOfPairs
                % Evaluate the intersection, logically select if true.
                if Interval.TestIntersection(pairs(i,1).zInterval,pairs(i,2).zInterval)
                   logicalIndices(i) = true; 
                end
            end
            % Prune
            pairs = pairs(logicalIndices,:);

            % Complete, 
            numberOfPairs = size(pairs,1);
            if numberOfPairs == 0
                return;
            end

            %% 4. Generate a manifold set to be evaluated
            manifolds = Manifold.empty(0,numberOfPairs);
            for i = 1:numberOfPairs
                % Create a collision manifold
                manifolds(i) = Manifold(pairs(i,1).Parent,pairs(i,2).Parent);
            end
        end
        function [handles] = DrawAABBs(this,container,colour)
            % This function allows the drawing of the AABB collision
            % primitives as a series of mesh primitives. This is mainly for
            % debugging.

            % Input handling
            if nargin < 3
                colour = 'g';
            end
            if nargin < 2
                container = gca;
            end
            % Create meshes and draw
            for i = 1:numel(this.AABBs)
                mesh = AABB.CreateMesh(this.AABBs(i));
                handles(i) = mesh.Draw(container,colour);
            end
        end
    end
end