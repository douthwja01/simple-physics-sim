
clear all;
close all;
addpath("Engine");

sim = Simulator();

numberOfObjects = 5;
numberPerColumn = 10;
gridPoints = CreateGrid([0.5;0;4],numberOfObjects,numberPerColumn,1.5);

for i = 1:numberOfObjects
    % Place the object
    entity_i = EntityCreator.Box(sprintf("Object %d (Box)",i),gridPoints(:,i),Quaternion.Zero);
%     entity_i.Transform.Position = entity_i.Transform.Position + 2*rand(3,1);
    
    % Add elements
    entity_i.RigidBody = RigidBody();
    entity_i.Renderer.Alpha = 0.2;
    if mod(i,2) == 0
        entity_i.Renderer.Colour = "b";
    else
        entity_i.Renderer.Colour = "c";
    end
    % Assign the object
    sim.Add(entity_i);
end

% Add an obstacle
fixed = EntityCreator.Sphere("Obstacle");
fixed.Transform.SetWorldPosition([0;0;2]);
fixed.Transform.IsStatic = true;
% Add elements
fixed.RigidBody = RigidBody();
fixed.Renderer.Colour = "r";
% Add the fix object
sim.Add(fixed);

% Add the ground plane
ground = EntityCreator.Plane("Ground");
ground.Transform.SetWorldScale([10;10;1]);
ground.Transform.IsStatic = true;
% Collisions
ground.Renderer.Colour = "g";
sim.Add(ground);

%% Simulator configuration
sim.WorldSize = 15;
sim.Physics.SubSteps = 10;
% Numeric integrators
sim.Physics.Integrator = EulerIntegrator();
% Collision solvers
% sim.Physics.BroadPhaseSolver = OctreeSolver();
sim.Physics.BroadPhaseSolver = SweepAndPrune();
% Simulate
sim.Simulate(inf);
