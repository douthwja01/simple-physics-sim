
clear all;
close all;
addpath("Engine");

sim = Simulator();

numberOfObjects = 12;
numberPerColumn = 3;
gridPoints = CreateGrid([-1.5;0;4],numberOfObjects,numberPerColumn,1.1);

for i = 1:numberOfObjects
    % Place the object
    entity_i = EntityCreator.Box(sprintf("Object %d (Box)",i),gridPoints(:,i),Quaternion.Zero);
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
% ground.Collider.Width = 10;
% ground.Collider.Depth = 10;
% Visuals
ground.Renderer.Colour = "g";
% ground.Renderer.Width = 10;
% ground.Renderer.Depth = 10;
% ground.Visuals 
%ground.Renderer.Mesh = MeshExtensions.Plane(zeros(3,1),[0;0;1],1,1);
ground.Renderer.Colour = "g";
sim.Add(ground);

% Simulate
sim.Physics.SubSteps = 10;
sim.Physics.Integrator = EulerIntegrator();
sim.Physics.BroadPhaseSolver = OctreeSolver();
sim.Simulate(inf);
