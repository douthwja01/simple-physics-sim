
clear all;
close all;
addpath("Engine");

sim = Simulator();

numberOfObjects = 12;
numberPerColumn = 3;
gridPoints = CreateGrid([-1.5;0;4],numberOfObjects,numberPerColumn,1.1);

for i = 1:numberOfObjects
    % Place the object
    entity_i = EntityCreator.Box("Box",gridPoints(:,i),[1;0;0;0]);
    % Add elements
    entity_i.RigidBody = RigidBody();
    % Renderer
    entity_i.Renderer.Alpha = 0.2;
    if mod(i,2) == 0
        entity_i.Renderer.Colour = "b";
    else
        entity_i.Renderer.Colour = "c";
    end
    % Assign the object
    sim.AddEntity(entity_i);
end

% Add an obstacle
fixed = EntityCreator.Sphere("Obstacle",[0;0;2]);
% Do not move
fixed.Transformation.IsStatic = true;
% Add elements
fixed.RigidBody = RigidBody();
fixed.Renderer.Colour = "r";
% Add the fix object
sim.AddEntity(fixed);

% Add the ground plane
ground = EntityCreator.Plane("Ground",zeros(3,1),[1;0;0;0]);
% Do not move
ground.Transformation.IsStatic = true;
% Collisions
ground.Collider.Width = 10;
ground.Collider.Depth = 10;
% Visuals
ground.Renderer.Colour = "g";
ground.Renderer.Width = 10;
ground.Renderer.Depth = 10;

sim.AddEntity(ground);

% Simulate
sim.Physics.SubSteps = 10;
sim.Physics.Integrator = EulerIntegrator();
sim.Simulate(inf);
