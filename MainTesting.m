
clear all;
close all;
addpath("Engine");

sim = Simulator();

numberOfObjects = 12;
numberPerColumn = 3;
gridPoints = CreateGrid([-1.5;0;4],numberOfObjects,numberPerColumn,1.3);

for i = 1:numberOfObjects
    % Place the object
    entity_i = EntityCreator.Box( ...
        "Box", ...
        gridPoints(:,i), ...
        Quaternion.FromEulers(0,0,pi/6));
    % Add elements
    entity_i.RigidBody = RigidBody();
    % Renderer
    entity_i.Renderer.Alpha = 0.2;
    if mod(i,2) == 0
        entity_i.Renderer.Colour = "b";
    else
        entity_i.Renderer.Colour = "c";
    end

    entity_i.Transform.SetLocalScale([1;3;1]);

    % Assign the object
    sim.Add(entity_i);
end

% Add an obstacle
fixed = EntityCreator.Sphere( ...
    "Obstacle", ...
    [0;0;2], ...
    Quaternion.Random());
% Do not move
fixed.Transform.IsStatic = true;
% Add elements
fixed.RigidBody = RigidBody();
fixed.Renderer.Colour = "r";
% Add the fix object
sim.Add(fixed);

% Add the ground plane
ground = EntityCreator.Plane("Ground",zeros(3,1));
% Do not move
ground.Transform.IsStatic = true;
ground.Transform.SetLocalScale([15;5;1]);
% Collisions

% Visuals
ground.Renderer.Colour = "g";
% Add the ground object
sim.Add(ground);

%found = sim.Find("Name","Box");

% Configure Simulator 
sim.Physics.SubSteps = 10;
sim.Physics.Integrator = EulerIntegrator();
sim.Physics.BroadPhaseSolver = OctreeSolver();
% Simulate
sim.Simulate(inf);
