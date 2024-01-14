
clear all;
close all;
addpath("Engine");

sim = Simulator();

numberOfObjects = 12;
numberPerColumn = 3;
gridPoints = CreateGrid([-1.5;0;4],numberOfObjects,numberPerColumn,1.1);

for i = 1:numberOfObjects
    % Place the object
    entity_i = Entity(sprintf("Object %d (Box)",i));
    entity_i.Transform.Position = gridPoints(:,i);
    % Add elements
    entity_i.RigidBody = RigidBody();
    entity_i.Collider = BoxCollider();
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
fixed = Entity("Obstacle");
fixed.Transform.Position = [0;0;2];
fixed.Transform.IsStatic = true;
% Add elements
fixed.RigidBody = RigidBody();
fixed.Collider = SphereCollider();
% renderer = fixed.Visuals;
fixed.Renderer.Mesh = MeshExtensions.UnitSphere();
fixed.Renderer.Colour = "r";
% Add the fix object
sim.AddEntity(fixed);

% Add the ground plane
ground = Entity("Ground");
ground.Transform.IsStatic = true;
ground.Transform.Scale = [20;20;1];
% Add elements
ground.Collider = PlaneCollider();
% ground.Visuals = 
ground.Renderer.Mesh = MeshExtensions.Plane(zeros(3,1),[0;0;1],1,1);
ground.Renderer.Colour = "g";
sim.AddEntity(ground);

% Simulate
sim.Physics.SubSteps = 10;
sim.Physics.Integrator = EulerIntegrator();
sim.Simulate(inf);
