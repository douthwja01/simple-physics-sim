
clear all;
close all;
addpath("Common");
addpath("Physics");
addpath("Physics/Integrators");
addpath("Physics/Solvers");
addpath("Physics/Graphics");
addpath("Physics/Collision");

sim = Simulator();

numberOfObjects = 15;
numberPerColumn = 5;
gridPoints = CreateGrid([-1.5;0;5],numberOfObjects,numberPerColumn,1.1);

for i = 1:numberOfObjects
    % Place the object
    entity_i = Entity(sprintf("Object %d (Box)",i));
    entity_i.Transform.position = gridPoints(:,i);
    % Add elements
    entity_i.AddElement(RigidBody());
    entity_i.AddElement(BoxCollider());
    renderer = entity_i.AddElement(MeshRenderer());
    extents = 0.5*[1;1;1];
    renderer.Mesh = MeshGenerator.CuboidFromExtents(-extents,extents);
    renderer.Alpha = 0.2;
    if mod(i,2) == 0
        renderer.Colour = "b";
    else
        renderer.Colour = "c";
    end
    % Assign the object
    sim.AddEntity(entity_i);
end

% Add an obstacle
fixed = Entity("Obstacle");
fixed.Transform.position = [0;0;2];
fixed.Transform.IsStatic = true;
% Add elements
fixed.AddElement(RigidBody())
fixed.AddElement(SphereCollider());
renderer = fixed.AddElement(MeshRenderer());
renderer.Mesh = MeshGenerator.Sphere(zeros(3,1),1);
renderer.Colour = "r";
% Add the fix object
sim.AddEntity(fixed);

% Add the ground plane
ground = Entity("Ground");
ground.Transform.IsStatic = true;
ground.Transform.scale = [20;20;1];
% Add elements
ground.AddElement(PlaneCollider());
renderer = ground.AddElement(MeshRenderer());
renderer.Mesh = MeshGenerator.Plane(zeros(3,1),[0;0;1],1,1);
renderer.Colour = "g";
sim.AddEntity(ground);

% Simulate
sim.PhysicsSubSteps = 10;
sim.Simulate(inf);
