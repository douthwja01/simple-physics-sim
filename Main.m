
clear all;
close all;
addpath("Common");
addpath("Physics");
addpath("Physics/Colliders");
addpath("Physics/Objects");
addpath("Physics/Solvers");

sim = Simulator();

numberOfObjects = 10;
for i = 1:numberOfObjects
    % Spawn positions
    if (i == 1)
        spawnPosition = [0;0;5];
    else
        spawnPosition = spawnPosition + 0.1*RandZero(3) + [2;0;0];
    end
    % Place the object
    entity_i = Entity(spawnPosition);
    entity_i.Name = sprintf("Object %d",i);
    entity_i.AddElement(RigidBody());
    entity_i.AddElement(SphereCollider());
    renderer = MeshRenderer();
    renderer.Mesh = MeshGenerator.Sphere(zeros(3,1),1);
    renderer.Colour = "b";
    entity_i.AddElement(renderer);
    % Assign the object
    sim.AddEntity(entity_i);
end

% fixed = Entity([0;0;5]);
% fixed.AddElement(RigidBody())
% fixed.AddElement(SphereCollider());
% renderer = MeshRenderer();
% renderer.Mesh = MeshGenerator.Sphere(zeros(3,1),1);
% renderer.Colour = "b";
% fixed.AddElement(renderer);
% tf_fixed = fixed.GetElement("Transform");
% tf_fixed.IsStatic = true;
% % Add the fix object
% sim.AddEntity(fixed);

% Add the ground plane
ground = Entity([0;0;0]);
ground.Name = "Ground";
tf_ground = ground.GetElement("Transform");
tf_ground.IsStatic = true;
ground.AddElement(PlaneCollider());
renderer = MeshRenderer();
renderer.Mesh = MeshGenerator.Plane(zeros(3,1),[0;0;1],20,20);
renderer.Colour = "g";
ground.AddElement(renderer);
sim.AddEntity(ground);

% Simulate
sim.Simulate(inf);
