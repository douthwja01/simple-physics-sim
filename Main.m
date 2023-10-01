
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
        spawnPosition = [0;0;10];
    else
        spawnPosition = spawnPosition + 0.1*RandZero(3) + [2;0;0];
    end
    % Place the object
    entity_i = Entity(spawnPosition);
    entity_i.AddElement(RigidBody());
    entity_i.AddElement(SphereCollider());
    entity_i.AddElement(MeshRenderer());
    % Assign the object
    sim.AddEntity(entity_i);
end

% Simulate
sim.Simulate(inf);
