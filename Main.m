
clear all;
close all;
addpath("Common");
addpath("Physics");
addpath("Physics/Colliders");
addpath("Physics/Integrators");
addpath("Physics/Solvers");

sim = Simulator();

numberOfObjects = 15;
for i = 1:numberOfObjects
    % Spawn positions
    if (i == 1)
        spawnPosition = [0;0;5];
    else
        spawnPosition = spawnPosition + RandZero(3); %+ [RandZero(2);0]*2;
    end

    % Place the object
    entity_i = Entity();
    entity_i.AddElement(RigidBody());
    transform_i = entity_i.GetElement("Transform");
    transform_i.position = spawnPosition;
    entity_i.Name = sprintf("Object %d (Box)",i);
    entity_i.AddElement(BoxCollider());  
    renderer = MeshRenderer();
    extents = 0.5*[1;1;1];
    renderer.Mesh = MeshGenerator.CuboidFromExtents(-extents,extents);
    renderer.Alpha = 0.2;
    if mod(i,2) == 0
        renderer.Colour = "b";
    else
        renderer.Colour = "c";
    end
    entity_i.AddElement(renderer);

%     if mod(i,2) == 0
%         entity_i.Name = sprintf("Object %d (Sphere)",i);
%         entity_i.AddElement(SphereCollider());
%         renderer = MeshRenderer();
%         renderer.Mesh = MeshGenerator.Sphere(zeros(3,1),1);
%         renderer.Colour = "b";
%         renderer.Alpha = 0.2;
%     else
%         entity_i.Name = sprintf("Object %d (Box)",i);
%         entity_i.AddElement(BoxCollider());
%         renderer = MeshRenderer();
%         extents = 0.5*ones(3,1);
%         renderer.Mesh = MeshGenerator.CuboidFromExtents(-extents,extents);
%         renderer.Colour = "c";
%         renderer.Alpha = 0.2;
%     end
%     entity_i.AddElement(renderer);
    % Assign the object
    sim.AddEntity(entity_i);
end

fixed = Entity("Obstacle");
fixed.AddElement(RigidBody())
fixed.AddElement(SphereCollider());

transform_f = fixed.GetElement("Transform");
transform_f.position = [0;0;2];
transform_f.IsStatic = true;

renderer = MeshRenderer();
renderer.Mesh = MeshGenerator.Sphere(zeros(3,1),1);
renderer.Colour = "r";
fixed.AddElement(renderer);
% Add the fix object
sim.AddEntity(fixed);

% Add the ground plane
ground = Entity("Ground");
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
