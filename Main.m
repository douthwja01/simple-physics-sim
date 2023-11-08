
clear all;
close all;
addpath("Common");
addpath("Physics");
addpath("Physics/Integrators");
addpath("Physics/Solvers");
addpath("Physics/Elements");
addpath("Physics/Elements/Colliders/");

sim = Simulator();

numberOfObjects = 15;
numberPerColumn = 5;
gridPoints = CreateGrid([0;0;5],numberOfObjects,numberPerColumn,1.1);

for i = 1:numberOfObjects
    % Place the object
    entity_i = Entity();
    entity_i.AddElement(RigidBody());
    transform_i = entity_i.GetElement("Transform");
    transform_i.position = gridPoints(:,i);
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
