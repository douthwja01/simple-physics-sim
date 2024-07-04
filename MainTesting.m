
clear all;
close all;
addpath("Engine");

sim = Simulator();

numberOfObjects = 12;
numberPerColumn = 3;
gridPoints = CreateGrid([-1.5;0;4],numberOfObjects,numberPerColumn,1.3);

% for i = 1:numberOfObjects
%     % Place the object
%     entity_i = EntityCreator.Box( ...
%         "Box", ...
%         gridPoints(:,i), ...
%         Quaternion.FromEulers(0,0,pi/6));
%     % Add elements
%     entity_i.RigidBody = RigidBody();
%     % Renderer
%     entity_i.Renderer.Alpha = 0.2;
%     if mod(i,2) == 0
%         entity_i.Renderer.Colour = "b";
%     else
%         entity_i.Renderer.Colour = "c";
%     end
% 
%     entity_i.Transform.SetLocalScale([1;3;1]);
% 
%     % Assign the object
%     sim.Add(entity_i);
% end

A = EntityCreator.Box( ...
        "Box A", ...
        [0;0;5], ...
        Quaternion.FromEulers(0,0,pi/6));
A.RigidBody = RigidBody();
sim.Add(A);

B = EntityCreator.Box( ...
        "Box B", ...
        [0;0;4], ...
        Quaternion.FromEulers(0,0,pi/6));
B.RigidBody = RigidBody();

B.Transform.Parent = A.Transform;
sim.Add(B);

C = EntityCreator.Box( ...
        "Box C", ...
        [0;0;4], ...
        Quaternion.FromEulers(0,0,pi/6));
C.RigidBody = RigidBody();

C.Transform.Parent = B.Transform;
sim.Add(C);

% Add an obstacle
fixed = EntityCreator.Sphere( ...
    "Obstacle", ...
    [0;0;2], ...
    Quaternion.Random());
% Do not move
fixed.RigidBody = RigidBody();
fixed.RigidBody.IsStatic = true;
fixed.Renderer.Colour = "r";
% Add the fix object
sim.Add(fixed);

% Add the ground plane
ground = EntityCreator.Plane("Ground",zeros(3,1));
ground.Transform.SetWorldScale([10;10;1]);
ground.RigidBody = RigidBody();
ground.RigidBody.IsStatic = true;
% Visuals
ground.Renderer.Colour = "g";
% Add the ground object
sim.Add(ground);

%found = sim.Find("Name","Box");

%% Simulator configuration
sim.WorldSize = 15;
sim.World.SubSteps = 5;
% Numeric integrators
sim.World.Integrator = EulerIntegrator();
% Collision solvers
% sim.Physics.BroadPhaseSolver = OctreeSolver();
% Simulate
sim.Simulate(inf);
