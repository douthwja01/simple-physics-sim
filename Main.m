
clear all;
close all;
addpath("Engine");

% Simulation setup
worldSize = 15;
sim = Simulator(worldSize);

numberOfObjects = 5;
numberPerColumn = 10;
gridPoints = CreateGrid([0;0;4],numberOfObjects,numberPerColumn,1.5);

for i = 1:numberOfObjects
    % Place the object
    entity_i = EntityCreator.Box(sprintf("Object %d (Box)",i),gridPoints(:,i),Quaternion.Identity);

    entity_i.Transform.SetWorldOrientation(Quaternion.FromEulers(rand(1),rand(1),rand(1)));
    
    % Add elements
    entity_i.Body = RigidBody();
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
fixed = EntityCreator.Sphere("Obstacle",[0;0;2]);
% Add elements
fixed.Body = RigidBody();
fixed.Body.IsStatic = true;
fixed.Renderer.Colour = "r";
fixed.Renderer.Alpha = 0.2;
% Add the fix object
sim.Add(fixed);

% Add the ground plane
ground = EntityCreator.Plane("Ground");
ground.Transform.SetWorldScale([10;10;1]);
ground.Body = RigidBody();
ground.Body.IsStatic = true;
% Collisions
ground.Renderer.Colour = "g";
sim.Add(ground);

%% Simulator configuration
sim.Physics.SubSteps = 5;
sim.Physics.EnableSubStepping = false;
% Backend
sim.Physics.Dynamics = GravityOnlyDynamics();
% sim.Physics.Dynamics = RNEDynamics(); 
% sim.Physics.Dynamics = FeatherstoneDynamics();
% sim.Physics.OdeSolver = EulerSolver();
% Simulate
sim.Simulate(inf);
