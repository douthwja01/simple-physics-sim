
clear all;
close all;
addpath("Engine");

sim = Simulator();

numberOfObjects = 5;
numberPerColumn = 10;
gridPoints = CreateGrid([0;0;4],numberOfObjects,numberPerColumn,1.5);

for i = 1:numberOfObjects
    % Place the object
    entity_i = EntityCreator.Box(sprintf("Object %d (Box)",i),gridPoints(:,i),Quaternion.Identity);

    entity_i.Transform.SetWorldOrientation(Quaternion.FromEulers(rand(1),rand(1),rand(1)));
    
    % Add elements
    entity_i.RigidBody = RigidBody();
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
fixed.RigidBody = RigidBody();
fixed.RigidBody.IsStatic = true;
fixed.Renderer.Colour = "r";
fixed.Renderer.Alpha = 0.2;
% Add the fix object
sim.Add(fixed);

% Add the ground plane
ground = EntityCreator.Plane("Ground");
ground.Transform.SetWorldScale([10;10;1]);
ground.RigidBody = RigidBody();
ground.RigidBody.IsStatic = true;
% Collisions
ground.Renderer.Colour = "g";
sim.Add(ground);

%% Simulator configuration
sim.WorldSize = 15;
sim.World.SubSteps = 5;
sim.World.EnableSubStepping = false;
% Numeric integrators
sim.World.Integrator = EulerIntegrator();
% Collision solvers
sim.World.BroadPhaseDetector = SweepAndPruneBPCD();
sim.World.Dynamics = RNEDynamics(); %FeatherstoneDynamics();
% sim.World.AddSolver(RotationalImpulseCR())
% Simulate
sim.Simulate(inf);
