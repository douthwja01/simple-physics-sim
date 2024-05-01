
clear all;
close all;
addpath("Engine");

sim = Simulator();

position = [0;0;3];

numberOfLinks = 3;
for i = 1:numberOfLinks
    % Place the object
    entity_i = EntityCreator.Box( ...
        sprintf("Box %d",i), ...
        position, ...
        Quaternion.FromEulers(0,0,0));
    % Add elements
    entity_i.RigidBody = RigidBody();
    % Renderer
    entity_i.Renderer.Alpha = 0.2;
    if mod(i,2) == 0
        entity_i.Renderer.Colour = "b";
    else
        entity_i.Renderer.Colour = "c";
    end

    position = position + [2;0;0];

    % Assign the object
    sim.Add(entity_i);
end

% Add the ground plane
ground = EntityCreator.Plane("Ground",zeros(3,1));
% Do not move
ground.Transform.IsStatic = true;
% Collisions
ground.Collider.Width = 10;
ground.Collider.Depth = 10;
% Visuals
ground.Renderer.Colour = "g";
ground.Renderer.Width = 10;
ground.Renderer.Depth = 10;
% Add ground to world
sim.Add(ground);

% Create joints
boxOne = sim.Find("Name","Box 1");
boxOne.Joints = FixedJoint();

boxTwo = sim.Find("Name","Box 2");
boxTwo.Joints = RevoluteJoint();

% Simulate
sim.WorldSize = 10;
sim.Physics.SubSteps = 10;
sim.Physics.Integrator = EulerIntegrator();
sim.Simulate(inf);
