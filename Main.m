
clear all;
close all;
addpath("Common");

sim = Simulator();

numberOfObjects = 20;
objectArray = SimObject.empty;
for i = 1:numberOfObjects
    object_i = SimObject();

    if (i == 1)
        spawnPosition = [0;0;10];
    else
        spawnPosition = spawnPosition + 0.1*RandZero(3) + [2;0;0]*object_i.Radius;
    end
    % Place the object
    object_i.Position = spawnPosition;
    % Assign the object
    sim.AddObject(object_i);
end

% sim.AddFixedObject();

sim.Simulate(inf);
