
clear all;
close all;
addpath("Common");

sim = Simulator();

numberOfObjects = 10;
objectArray = SimObject.empty;
for i = 1:numberOfObjects
    object_i = SimObject();
%     objectArray(i).Position = RandZero(3)*10;
%     objectArray(i).Position(3) = rand(1)*10;

    if (i == 1)
        spawnPosition = [0;0;10];
    else
        spawnPosition = spawnPosition + [2;0;0]*object_i.Radius;
    end
    % Place the object
    object_i.Position = spawnPosition;
    % Assign the object
    sim.AddObject(object_i);
end

sim.Simulate(inf);
