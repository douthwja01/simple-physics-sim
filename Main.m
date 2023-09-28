
clear all;
close all;
addpath("Common");

numberOfObjecst = 10;
objectArray = SimObject.empty;
for i = 1:numberOfObjecst
    objectArray(i) = SimObject();
    objectArray(i).Position = RandZero(3)*10;
    objectArray(i).Position(3) = rand(1)*10;
end

sim = Simulator();
sim.Objects = objectArray;
sim.Simulate(inf);
