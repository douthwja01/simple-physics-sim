
clear all;
close all;
addpath("Common");

numberOfObjecst = 10;
objectArray = SimObject.empty;
for i = 1:numberOfObjecst
    objectArray(i) = SimObject();
    objectArray(i).p = RandZero(3)*10;
    objectArray(i).p(3) = rand(1)*10;
end

sim = Simulator();
sim.Objects = objectArray;
sim.Simulate(2);
