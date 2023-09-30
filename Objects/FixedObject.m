
classdef FixedObject < CollisionObject

    properties
        Acceleration = zeros(3,1);
        Velocity = zeros(3,1);
        Position = zeros(3,1);
    end
end