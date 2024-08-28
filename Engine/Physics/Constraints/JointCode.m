%% Joint code from linked robotic systems (JointCode.m) %%%%%%%%%%%%%%%%%%
% Definitions from "Rigidbody Dynamics Algorithms" [Table 4.1 (p88)]

classdef JointCode < uint8

    enumeration
        None(0);
        Fixed(1);
        Revolute(2);
        Prismatic(3);
        Helical(4);
        Cylindrical(5);
        Planar(6);
        Floating(7);
    end
end