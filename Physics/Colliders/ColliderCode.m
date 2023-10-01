%% COLLIDER-CODE 
% Author: James A. Douthwaite 

classdef ColliderCode < uint8
   % Assign uint8 type to enumeration values
   enumeration
      None(0);          % Generic 
      Sphere(1);        % Spherical bounding box assumption
      Plane(2);         % Planar
      AABB(3);          % Axis aligned bounding box
      OBB(4);           % Object orientated bounding box
      Capsule(5);       % Capsule enclosed volume
      Mesh(6);          % Mesh enclosed volume
   end
end