%% COLLIDER-CODE 
% Author: James A. Douthwaite 

classdef ColliderCode < uint8
   % Assign uint8 type to enumeration values
   enumeration
      None(0);          % Generic 
      Line(1);
      Sphere(2);        % Spherical bounding box assumption
      Plane(3);         % Planar
      AABB(4);          % Axis aligned bounding box
      OBB(5);           % Object orientated bounding box
      Capsule(6);       % Capsule enclosed volume
      Mesh(7);          % Mesh enclosed volume
   end
end