%% COLLIDER-CODE 
% Author: James A. Douthwaite 

classdef ColliderCode < uint8
   % Assign uint8 type to enumeration values
   enumeration
      None(0);          % Generic 
      Point(1);
      Line(2);
      Ray(3);
      Triangle(4);      
      Sphere(5);        % Spherical bounding box assumption
      Plane(6);         % Planar
      Capsule(7);       % Capsule enclosed volume
      AABB(8);          % Axis aligned bounding box
      OBB(9);           % Object orientated bounding box
      Mesh(10);         % Mesh enclosed volume
   end
end