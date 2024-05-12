%% COLLIDER-CODE 
% Author: James A. Douthwaite 

classdef ColliderCode < uint8
   % Assign uint8 type to enumeration values
   enumeration
      None(0);          % Generic 
      Point(1);
      Line(2);
      Ray(3);     
      Sphere(4);        % Spherical bounding box assumption
      Plane(5);         % Planar
      Capsule(6);       % Capsule enclosed volume
      AABB(7);          % Axis aligned bounding box
      OBB(8);           % Object orientated bounding b
      Triangle(9); 
      Mesh(10);         % Mesh enclosed volume
   end
end