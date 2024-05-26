classdef RotationalImpulseCR < CollisionResolver
    %ROTATIONALIMPULSESOLVER A collision solver that uses the rotational
    % impulse solution method.

    % Inspired by: https://www.chrishecker.com/images/e/e7/Gdmphys3.pdf
    % via: https://www.youtube.com/watch?v=VbvdoLQQUPs&t=1076s

    properties (Constant)
        Name = "A rotational impulse-based collision resolution implementation.";
    end

    methods
        function [this] = Resolve(this,manifolds,dt)
            % SOLVE - Solve the set of the collision manifolds using the
            % rotational impulse solver.

            % Move through the collsion instances
            for i = 1:numel(manifolds)
                % The current manifold
                this.ResolveIndividual(manifolds(i));
            end
        end
    end
    methods (Static)
        function [data] = ResolveIndividual(manifold)

            bodyA = manifold.BodyA;
            bodyB = manifold.BodyB;

            if isempty(bodyA)
                return;
            end
            if isempty(bodyB)
                return;
            end

            contacts = {manifold.Points.A,manifold.Points.B};
            contactCount = numel(contacts);
            n = -manifold.Points.Normal;

            e = bodyA.Restitution*bodyB.Restitution;

            centroidTfA = bodyA.Transform;
            centroidTfB = bodyB.Transform;
            centroidPositionA = centroidTfA.Inertial.Position;
            centroidVelocityA = centroidTfA.Velocity;
            centroidAngularVelocityA = centroidTfA.AngularVelocity;
            centroidPositionB = centroidTfA.Inertial.Position;
            centroidVelocityB = centroidTfB.Velocity;
            centroidAngularVelocityB = centroidTfB.AngularVelocity;

            impulseData = [];
            
            for i = 1:contactCount
                
                contactPoint = contacts{i};
                
                rap = contactPoint - centroidPositionA;
                rbp = contactPoint - centroidPositionB;

                % We need to calculate the perpendicular vector at the
                % contact point.


%                 % Recalulate the relative velocity
%                 relativeVelocity = centroidVelocityB - centroidVelocityA;
%                 relativeSpeed = dot(relativeVelocity, collisionNormal);
%                 % Tangent vector
%                 tangent = relativeVelocity - relativeSpeed * collisionNormal;
%                 % Safe normalize
%                 n_tangent = norm(tangent);
%                 if n_tangent > 1E-5
%                     tangent = tangent/n_tangent;
%                 end


%                 data = [contactPoint,contactPoint+tangent]';
%                 plot3(gca,data(:,1),data(:,2),data(:,3),"ro","LineWidth",2);

%                 raPerp = cross(ra,centroidAngularVelocityA);
       
%                 data = [centroidPositionA,contactPoint]';
%                 plot3(gca,data(:,1),data(:,2),data(:,3),"go","LineWidth",2);
% 
%                 data = cross(rap,n); 
                
%                 data = [contactPoint,contactPoint +]';
%                 plot3(gca,data(:,1),data(:,2),data(:,3),"g^","LineWidth",2);

%                 raPerp = tangent

                % The point velocity at contact
                v_ab = (centroidVelocityB + cross(rbp,centroidAngularVelocityB)) - ...
                    (centroidVelocityA + cross(rap,centroidAngularVelocityA));
                
                
                % Calculate the contact speed
                contactSpeed = dot(v_ab,n);
                if contactSpeed > 0
                    continue;
                end

                

                % The perpendicular scalars
%                 raPerpDotN = dot(raPerp,normal);
%                 rbPerpDotN = dot(rbPerp,normal);

                % Numerator
                numer = -(1+e)*dot(v_ab,n);
                % Denominator 
                d_termA = dot(n,n)*(bodyA.InverseMass + bodyB.InverseMass);
                d_termB = cross(bodyA.InverseInertia*cross(rap,n),rap);
                d_termC = cross(bodyB.InverseInertia*cross(rbp,n),rbp);
                denom = d_termA + dot(d_termB + d_termC,n);

                j = numer/denom;


%                 rAI = (raPerpDotN * raPerpDotN) * bodyA.InvInertia;
%                 rBI = (rbPerpDotN * rbPerpDotN) * bodyB.InvInertia;
%                 denominator = bodyA.InvMass + bodyB.InvMass + rAI + rBI;
% 
%                 % Impulse
%                 j = -(1 + resitutionA*resitutionB) * contactSpeed;
%                 j = j/denominator;



                j = j/contactCount;

                % Impulse vector
                impulseStruct = struct( ...
                    "Impulse",j * n, ...
                    "PointOnA",rap, ...
                    "PointOnB",rbp);

                impulseData = vertcat(impulseData,impulseStruct);
            end

            % Apply the impulses
            for i = 1:numel(impulseData)
                data = impulseData(i);

                raImpulse = cross(data.PointOnA,data.Impulse);
                rbImpulse = cross(data.PointOnB,data.Impulse);

                % Modify body a's velocities
                centroidTfA.Velocity = centroidTfA.Velocity - data.Impulse*bodyA.InverseMass;
                centroidTfA.AngularVelocity = centroidTfA.AngularVelocity - bodyA.InverseInertia*raImpulse;

                % Modify body-b's velocities
                centroidTfB.Velocity = centroidTfB.Velocity + data.Impulse*bodyB.InverseMass;
                centroidTfB.AngularVelocity = centroidTfB.AngularVelocity + bodyB.InverseInertia*rbImpulse;
            end
        end
    end
end

