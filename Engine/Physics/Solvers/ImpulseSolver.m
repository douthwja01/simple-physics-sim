classdef ImpulseSolver < Solver
    % This basic collision solver resolves collisions between two objects
    % by calculating the impulse required to move the two objects apparent
    % by the next time-step.

    methods
        function [this] = Solve(this,collisions,dt)

            for i = 1:numel(collisions)
                % Replaces non dynamic objects with default values.

                manifold = collisions(i);
                % Get the corresponding elements from each member entity
                transformA = manifold.ColliderA.Entity.GetElement("Transform");
                rigidBodyA = manifold.ColliderA.Entity.GetElement("RigidBody");
                transformB = manifold.ColliderB.Entity.GetElement("Transform");
                rigidBodyB = manifold.ColliderB.Entity.GetElement("RigidBody");
                % Logicals
                aHasRigidBody = ~isempty(rigidBodyA);
                bHasRigidBody = ~isempty(rigidBodyB);

                p_b = transformB.position;
                p_a = transformA.position;

                if aHasRigidBody
                    fprintf("boop A\n.");
                    v_a = transformA.Velocity;
                    m_a = rigidBodyA.Mass;
                else
                    v_a = zeros(3,1);
                    m_a = 0;
                end
                if bHasRigidBody
                    fprintf("boop B\n.");
                    v_b = transformB.Velocity;
                    m_b = rigidBodyB.Mass;
                else
                    v_b = zeros(3,1);
                    m_b = 0;
                end
                
                massRatioA = 2*m_b/(m_a + m_b);
                massRatioB = 2*m_a/(m_a + m_b);

                p_ab = -manifold.Points.Normal; %p_a - p_b;
                v_ab = v_a - v_b;
                tempA = dot(v_ab,p_ab)/norm(p_ab);
                if tempA <= 0
                    continue;
                end
                v_a = v_a - massRatioA * tempA * p_ab/norm(p_ab);

                p_ba = manifold.Points.Normal; %p_b - p_a;
                v_ba = v_b - v_a;
                tempB = dot(v_ba,p_ba)/norm(p_ba);
                if tempB <= 0
                    continue;
                end
                v_b = v_b - massRatioB * tempB * (p_ba)/norm(p_ba);

                transformA.Velocity = v_a;
                transformB.Velocity = v_b;


%                 % aBody and bBody are Rigidbody;
%                 if aHasRigidBody
%                     velocityA = transformA.Velocity;
%                     aInvMass = rigidBodyA.InverseMass;
%                     a_e = rigidBodyA.Restitution;
%                 else
%                     velocityA = zeros(3,1);
%                     aInvMass = 1;
%                     a_e = 1;
%                 end
% 
%                 if bHasRigidBody
%                     velocityB = transformB.Velocity;
%                     bInvMass = rigidBodyB.InverseMass;
%                     b_e = rigidBodyB.Restitution;
%                 else
%                     velocityB = zeros(3,1);
%                     bInvMass = 1;
%                     b_e = 1;
%                 end
% 
%                 collisionNormal = manifold.Points.Normal;
% 
%                 rVel = velocityB - velocityA;
%                 nSpd = dot(rVel, collisionNormal);
% 
%                 % === Impulse ===
% 
%                 % This is important for convergence
%                 % a negative impulse would drive the objects closer together
%                 if nSpd <= 0
%                     continue;
%                 end
%                 % Resitution magnitude
%                 e = a_e*b_e;
%                 j = -(1 + e) * nSpd / (aInvMass + bInvMass);
%                 % Impulse along the collision axis
%                 impluse = j * collisionNormal;
% 
%                 if aHasRigidBody && rigidBodyA.IsSimulated
%                     velocityA = velocityA - impluse * aInvMass;
%                 end
% 
%                 if bHasRigidBody && rigidBodyB.IsSimulated
%                     velocityB = velocityB + impluse * bInvMass;
%                 end
% 
%                 % ==== Friction ====
%                 rVel = velocityB - velocityA;
%                 nSpd = dot(rVel,collisionNormal);
% 
%                 % Recalculate the relative speed 
%                 if nSpd <= 0
%                     continue;
%                 end
% 
%                 % The vector along the surface
%                 tangent = rVel - nSpd * collisionNormal;
%                 
%                 % Normalise the tangent vector
%                 if norm(tangent) > 0.0001  
%                     tangent = tangent/norm(tangent);
%                 end
% 
%                 fVel = dot(rVel, tangent);
% 
%                 if aHasRigidBody
%                     aSF = rigidBodyA.StaticFriction;
%                     aDF = rigidBodyA.DynamicFriction;
%                 else
%                     aSF = 0;
%                     aDF = 0;
%                 end
% 
%                 if bHasRigidBody
%                     bSF = rigidBodyB.StaticFriction;
%                     bDF = rigidBodyB.DynamicFriction;
%                 else
%                     bSF = 0;
%                     bDF = 0;
%                 end
%                 %
%                 mu = norm([aSF,bSF]); %(scalar)glm::vec2(aSF, bSF).length();
% 
%                 f = - fVel / (aInvMass + bInvMass);
%                 if abs(f) < j * mu
%                     friction = f * tangent;
%                 else
%                     mu = norm([aDF, bDF]);
%                     friction = -j * tangent * mu;
%                 end
%                 % Exert friction on A
%                 if aHasRigidBody && rigidBodyA.IsSimulated
%                     transformA.Velocity = velocityA - friction * aInvMass;
%                 end
%                 % Exert friction on B
%                 if bHasRigidBody && rigidBodyB.IsSimulated
%                     transformB.Velocity = velocityB + friction * bInvMass;
%                 end

            end
        end
    end
end