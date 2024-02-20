classdef ImpulseCollisionSolver < NarrowPhaseCollisionSolver
    % This basic collision solver resolves collisions between two objects
    % by calculating the impulse required to move the two objects apparent
    % by the next time-step.

    methods
        function [this] = Solve(this,manifolds,dt)
            % Solve the set of collisions using the impulse solver.

            % Move through the collsion instances
            for i = 1:numel(manifolds)
                % The current manifold
                manifold = manifolds(i);

                bodyA = manifold.ColliderA.Entity.RigidBody;
                hasRigidBodyA = ~isempty(bodyA);
                tfA = manifold.ColliderA.Transformation;

                bodyB = manifold.ColliderB.Entity.RigidBody;
                hasRigidBodyB = ~isempty(bodyB);
                tfB = manifold.ColliderB.Transformation;

                collisionNormal = -manifold.Points.Normal;

                % Object velocities
                velocityA = tfA.Velocity;
                velocityB = tfB.Velocity;
                relativeVelocity = velocityB - velocityA;
                % Scalar speed
                relativeSpeed = dot(relativeVelocity, collisionNormal);

                % Extract dynamic properties
                if hasRigidBodyA
                    inverseMassA = bodyA.InverseMass;
                    resitutionA = bodyA.Restitution;
                    staticFrictionA = bodyA.StaticFriction;
                    dynamicFrictionA = bodyA.DynamicFriction;
                else
                    inverseMassA = 1;
                    resitutionA = 1;
                    staticFrictionA = 0;
                    dynamicFrictionA = 0;
                end

                if hasRigidBodyB
                    inverseMassB = bodyB.InverseMass;
                    resitutionB = bodyB.Restitution;
                    staticFrictionB = bodyB.StaticFriction;
                    dynamicFrictionB = bodyB.DynamicFriction;
                else
                    inverseMassB = 1;
                    resitutionB = 1;
                    staticFrictionB = 0;
                    dynamicFrictionB = 0;
                end

                % --- Impulse

                % This is important for convergence
                % a negative impulse would drive the objects closer together
                if relativeSpeed >= 0
                    continue;
                end

                % Impulse
                impulse = -(1 + resitutionA*resitutionB) * relativeSpeed / (inverseMassA + inverseMassB);
                % Impulse vector
                impulseVector = impulse * collisionNormal;

                % Apply the velocity modification to the first body
                if hasRigidBodyA && bodyA.IsSimulated
                    velocityA = velocityA - impulseVector * inverseMassA;
                end
                % Apply the velocity modification to the second body
                if hasRigidBodyB && bodyB.IsSimulated
                    velocityB = velocityB + impulseVector * inverseMassB;
                end

                % --- Friction

                % Recalulate the relative velocity
                relativeVelocity = velocityB - velocityA;
                relativeSpeed = dot(relativeVelocity, collisionNormal);

                % Tangent vector
                tangent = relativeVelocity - relativeSpeed * collisionNormal;
                % Safe normalize
                n_tangent = norm(tangent);
                if n_tangent > 1E-5
                    tangent = tangent/n_tangent;
                end

                mu = norm([staticFrictionA;staticFrictionB]);
                % Calculate the effective friction force
                friction = - dot(relativeVelocity, tangent) / (inverseMassA + inverseMassB);
                if abs(friction) < impulse * mu
                    frictionVector = friction * tangent;
                else
                    mu = norm([dynamicFrictionA;dynamicFrictionB]);
                    frictionVector = -impulse * tangent * mu;
                end

                % If the first body can have friction applied and is simulated.
                if hasRigidBodyA && bodyA.IsSimulated
                    tfA.Velocity = velocityA - frictionVector * inverseMassA;
                end

                % If the second body can have friction applied and is simulated.
                if hasRigidBodyB && bodyB.IsSimulated
                    tfB.Velocity = velocityB - frictionVector * inverseMassB;
                end

                % Should be adding an impulse directly to the object to
                % create the implied velocity?
            end
        end
    end
end