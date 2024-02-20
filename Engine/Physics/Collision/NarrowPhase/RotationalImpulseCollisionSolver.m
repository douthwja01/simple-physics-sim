classdef RotationalImpulseCollisionSolver < NarrowPhaseCollisionSolver
    %ROTATIONALIMPULSESOLVER A collision solver that uses the rotational 
    % impulse solution method.

    methods        
        function [this] = Solve(this,manifolds,dt)
            % SOLVE - Solve the set of the collision manifolds using the
            % rotational impulse solver.

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

                pointA = manifold.Points.A;
                pointB = manifold.Points.B;


                centroidA = tfA.GetWorldPosition();

                armVector = pointA - centroidA;
                
                forceBA = bodyB.Mass * tfB.Acceleration;


                if hasRigidBodyA
                    
                    % Add the torque to the body
                    bodyA.AddTorque(forceBA,armVector)
                end

                if hasRigidBodyB
                    
                    % Add the torque to the body
                    bodyB.AddTorque(forceAB,armVector)
                end
            end
        end
    end
end

