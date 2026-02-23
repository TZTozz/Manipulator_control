%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end

        function bJi = getJacobianOfLinkWrtBase(self, j)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint indnex ;

            % The function returns:
            % bJi
            Tn = self.gm.getTransformWrtBase(j);
            bJi = zeros(6, j);
            
            for i = 1:j
                Ti = self.gm.getTransformWrtBase(i);

                if(self.gm.jointType(i) == 0)   %Rotational joint
    
                    bJi(1:3, i) = Ti(1:3, 3);
                    bJi(4:6, i) = cross(Ti(1:3, 3), Tn(1:3, 4) - Ti(1:3, 4));
                end
    
                if(self.gm.jointType(i) == 1)   %Prismatic joint
                    bJi(4:6, i) = Ti(1:3, 3);
                end
            end
        end

        %% Update Jacobian function
        function updateJacobian(self)
            % The function update:
            % - J: end-effector jacobian matrix
            self.J = getJacobianOfToolWrtBase(self);
        end
        

        function [J_t] = getJacobianOfToolWrtBase(self)
            bJt = zeros(6, self.gm.jointNumber);
            bTt = self.gm.getToolTransformWrtBase();
            for j = 1:self.gm.jointNumber
                bTj = self.gm.getTransformWrtBase(j);
                if self.gm.jointType(j) == 0
                    bJt(1:3, j) = bTj(1:3, 3);
                    bJt(4:6, j) = cross(bTj(1:3, 3), (bTt(1:3, 4) - bTj(1:3, 4)));
                end
                if self.gm.jointType(j) == 1
                    bJt(1:3, j) = [0; 0; 0];
                    bJt(4:6, j) = bTj(1:3, 3);
                end
            end
            J_t  = bJt;
        end 
    end
end

