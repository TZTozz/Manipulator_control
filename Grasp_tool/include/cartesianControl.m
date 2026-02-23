%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            b_e = self.cartesianError(bTg);

            LAMBDA = [self.k_a * eye(3), zeros(3,3); zeros(3,3), self.k_l * eye(3)];
            x_dot = LAMBDA * b_e;
            
        end

        function [b_e] = cartesianError(self, bTg)
            %% cartesianError function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % b_e : cartesian error

            bTt = self.gm.getToolTransformWrtBase();
            tTg = bTt\bTg;
            bRt = bTt(1:3,1:3);
            [h, theta] = RotToAngleAxis(tTg(1:3,1:3));

            % position error expressed in base frame
            r = bTg(1:3, 4) - bTt(1:3, 4); 

            % angular error expressed in base frame
            b_rho = bRt*(h*theta);

            b_e = [b_rho; r];   % cartesian error
        end
    end
end

