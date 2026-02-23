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
            % bTg : goal frame matrix (4x4)
            % Outputs :
            % b_e : cartesian error vector [angular_error; linear_error] (6x1)
        
            bTt = self.gm.getToolTransformWrtBase();
            
            r_err = bTg(1:3, 4) - bTt(1:3, 4); 
        
            bRg = bTg(1:3, 1:3);
            bRt = bTt(1:3, 1:3);

            ypr_goal = rotm2eul(bRg, 'ZYX')'; 
            ypr_curr = rotm2eul(bRt, 'ZYX')';

            e_ypr = angdiff(ypr_curr, ypr_goal); 

            yaw   = ypr_curr(1);
            pitch = ypr_curr(2);

            T = [ 0, -sin(yaw),  cos(yaw)*cos(pitch);
                  0,  cos(yaw),  sin(yaw)*cos(pitch);
                  1,         0,         -sin(pitch)];
        

            b_rho = T * e_ypr;
            b_e = [b_rho; r_err];   
        end
    end
end

