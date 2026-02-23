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
            %% cartesianError function (VERSIONE QUATERNIONI)
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % b_e : cartesian error
            
            bTt = self.gm.getToolTransformWrtBase();
            tTg = bTt \ bTg; 
            tRg = tTg(1:3, 1:3);
            
            q_err = rotm2quat(tRg);
            
           if q_err(1) < 0
                q_err = -q_err;
            end
            
            q_vec = q_err(2:4)';
            t_rho = 2 * q_vec; 
            
            bRt = bTt(1:3, 1:3);
            b_rho = bRt * t_rho;
            
            r = bTg(1:3, 4) - bTt(1:3, 4); 
            
            b_e = [b_rho; r];   
        end
    end
end

function q = RotToQuat(R)
    % RotToQuat converte una matrice di rotazione 3x3 in un quaternione.
    % Output: q = [w, x, y, z] (Scalare come primo elemento)
    % Algoritmo numericamente stabile per evitare divisioni per zero.

    tr = trace(R);
    
    if tr > 0
        S = sqrt(tr + 1.0) * 2; % S = 4 * qw
        qw = 0.25 * S;
        qx = (R(3,2) - R(2,3)) / S;
        qy = (R(1,3) - R(3,1)) / S;
        qz = (R(2,1) - R(1,2)) / S;
    else
        if (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
            S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; % S = 4 * qx
            qw = (R(3,2) - R(2,3)) / S;
            qx = 0.25 * S;
            qy = (R(1,2) + R(2,1)) / S;
            qz = (R(1,3) + R(3,1)) / S;
            
        elseif (R(2,2) > R(3,3))
            S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; % S = 4 * qy
            qw = (R(1,3) - R(3,1)) / S;
            qx = (R(1,2) + R(2,1)) / S;
            qy = 0.25 * S;
            qz = (R(2,3) + R(3,2)) / S;
            
        else
            S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; % S = 4 * qz
            qw = (R(2,1) - R(1,2)) / S;
            qx = (R(1,3) + R(3,1)) / S;
            qy = (R(2,3) + R(3,2)) / S;
            qz = 0.25 * S;
        end
    end
    
    q = [qw, qx, qy, qz];
    q = q / norm(q);
end

