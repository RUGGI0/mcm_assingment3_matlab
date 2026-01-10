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

            bTt = self.gm.getToolTransformWrtBase;

            %*% Cartesian error: notes 12/12
            
            r = bTg(1:3,4) - bTt(1:3,4); %*% CK: translational error as difference between positions

            bRt = bTt(1:3,1:3);
            bRg = bTg(1:3,1:3);

            R_err = bRg * bRt';  %*% CK: rotational error as relative orientation

            [h, theta] = RotToAngleAxis(R_err);  %*% Defined in 1st assignment

            rho = h * theta;

            error = [rho;r];

            delta = zeros(6);
            delta(1:3,1:3) = eye(3)*self.k_a;
            delta(4:6,4:6) = eye(3)*self.k_l;

            x_dot = delta*error;  %*% Definition given by 3rd assignmnet --> formula in es Q2.2
            
        end
    end
end

