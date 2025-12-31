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

        function bJi = getJacobianOfJointWrtBase(self, i)
            %% getJacobianOfJointWrtBase function
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint indnex ;

            % The function returns:
            % bJi
            
            %TO DO

            bJi = zeros(6,i);

            bTi = self.gm.getTransformWrtBase(i);
            bri = bTi(1:3,4);

            for j = 1:i
                
                bTj = self.gm.getTransformWrtBase(j);

                % axis z of joint j expressed in base frame
                % <taking into account eventual rotations of the joint>
                zj = bTj(1:3,3);

                if (self.gm.jointType(j) == 1)

                    JL = zj;
                    bJi(:,j) = [0; 0; 0; JL];

                end

                if (self.gm.jointType(j) == 0)
                    % position of link j relative to base frame
                    brj = bTj(1:3,4);
 
                    JA = zj; % angular jacobian
                    JL = cross(zj,bri - brj); % linear jacobian
                    % <JL simply that's how is the formula>
                    bJi(:,j) = [JA; JL];

                end
            end

        end

        function updateJacobian(self)
        %% updateJacobian function
        % The function update:
        % - J: end-effector jacobian matrix
            % TO DO

            self.J = self.getJacobianOfJointWrtBase(self.gm.jointNumber);
            
        end
    end
end

