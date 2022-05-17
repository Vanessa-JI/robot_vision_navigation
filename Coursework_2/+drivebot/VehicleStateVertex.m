% The object state vertex:
% x(1) - x
% x(2) - y
% x(3) - theta

% Note that the oplus method has to be used to account for angular
% discontinuities

classdef VehicleStateVertex < g2o.core.BaseVertex
    
    properties(Access = protected)
        T;
    end
    
    methods(Access = public)
        function this = VehicleStateVertex(T)
            this=this@g2o.core.BaseVertex(3);
            this.T = T;
        end
        
        function T = time(this)
            T = this.T;
        end
        
        function oplus(this, update)
            %% Q1c %%%%%%%%%%%
            this.x = this.x + update; 

            % Wrap the angle to [-pi,pi]
            this.x(3) = g2o.stuff.normalize_theta(this.x(3));
            
%             warning('vehiclestatevertex:oplus:unimplemented', ...
%                 'Implement the rest of this method for Q1b.');
            
        end
    end
end