classdef Drone < handle
    
%% MEMBERS    
    properties
        g               % gravity acceleration
        t               % start time
        dt              % derivative with respect to time
        tf              % final time
        
        %% Initial Condition

        x              % position
        v              % velocity
        R              % actitude
        omega          % angular velocity
        

        %% Body Frame

        phi
        psi
        theta

        %% Parameter
        m              % mass 
        b              % gyro bias
        M              % inertial matrix
        
        %% State
        s
        stateR
        dotS
        DotR
        %% Input
        u
        f
        tau

    end
      
%% METHODS
    methods
    %% CONSTRUCTOR
    function obj = Drone(params, initStates, initInputs,simTime)
            obj.g = 9.81;
            obj.t = 0.0;
            obj.dt = 0.01;
            obj.tf = simTime;

            obj.m = params('mass');
            obj.b = params('gyro');
            obj.M = params('inertialMatrix');
            
            obj.x = initStates('position');
            obj.v = initStates('velocity');
            obj.R = initStates('attitude');
            [obj.phi, obj.theta, obj.psi] = Rot2RPY_ZXY(obj.R);
            obj.omega = initStates('angularV');
            

            % obj.s = [obj.x;obj.v;obj.omega];
            obj.s = [obj.x; obj.v; obj.omega];
            obj.stateR = obj.R;
            obj.dotS = zeros(9,1);
            obj.DotR = zeros(3);

            
            obj.u = initInputs;
            obj.f = obj.u(1);
            obj.tau = obj.u(2:4);
        end
        
    %% RETURNS DRONE STATE
        function state = GetState(obj)
            state.s = obj.s;
            state.R = obj.R;

        end
        
    %% STATE SPACE (DIFFERENTIAL) EQUATIONS: INCOMPLETE!
        function obj = EvalEOM(obj)
          
            % Body to Worl Rotation Matrix
            wRb = obj.R';

            % Translational Motions
            obj.dotS(1:3) = obj.v;
            obj.dotS(4:6) = 1 / obj.m *([0; 0; -obj.g * obj.m] + obj.R * [0; 0; obj.f]);

            
            % Angular velocity
            obj.DotR = obj.R*wedgeMap(obj.omega);

            % Angular Acceleration
            obj.dotS(7:9) = obj.M\ (wedgeMap(obj.M*obj.omega)*obj.omega + obj.tau);

           
        end

    %% PREDICT NEXT DRONE STATE
        function obj = UpdateState(obj)
            obj.t = obj.t + obj.dt;
            
            % Find(update) the next state of obj.X
            obj.EvalEOM();
            obj.s = obj.s + obj.dotS.*obj.dt;
            obj.stateR = obj.R + obj.DotR.*obj.dt;
            
            obj.x = obj.s(1:3);
            obj.v = obj.s(4:6);
            obj.omega = obj.s(7:9);
            obj.R = obj.stateR;
            [obj.phi, obj.theta, obj.psi] = Rot2RPY_ZXY(obj.R);
            
        end
        
    %% CONTROLLER
         function obj = AttitudeCtrl(obj, refSig)
			
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.u(1) = obj.m*obj.g;
            %obj.u(1) = 0;
            obj.u(2) = 0;   % pith(theta)  Y
            obj.u(3) = 1;   % roll(phi)    X
            obj.u(4) = 0;   % yaw(psi)     Z

            obj.f = obj.u(1);
            obj.tau = obj.u(2:4);
         end
    end
end
