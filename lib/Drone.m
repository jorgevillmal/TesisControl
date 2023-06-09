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

        psi_d
        dot_psi_d
        e_f
        

        %% Body Frame

        euler

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
        dotR
        %% Input
        u
        f
        tau

    end

    %% Control Problem

    properties
        % Position-tracking
        
        x_des       % desired position
        x_err       % error position
        x_err_prev
        x_err_sum

        v_des       % desired velocity
        v_err       % error position
        v_err_prev
        v_err_sum

        dotV_des    % desired acceleration
        dotV_err    % error acceleration
        dotV_err_prev
        dotV_err_sum


        R_des % desired attitude
        dotR_des
        omega_des   % desired omega
    end
      
    properties
        % Position Controller

        k
        k_x
        K_f
    end

    properties
        %control law
        
        T
        dotE_f
        eta
        ddx_d
    end

%% METHODS
    methods
    %% CONSTRUCTOR
    function obj = Drone(params, initStates, initInputs,simTime)
            obj.g = 9.81;
            obj.t = 0.0;
            obj.dt = 0.1;
            obj.tf = simTime;

            obj.m = params('mass');
            obj.b = params('gyro');
            obj.M = params('inertialMatrix');
            
            obj.x = initStates('position');
            obj.v = initStates('velocity');
            obj.R = initStates('attitude');
            euler = rotm2eul(obj.R);
            obj.phi = euler(3);
            obj.theta = euler(2);
            obj.psi = euler(1);
            obj.omega = initStates('angularV');
            
            obj.s = [obj.x; obj.v; obj.omega];
            obj.stateR = obj.R;
            obj.dotS = zeros(9,1);
            obj.dotR = zeros(3);

            obj.u = initInputs;
            obj.f = obj.u(1);
            obj.tau = obj.u(2:4);

            % error

            obj.x_err = 0.0;
            obj.x_err_prev = 0.0;
            obj.x_err_sum = 0.0;

            obj.v_err = 0.0;
            obj.v_err_prev = 0.0;
            obj.v_err_sum = 0.0;

            obj.dotV_err = 0.0;
            obj.dotV_err_prev = 0.0;
            obj.dotV_err_sum = 0.0;

            obj.R_des = zeros(3);
            obj.omega_des = zeros(3,1);

            % position controller

            obj.k = 4;
            obj.k_x = 0.1;
            obj.K_f = eye(3);

            % control law

            obj.psi_d = pi/4;
            obj.dot_psi_d = 0;
            obj.e_f = zeros(3,1);

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
            obj.dotS(4:6) = 1 / obj.m *([0; 0; -obj.g * obj.m] + wRb * [0; 0; obj.f]);

            
            % Angular velocity
            obj.dotR = obj.R * wedgeMap(obj.omega);

            % Angular Acceleration
            obj.dotS(7:9) = (obj.M) \ (cross(obj.omega,obj.M*obj.omega) + obj.tau);
            
           
        end

        function ControlStatmet(obj)

            e_z = [0, 0, 1]';

            obj.v_err = obj.eta - obj.k_x * obj.x_err - Tanh(obj.e_f);

            % CONTROL  STATEMENT

            obj.dotS(1:3) = obj.v_err;

            % obj.dotS(4:6) = 1 / obj.m * (-obj.m * obj.g * e_z - obj.m * obj.ddx_d ...
            %     + obj.f * obj.R_des * e_z + obj.f * (obj.R - obj.R_des) * e_z);

            obj.dotS(4:6) = (1/obj.m) *(-obj.m * (obj.k - obj.k_x) * obj.eta ...
                + obj.k*Tanh(obj.e_f) - obj.k_x^2*obj.x_err);

            % Angular velocity
            obj.dotR = obj.R * wedgeMap(obj.omega);

            % Angular Acceleration
            obj.dotS(7:9) = (obj.M) \ (cross(obj.omega,obj.M*obj.omega) + obj.tau);
            
         end

    %% PREDICT NEXT DRONE STATE
        function obj = UpdateState(obj)
            
            obj.t = obj.t + obj.dt;
            
            % Find(update) the next state of obj.X
            % obj.EvalEOM();
            obj.ControlStatmet();
            obj.s = obj.s + obj.dotS.*obj.dt;
            obj.stateR = obj.R + obj.dotR.*obj.dt;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.e_f = obj.e_f + obj.dotE_f * obj.dt;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            obj.x = obj.s(1:3);
            obj.v = obj.s(4:6);
            
            obj.R = obj.stateR;
            obj.euler = rotm2eul(obj.R); 
            obj.phi = obj.euler(3);
            obj.theta = obj.euler(2); 
            obj.psi = obj.euler(1);

            obj.omega = obj.s(7:9);

            

            


            %%%%%%%%%%%%%%%%%%%%% Measure %%%%%%%%%%%%%%%%%%%%%%%%%

%             obj.omega(1) = obj.omega(1) + 0.2;
%             obj.omega(2) = obj.omega(2) + 0.1;
%             obj.omega(3) = obj.omega(3) - 0.1;
            
        end
        
    %% CONTROLLER
    function obj = PositionCtrl(obj, pos_des)
             
             
              e_z = [0, 0, 1]';
              c_d = [cos(obj.psi_d), sin(obj.psi_d), 0]';


             %%%%%%%%%%%%%%%%%%%   Position   %%%%%%%%%%%%%%%%%%%%%%

             obj.x_des = pos_des;


             %%%%%%%%%%%%%%%%%%%   Velocity   %%%%%%%%%%%%%%%%%%%%%%

             % obj.v_des = gradient(obj.x_des);
             obj.v_des = [gradient(obj.x_des(1)), gradient(obj.x_des(2)), ...
                 gradient(obj.x_des(3))]';

             %%%%%%%%%%%%%%%%%%% Acceleration %%%%%%%%%%%%%%%%%%%%%%

             %obj.dotV_des = gradient(obj.v_des);

             obj.dotV_des = [gradient(obj.v_des(1)), gradient(obj.v_des(2)), ...
                 gradient(obj.v_des(3))]';


             obj.ddx_d = obj.dotV_des;


             obj.x_err = obj.x - obj.x_des;
             obj.v_err = obj.v - obj.v_des;


             % control law

             obj.eta = obj.v_err + obj.k_x * obj.x_err + Tanh(obj.e_f);

             obj.dotE_f = Cosh(obj.e_f)^2 * (-obj.K_f * Tanh(obj.e_f) + obj.k_x^2 ...
                 * (1-(1/obj.m))* obj.x_err - obj.k * obj.eta);

             obj.T = obj.m * obj.g * e_z + obj.m * obj.ddx_d + (obj.k * eye(3) ...
                   + obj.m * (obj.k_x * eye(3) + obj.K_f)) * Tanh(obj.e_f);


             obj.f = norm(obj.T);


             % attitude trajectory

             c_3 = obj.T/norm(obj.T);
             c_2 = cross(c_3,c_d)/norm(cross(c_3,c_d));
             c_1 = cross(c_2,c_3);

             obj.R_des = [c_1, c_2, c_3];
             obj.dotR_des = gradient(obj.R_des);

             obj.omega_des = veeMap(obj.R_des'*obj.dotR_des);

             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % obj.u(1) = obj.m*obj.g;
             % % obj.u(1) = 0.0;
             % obj.u(2) = 0.0;   % roll(phi)    X
             % obj.u(3) = 0.0;   % pith(theta)  Y
             % obj.u(4) = 0.0;   % yaw(psi)     Z
             % 
             % 
             % 
             % obj.f = obj.u(1);
             % obj.tau = obj.u(2:4);
         end

         
    end
end
