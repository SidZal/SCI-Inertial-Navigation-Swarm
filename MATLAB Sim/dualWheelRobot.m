classdef dualWheelRobot
    
    properties
        r;
        d;
        dt;
        robotIndex;
        color;
        
    end
    
    methods(Static)
        function q_IC = randomIC
            q_IC = [(rand()-0.5)*4 (rand()-0.5)*4 (rand()-0.5)*4*pi];
        end
    end

    methods
        function obj = dualWheelRobot(simParams,robotIndex,color)
            obj.dt = simParams(1);
            obj.r = simParams(2);
            obj.d = simParams(3);
            obj.robotIndex = robotIndex;
            obj.color = color;
        end
        
        function q_kp1 = discreteModel(obj,q_k, wL_k, wR_k)

            x_k = q_k(1);
            y_k = q_k(2);
            theta_k = q_k(3);
            
            x_kp1 = x_k + obj.dt * (obj.r/2 * (wL_k+wR_k) * cos(theta_k));
            y_kp1 = y_k + obj.dt * (obj.r/2 * (wL_k+wR_k) * sin(theta_k));
            theta_kp1 = theta_k + obj.dt * (obj.r/obj.d * (wR_k - wL_k));
            
            q_kp1 = [x_kp1;y_kp1;theta_kp1];
        end

        function [wL,wR] = headingController(obj,theta_desired,q_k)
            kp = 0.5;
            theta_k = q_k(3);
            error = theta_desired - theta_k;
            wL = -kp*error;
            wR = kp*error;
        end
        
        function [wL,wR] = positionController(obj,q_desired,q_k)
            
            k_theta = 1;
            k_v = 1;

            x_k = q_k(1);
            y_k = q_k(2);
            theta_k = q_k(3);

            x_desired = q_desired(1);
            y_desired = q_desired(2);

            paramMatrix = [obj.r/2 obj.r/2;
                           -obj.r/obj.d obj.r/obj.d];

            vB = k_v * sqrt( (x_desired - x_k)^2 + (y_desired - y_k)^2 );
         
            dth = atan2((y_desired - y_k), (x_desired - x_k));
            tmp = unwrap([theta_k, dth]);
            theta_k = tmp(1);
            dth = tmp(2);
                
            theta_dot = k_theta*( dth - theta_k );

            temp = inv(paramMatrix) * [vB; theta_dot];

            if sqrt((x_desired - x_k)^2 + (y_desired - y_k)^2) <= 0.01
                temp = [0;0];
            end

            wL = temp(1);
            wR = temp(2);
        end

        function [wL,wR] = verticalController(obj,q_desired,q_k)
            vc = 1;
            k1 = -2;
            k2 = 2;
            
            x_desired = 0;
            theta_desired = pi/2;

            x_k = q_k(1);
            y_k = q_k(2);
            theta_k = q_k(3);

            v = vc;
            w = -k1*(x_k - x_desired) - k2*(theta_k-theta_desired);
            wR = (v + w * obj.d/2)/obj.r;
            wL = (v - w * obj.d/2)/obj.r;

        end

        function [wL,wR] = horizontalController(obj,q_desired,q_k)
            vc = 1;
            k1 = 2;
            k2 = 2;
            
            x_desired = 0;
            y_desired = 0;
            theta_desired = 0;

            x_k = q_k(1);
            y_k = q_k(2);
            theta_k = q_k(3);

            v = vc;
            w = -k1*(y_k - y_desired) - k2*(theta_k-theta_desired);
            wR = (v + w * obj.d/2)/obj.r;
            wL = (v - w * obj.d/2)/obj.r;
            
        end
        
        function [wL,wR] = MPCController(obj,args,solver,N,q_desired,q_k)

            x0 = [q_k(1);q_k(2);q_k(3)]; 
            xs = [q_desired(1);q_desired(2);q_desired(3)]; 
            
            u0 = zeros(N,2);
            
            args.p = [x0;xs];
            args.x0 = reshape(u0',2*N,1);
            sol = solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'p',args.p);
            u = reshape(full(sol.x)', 2, N)';
            wL = u(1,1);
            wR = u(1,2);
        end

        function plotCurrentPos(obj,q_k)

            q = quiver(q_k(1),q_k(2),cos(q_k(3)),sin(q_k(3)));
            q.ShowArrowHead = "on";
            q.Marker = "o";
            q.MarkerSize = 10;
            q.LineStyle = "-";
            q.LineWidth = 2;
            q.Color = obj.color;
            xlim([-10 10])
            ylim([-10 10])

        end

    end
end

