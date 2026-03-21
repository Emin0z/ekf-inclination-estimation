classdef MyFunctions
    % EKF Helper Functions for IMU-based Attitude Estimation

    methods (Static)

        %% ================= Dynamic Model =================
        function [F, x_next] = DynamicModel(x, w, dt)

            % Angular velocity
            wx = w(1); wy = w(2); wz = w(3);

            % Quaternion
            qw = x(1); qx = x(2); qy = x(3); qz = x(4);

            % Gyro bias
            Bx = x(5); By = x(6); Bz = x(7);

            % Bias-compensated angular rate
            wxb = wx + Bx;
            wyb = wy + By;
            wzb = wz + Bz;

            % -------- State Propagation --------
            x_next = [
                qw - 0.5*dt*(qx*wxb + qy*wyb + qz*wzb);
                qx + 0.5*dt*(qw*wxb - qz*wyb + qy*wzb);
                qy + 0.5*dt*(qw*wyb + qz*wxb - qx*wzb);
                qz + 0.5*dt*(qx*wyb - qy*wxb + qw*wzb);
                Bx;
                By;
                Bz
            ];

            % -------- Jacobian (F matrix) --------
            F = [
                1, -0.5*dt*wxb, -0.5*dt*wyb, -0.5*dt*wzb, -0.5*dt*qx, -0.5*dt*qy, -0.5*dt*qz;
                0.5*dt*wxb, 1, 0.5*dt*wzb, -0.5*dt*wyb, 0.5*dt*qw, -0.5*dt*qz, 0.5*dt*qy;
                0.5*dt*wyb, -0.5*dt*wzb, 1, 0.5*dt*wxb, 0.5*dt*qz, 0.5*dt*qw, -0.5*dt*qx;
                0.5*dt*wzb, 0.5*dt*wyb, -0.5*dt*wxb, 1, -0.5*dt*qy, 0.5*dt*qx, 0.5*dt*qw;
                0,0,0,0,1,0,0;
                0,0,0,0,0,1,0;
                0,0,0,0,0,0,1
            ];
        end


        %% ================= Process Noise =================
        function Q = Qupdate(sigma, x, dt)

            qw = x(1); qx = x(2); qy = x(3); qz = x(4);

            Sigma = diag(sigma);

            Wt = [
                -0.5*dt*qx, -0.5*dt*qy, -0.5*dt*qz, -0.5*dt*qx, -0.5*dt*qy, -0.5*dt*qz;
                 0.5*dt*qw, -0.5*dt*qz,  0.5*dt*qy,  0.5*dt*qw, -0.5*dt*qz,  0.5*dt*qy;
                 0.5*dt*qz,  0.5*dt*qw, -0.5*dt*qx,  0.5*dt*qz,  0.5*dt*qw, -0.5*dt*qx;
                -0.5*dt*qy,  0.5*dt*qx,  0.5*dt*qw, -0.5*dt*qy,  0.5*dt*qx,  0.5*dt*qw;
                 0,0,0,1,0,0;
                 0,0,0,0,1,0;
                 0,0,0,0,0,1
            ];

            Q = Wt * Sigma * Wt.';
        end


        %% ================= Measurement Model =================
        function [H, h] = MeasurmentModel(x, g)

            q = x(1:4);

            % -------- Expected measurement (gravity in body frame) --------
            h = [
                g(2)*(2*q(1)*q(4) + 2*q(2)*q(3)) ...
              - g(1)*(2*q(3)^2 + 2*q(4)^2 - 1) ...
              - g(3)*(2*q(1)*q(3) - 2*q(2)*q(4));

                g(3)*(2*q(1)*q(2) + 2*q(3)*q(4)) ...
              - g(1)*(2*q(1)*q(4) - 2*q(2)*q(3)) ...
              - g(2)*(2*q(2)^2 + 2*q(4)^2 - 1);

                g(1)*(2*q(1)*q(3) + 2*q(2)*q(4)) ...
              - g(3)*(2*q(2)^2 + 2*q(3)^2 - 1) ...
              - g(2)*(2*q(1)*q(2) - 2*q(3)*q(4))
            ];

            % -------- Jacobian H --------
            H = [
                2*( g(2)*q(4) - g(3)*q(3) ),  2*( g(2)*q(3) + g(3)*q(4) ), ...
                2*( g(2)*q(2) - 2*g(1)*q(3) - g(3)*q(1) ), ...
                2*( g(2)*q(1) - 2*g(1)*q(4) + g(3)*q(2) ), 0,0,0;

                2*( g(3)*q(2) - g(1)*q(4) ), ...
                2*( g(3)*q(1) + g(1)*q(3) - 2*g(2)*q(2) ), ...
                2*( g(3)*q(4) + g(1)*q(2) ), ...
                2*( g(3)*q(3) - g(1)*q(1) - 2*g(2)*q(4) ), 0,0,0;

                2*( g(1)*q(3) - g(2)*q(2) ), ...
                2*( g(1)*q(4) - 2*g(3)*q(2) - g(2)*q(1) ), ...
                2*( g(1)*q(1) - 2*g(3)*q(3) + g(2)*q(4) ), ...
                2*( g(1)*q(2) + g(2)*q(3) ), 0,0,0
            ];
        end


        %% ================= Quaternion to Rotation =================
        function R = q2R(q)

            qw = q(1); qx = q(2); qy = q(3); qz = q(4);

            R = [
                qw^2 + qx^2 - qy^2 - qz^2, 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy);
                2*(qx*qy + qw*qz), qw^2 + qy^2 - qx^2 - qz^2, 2*(qy*qz - qw*qx);
                2*(qx*qz - qw*qy), 2*(qw*qx + qy*qz), qw^2 + qz^2 - qx^2 - qy^2
            ];
        end


        %% ================= Attitude Extraction =================
        function [roll, pitch, yaw] = R2RollPitch(R)

            roll  = asin(R(3,2));
            pitch = -atan(R(3,1) / R(3,3));
            yaw   = -atan(R(1,2) / R(2,2));
        end

    end
end
