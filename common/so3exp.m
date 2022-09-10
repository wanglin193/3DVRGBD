        function  rotationMatrix = so3exp(rotationVector)
            rotationVector = rotationVector';
            theta = norm(rotationVector);
            if theta < 1e-6
                rotationMatrix = eye(3,'like',rotationVector);
                return;
            end
            u = rotationVector./ theta;
            u = u(:);
            A = [0,-u(3),u(2);
                u(3),0,-u(1);
                -u(2),u(1),0];
            B = u * u';
            alpha = cos(theta);beta = sin(theta);gamma = 1 - alpha;
            rotationMatrix = eye(3)*alpha + beta*A + gamma*B;
        end