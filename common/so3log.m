function [omega,theta,axis] = so3log(R)

   %det(R)
    %theta = acos((trace(R) - 1) / 2);
    theta = acos(max(-1,min((trace(R)-1)/2,1)));

    if isreal(theta) == 0
        R = R / abs(det(R));
        theta = acos((trace(R) - 1) / 2);        
    end

    if abs(theta) < 1e-9
        B = 0.5;
        SO = (1 / (2)) * (R - R'); % =skew(omega)
        %iV = eye(3); % use directly skew of omega
        axis=[0,0,0];
        omega = [SO(3, 2) SO(1, 3) SO(2, 1)];
    else
        %A = sin(theta) / theta;
        invA = theta/sin(theta);
        % B = (1 - cos(theta)) / (theta * theta);
        SO = (invA/2) * (R - R'); % =skew(omega)
            
        %??
        % syms x real
        % A = sin(x)/x
        % B= (1-cos(x))/(x*x)
        % Q=1/(x*x)*(1 - A/2/B)
        % taylor(Q,x,0)
        %       x^4/30240 + x^2/720 + 1/12
        %Q = 1 / (theta^2) * (1 - A / 2 / B);
        %iV = eye(3) - 1/2 * SO + Q * SO^2; % use directly skew of omega
        omega = [SO(3, 2) SO(1, 3) SO(2, 1)];
        axis=[SO(3, 2) SO(1, 3) SO(2, 1)]/theta;
        %norm(omega)
        %theta
    end

    
