function [A, B, C, G] = formulteDynamics(...
    m_1, m_2, L_1, L_2, r_c1, r_c2, I_zz1, I_zz2, I_xx, I_yy)
    syms q_1 q_2

    g = -9.81; %m/s^2

    I_1 = [ I_xx, 0, 0;
            0, I_yy, 0;
            0, 0, I_zz1];

    I_2 = [ I_xx, 0, 0;
            0, I_yy, 0;
            0, 0, I_zz2];
        

    % DH Parameters
    DH = [  L_1, 0, 0, q_1;
            L_2, 0, 0, q_2;
            0, pi/2, 0, pi/2];


    % Jacobian matricies
    J_v1 = [-r_c1*sin(q_1), 0;
            r_c1*cos(q_1), 0;
            ];

    J_w1 = [0, 0;
            0, 0;
            1, 0];

    J_v2 = [-L_1*sin(q_1) - r_c2*sin(q_1 + q_2), -r_c2*sin(q_1 + q_2);
            L_1*cos(q_1) + r_c2*cos(q_1 + q_2), r_c2*cos(q_1 + q_2);
            ];

    J_w2 = [0, 0;
            0, 0;
            1, 1];


    % Dynamics matricies 

    % Inertia
    A = simplify(m_1.*transpose(J_v1)*J_v1 + transpose(J_w1)*I_1*J_w1 + ...
        m_2.*transpose(J_v2)*J_v2 + transpose(J_w2)*I_2*J_w2);


    % Christoffel Symbol function
    Q = [q_1; q_2];

    aijk = @(i, j, k)( ...
                    diff(A(i,j), Q(k)) ...
                    );

    b = @(i, j, k)( ...
                    1/2 * (aijk(i,j,k) + aijk(i,k,j) - aijk(j,k,i)) ...
                    );


    % Coriolis
    B = 2.*[b(1,1,2);
            b(2,1,2)];

    % Centrifugal
    C = [b(1,1,1), b(1,2,2);
         b(2,1,1), b(2,2,2)];

    % Gravity
    y = [0;1]; % y hat
    G = -[transpose(J_v1)*m_1*g*y + transpose(J_v2)*m_2*g*y];   

end