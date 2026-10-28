function [Ac, Bc] = get_linearized_model(psi_r, v_r, delta_r, L)
% Jacobian linearization of the kinematic bicycle model at (psi_r, v_r, delta_r).
%
% States  x = [X, Y, psi, v]
% Inputs  u = [delta, a]
%
% Continuous dynamics:  x_dot = Ac * (x - x_r) + Bc * (u - u_r)

Ac = [0, 0, -v_r*sin(psi_r),            cos(psi_r);
      0, 0,  v_r*cos(psi_r),            sin(psi_r);
      0, 0,  0,               tan(delta_r) / L;
      0, 0,  0,               0];

Bc = [0,                              0;
      0,                              0;
      v_r / (L * cos(delta_r)^2),    0;
      0,                              1];
end
