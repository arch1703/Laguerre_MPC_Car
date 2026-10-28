function [Phi, Theta, affine] = build_prediction_matrices(A, B, c, Np, Nc)
% Lift a time-varying affine model, holding the final input after Nc.
nx = size(A,1);
nu = size(B,2);
Phi = zeros(Np*nx,nx);
Theta = zeros(Np*nx,Nc*nu);
affine = zeros(Np*nx,1);
F = eye(nx);
G = zeros(nx,Nc*nu);
g = zeros(nx,1);
for i = 1:Np
    Ai = A(:,:,min(i,size(A,3)));
    Bi = B(:,:,min(i,size(B,3)));
    ci = c(:,min(i,size(c,2)));
    F = Ai*F;
    G = Ai*G;
    cols = (min(i,Nc)-1)*nu+(1:nu);
    G(:,cols) = G(:,cols)+Bi;
    g = Ai*g+ci;
    rows = (i-1)*nx+(1:nx);
    Phi(rows,:) = F;
    Theta(rows,:) = G;
    affine(rows) = g;
end
end
