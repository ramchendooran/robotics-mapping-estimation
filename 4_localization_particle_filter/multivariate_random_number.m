mu = [0 0 0];
sigma = diag([0.1 0.1 0.1]);
R = chol(sigma);
z = (repmat(0,10,1) + randn(10,3)*R)';