% Display the parameters of the RBF policy to the command window
% This way they only need to be copied and pasted into the Arduino code
function  writePolicyArd(policy)

nc = length(policy.p.targets);
Kxx = zeros(nc,nc);                         % initialize kernel matrix
sf = exp(policy.p.hyp(5));                  % signal variance
sn = exp(policy.p.hyp(6))*sf;               % noise variance
Lam = diag(1./exp(2*policy.p.hyp(1:4)));    % characteristic length-scales

% constructing kernel matrix
for ii=1:nc
    for jj=1:nc
        Kxx(ii,jj)=sf^2*kernel(policy.p.inputs(ii,:)',policy.p.inputs(jj,:)',Lam);
    end
end
alpha = (Kxx+sn^2*eye(nc))\policy.p.targets;
disp(['double alpha[10] = ' printArd(alpha') ';'])
disp(['double X[10][4] = ' printArd(policy.p.inputs) ';'])
disp(['double lam[4] = ' printArd(diag(Lam)') ';'])