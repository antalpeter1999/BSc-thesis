function [input, output, L, latent] =  readArdData(filename, dt, H, cost)

% High level steps:
% Send policy parameters to Arduino
% Read measured states
% Save measurements

% For now: only read and convert measurements
fileID = fopen(filename,'r');
formatSpec = '%f\n%f\n%f\n%f\n%f';
sizeA = [3 Inf];
Meas = fscanf(fileID,formatSpec,sizeA)';
% Meas(:,5) = 2*Meas(:,5);

m_phi = Meas(1:H+2,1)/1000;
m_x = Meas(1:H+2,2)/10000;
m_dphidt = (m_phi(2:H+2) - m_phi(1:H+1)) / dt; % backward Euler method
m_dxdt = (m_x(2:H+2) - m_x(1:H+1)) / dt;
m_u = Meas(2:H+1,3) / 100;

% maximum applied voltage cannot be larger than +-40 V
for i = 1:H
    if m_u(i) > 40
        m_u(i) = 40;
    elseif m_u(i) < -40
        m_u(i) = -40;
    end
end

input = [m_x(2:H+1) m_dxdt(1:H) m_dphidt(1:H) m_phi(2:H+1) m_u];
output = [m_x(3:H+2) m_dxdt(2:H+1) m_dphidt(2:H+1) m_phi(3:H+2)];
latent = input;

L = zeros(H,1);
for i = 1:H
    L(i) = cost.fcn(cost,input(i,1:4)',zeros(4));
end