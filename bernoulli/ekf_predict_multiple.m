function [m_predict,P_predict] = ekf_predict_multiple(model,m,P)    

plength= size(m,2);

m_predict = zeros(size(m));
P_predict = zeros(size(P));

for idxp=1:plength
    [m_temp,P_temp] = ekf_predict_single(model,m(:,idxp),P(:,:,idxp));
    m_predict(:,idxp) = m_temp;
    P_predict(:,:,idxp) = P_temp;
end

function [m_predict,P_predict] = ekf_predict_single(model,m,P)

F = model.F;

m_predict = F * m;
P_predict = F * P * F' + model.Q;
