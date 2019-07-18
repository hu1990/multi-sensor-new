function Z= gen_observation_fn(model,X,W, sensor_pos)
% X: array(state, target)

if ~isnumeric(W)
    if strcmp(W,'noise')
        W= model.D*randn(size(model.D,2),size(X,2));
    elseif strcmp(W,'noiseless')
        W= zeros(size(model.D,1),size(X,2));
    end
end

if isempty(X)
    Z= [];
else %modify below here for user specified measurement model
	Z = model.hx(X, sensor_pos) + W;
end
