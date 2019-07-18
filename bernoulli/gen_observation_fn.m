function Z= gen_observation_fn(model,X,W)

%r/t observation equation

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
    P= X([1 3],:);
    P2= X([1 3],:)-[1000 0]';
    Z(1,:)= atan2(P(1,:),P(2,:));   
    Z(2,:)= atan2(P2(1,:),P2(2,:));   
    Z= Z+ W;
end