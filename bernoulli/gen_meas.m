function meas= gen_meas(model,truth)

%variables
meas.K= truth.K;
% no. of sensors
meas.S = 2;
meas.Z= cell(truth.K, meas.S);
meas.sensor_pos = [0 0; 1000 0]';

%generate measurements
for s = 1:meas.S
	for k=1:truth.K
	    if truth.N(k) > 0
	        %detected target indices
			idx= find( rand(truth.N(k),1) <= model.P_D );                                            
	        %single target observations if detected
			meas.Z{k, s}= gen_observation_fn(model,truth.X{k}(:,idx),'noise', meas.sensor_pos(:,s));                           
	    end
	    %number of clutter points
		N_c= poissrnd(model.lambda_c);                                                               
	    %clutter generation
		C= repmat(model.range_c(:,1),[1 N_c])+ diag(model.range_c*[ -1; 1 ])*rand(model.z_dim,N_c);  
	    %measurement is union of detections and clutter
		meas.Z{k, s}= [ meas.Z{k, s} C ];                                                                  
	end
end
