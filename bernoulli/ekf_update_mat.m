function [H,U]= ekf_update_mat(model,mu)

 p= mu([1 3],:);
 p2= mu([1 3],:)-[1000 0]';
 mag= p(1)^2 + p(2)^2;
 mag2= p2(1)^2 + p2(2)^2;
 H= [ p(2)/mag  0  -p(1)/mag  0; ...
  p2(2)/mag2  0  -p2(1)/mag2  0];
 U= eye(2);
 