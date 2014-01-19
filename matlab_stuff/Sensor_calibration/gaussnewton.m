
function retval =gaussnewton(D_std, beta_guess, n)
beta_std = gn(D_std, beta_guess, n);

% 
% sp = normalize_samples(D_std, beta);
% sp_std = normalize_samples(D_std, beta_std);
% 
% err = sp - sp_std;
% mag = row_magnitudes(sp);
% mag_std = row_magnitudes(sp_std);
% 
% mag_err = row_magnitudes(err);
% for i = 1:14
%     retval(i,1) = mag_err(i);
%     retval(i,2) = mag(i) - mag_std(i);
%     retval(i,3) = sqrt(retval(i,1)^2 - retval(i,2)^2);
% end
retval = beta_std;
 return


function retval = residuals(adxl_data, beta)
retval = ones(length(adxl_data),1);
for i = 1:length(adxl_data)
    for j = 1:3
        retval(i) = retval(i) - (beta(3+j)^2*(adxl_data(i,j) - beta(j))^2);
    end
end
return

function retval = jacobian(adxl_data, beta)
retval = zeros(length(adxl_data),6);
for i = 1:length(adxl_data)
    for j = 1:3
        retval(i,j) = 2*beta(3+j)^2*(adxl_data(i,j) - beta(j));
        retval(i, j+3) = -2*beta(3+j)*(adxl_data(i,j) - beta(j))^2;
    end
end
return

function retval = gn_step(adxl_data, beta)
r = residuals(adxl_data, beta);
J = jacobian(adxl_data, beta);
JS = J'*J;
delta = JS \ (J'*r);
retval = beta - delta';
norm(residuals(adxl_data, retval));
return

function retval = gn(adxl_data, beta, n)
A = adxl_data;
change = 100;
step = 0;

while change > 0.000000001 && step < n
    oldbeta = beta;
    beta = gn_step(A,beta);
    change = 0;
    for i = 1:6
        change = change+ abs((beta(i) - oldbeta(i))/oldbeta(i));
    end
step = step+1;
end
retval = beta;

return



function retval = normalize_samples(adxl_data, beta)
retval = (adxl_data - ones(length(adxl_data),3)*diag([beta(1) beta(2) beta(3)]))*diag([beta(4) beta(5) beta(6)]);
return

function retval = row_magnitudes(data)
retval = sqrt(diag(data*data'));
return

