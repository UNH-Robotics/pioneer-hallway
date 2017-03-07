clear variables ;
close all ;
low_res = csvread('utm_0.csv') ;
low_res = flipud(low_res') ; % transform to correct orientation

high_res = zeros(size(low_res,1)*2,size(low_res,2)*2) ;

for i = 1:size(low_res,1)
    for j = 1:size(low_res,2)
        if (low_res(i,j))
            high_res(2*i-1:2*i,2*j-1:2*j) = ones(2) ;
        end
    end
end

figure
imshow(high_res)
% saveas(gcf,'utm_0_inverted','fig') ;