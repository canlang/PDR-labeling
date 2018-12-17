 rgb = imread('peppers.png'); 
 imshow(rgb); 
 I = rgb2gray(rgb); 
 hold on 
 % Save the handle for later use 
 h = imshow(I); 
 hold off
 %%
 [M,N] = size(I); 
 block_size = 50; 
 P = ceil(M / block_size); 
 Q = ceil(N / block_size); 
 alpha = checkerboard(block_size, ... 
     P, Q) > 0; 
 alpha = alpha(1:M, 1:N); 
 set(h, 'AlphaData', alpha);
 
