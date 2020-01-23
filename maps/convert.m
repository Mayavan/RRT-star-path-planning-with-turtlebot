clear;
img = imread('map.pgm');
img_new = ones(size(img,1), size(img,2));
for i = 1: size(img,1)
    for j = 1:size(img,2)
        if (img(i,j) < 220)
            for ii = -4:4
                for jj = -4:4
                    if i+ii>0 && j+jj>0
                        img_new(i+ii,j+jj)=0;
                    end
                end
            end
        end
    end
end

imshow(img_new)
imwrite(img_new,'point_map.png')
    %0.1435 is radius from centre. 