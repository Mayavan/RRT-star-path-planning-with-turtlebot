function image = getmap()

image=zeros(151,251);

for xx=1:251
    for yy=1:151
        % xx-1 and yy-1 are used because we need to represent image from 
        % 0-150 and 0-250, while matlab index starts with 1. 
        image(yy,xx)=~checkObstacle([xx-1 yy-1]);
    end
end
image = 255 * repmat(double(image), 1, 1, 3);
end

