function [y] = smooth_sign(x, x_band)

    if x < -x_band
        y = -1;
    elseif x > x_band
        y = 1;
    else
        div = (x + x_band)/(2*x_band);
        y = -1 + (2*div*div)*(2-(x/x_band));
    end
        
end

