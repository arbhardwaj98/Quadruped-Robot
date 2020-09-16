function [val, d] = signPoints(pt, v1, v2)

    val = (pt(1) - v2(1)) * (v1(2) - v2(2)) - (v1(1) - v2(1)) * (pt(2) - v2(2));
    a = v1 - v2;
    b = pt - v2;
    a = [a, 0];
    b = [b, 0];
    d = norm(cross(a,b)) / norm(a);
    
end