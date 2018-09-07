%DIFFANGLE Take difference of two angles and unwrap it.
% A = DIFFANGLE(A1,A2) determines the minimal difference
% A = A1-A2 between two angles A1 and A2. If either A1 or A2
% is Inf, Inf is returned.
%
% See also NORMANGLE.

% v.1.0, Dec. 2003, Kai Arras, CAS-KTH

function delta = diffangle(a1,a2)

if (a1 < Inf) && (a2 < Inf)
    % Normalize angles a1 and a2::: Vi input la tu 0-360 nen can phai bo Normangle.m và còn boi vi khi ?e ham Normangle.m se lam sai gia tri that cua goc input neu no co gia tri lon hon 180deg (pi).
%     a1 = normangle(a1);  %rad
%     a2 = normangle(a2);  %rad
    % Take difference and unwrap ::: brings angle delta to the range (-pi .. pi]
    delta = a1 - a2;
    if a1 > a2
        while delta > pi
            delta = delta - 2*pi;
        end
    elseif a2 > a1
        while delta < -pi
            delta = delta + 2*pi;
        end
    end
else
    delta = Inf;
end
end