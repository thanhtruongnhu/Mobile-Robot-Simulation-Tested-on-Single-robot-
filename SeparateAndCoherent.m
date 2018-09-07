function [Vs_comp,Vc_comp] = SeparateAndCoherent(r_ij)
global r_a r_nm
r = norm(r_ij);
r_unit = r_ij/r;

if      r <= r_a  % Repulsive (Vung day)
    Vs_comp = -exp(-(r - r_a))*r_unit;
    Vc_comp = [0,0];
elseif  r >= r_nm % Attractive (Vung hut)
    Vc_comp = r_ij;
    Vs_comp = [0,0];
elseif  (r_a < r) && (r < r_nm) % No manilpulate
    Vc_comp = [0,0];
    Vs_comp = [0,0];
end

end
