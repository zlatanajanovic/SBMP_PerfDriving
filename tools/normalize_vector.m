function v_norm = normalize_vector(v)

%v_norm = v-min(v);

if max(v)~=0
    v_norm = v./(max(v)+eps);
elseif all(v==0)
    v_norm = v;
else
    v_norm = abs(v)./(max(abs(v))+eps);
end