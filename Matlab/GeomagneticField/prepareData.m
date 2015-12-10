clc
clear

coeffs = load('data.txt');

i = 1;

for n = 1:13
    for m = 0:n
        
        if m == 0
            index = n^2;
            g_nm = coeffs(index, 1);
            g_nm_deriv = coeffs(index, 2);
            h_nm = 0;
            h_nm_deriv = 0;
        else
            index = n^2 + 2*m - 1;
            g_nm = coeffs(index, 1);
            g_nm_deriv = coeffs(index, 2);
            h_nm = coeffs(index+1, 1);
            h_nm_deriv = coeffs(index+1, 2);
        end

        A(i,:) = [g_nm, g_nm_deriv, h_nm, h_nm_deriv];
        i = i + 1;
    end
end

save igrf11coeffs.txt A -ascii