function solartorq = solarTorque(earth2sunVector, q, eclipse, face)
% Calculate torque due to solar radiation pressure,
% torque is expressed in satellite reference frame
% *****************************************************
% ASSUMPTIONS:
% 1. condition for eclipse for cylindrical approximation, radius of earth
% extended (up to 20 km) accounting for penumbra, spherical earth
% 2. solar constant assumed to be average value at the solar maximum activity;
% 3. rapid fluctuations of 5 W/m2 taken into account as pseudorandom uniformly
% distributed number added to the average value;
% 4. torque calculated using division of the satellite into separate faces
% *****************************************************

if eclipse == 1
    solartorq = 0;
    return
end

A_i2s = q2m(q);
unitsat = A_i2s * earth2sunVector;

c = 299792458; % speed of light [m/s]
sunconst = 1363 + 1e-1 * (randi(2 * 5/1e-1 + 1) - 5/1e-1 - 1); % solar constant [W/m2]
sunpress = sunconst / c; % solar pressure

solartorq = 0;
for i = 1:length(face.s)
    cos_i = face.v(:,i)' * unitsat;
    force_i = - sunpress * face.s(i) * (2 * (face.diffuse(i)/3 + face.spec(i) * cos_i)*face.v(:,i) + (1 - face.spec(i))*unitsat) * max(cos_i, 0); % [N]
    solartorq_i = skew(face.cp(:,i)) * force_i;
    
    solartorq = solartorq + solartorq_i;
end