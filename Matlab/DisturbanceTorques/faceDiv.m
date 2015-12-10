function face = faceDiv()
% Dividing the satellite shape into 6 flat plates
% ***************************************************
% ASSUMPTIONS:
% 1. avoiding concave shape by modelling the satellite as a extruded
% trapeze;
% 2. center of pressure for each face assumed to be in a geometrical center
% 3. s stands for area;
% 4. v stands for normal unit vector in satellite frame to each face
% 5. cp stands for position of center of pressure for each face with
% respect to the center of mass
% 6. diffuse stands for diffuse reflection coefficient
% 7. spec stands for specular reflection coefficient
% 8. absorpt stands for absorption coefficient
% 9. diff + spec + abs = 1
% 10. almn stands for aluminium
% 11. mli stands for MLI
% 12. arr stands for solar arrays
% 13. each coefficient calculated as weighted mean depending of area ratio
% of each material
%
% face 1 normal = +Xb
% face 2 normal = -Zb
% face 3 normal = +Zb
% face 4 normal = -Xb
% face 5 normal = face 4 normal rotated 45 deg about +Zb axis
% face 6 normal = face 4 normal rotated -45 deg about +Zb axis
%
% b index refers to satellite body frame
% ***************************************************

global r_cm;

almn_abs = 0.06;
almn_spec = 0.1;
almn_diff = 0.84;

mli_abs = 0.05;
mli_spec = 0;
mli_diff = 0.95;

arr_abs = 0.92;
arr_spec = 0.04;
arr_diff = 0.04;

face.v(:,1) = [1 0 0]';
face.s(1) = 0.2 * 0.3;
face.cp(:,1) = [0.05 0 0]' - r_cm;
face.diffuse(1) = 0.9*arr_diff + 0.1*almn_diff;
face.spec(1) = 0.9*arr_spec + 0.1*almn_spec;
face.absorpt(1) = 0.9*arr_abs + 0.1*almn_abs;

face.v(:,2) = [0 0 1]';
face.s(2) = 0.1 * 0.1;
face.cp(:,2) = [0 0 0.1]' - r_cm;
face.diffuse(2) = almn_diff;
face.spec(2) = almn_spec;
face.absorpt(2) = almn_abs;

face.v(:,3) = [0 0 -1]';
face.s(3) = face.s(2);
face.cp(:,3) = [0 0 -0.1]' - r_cm;
face.diffuse(3) = face.diffuse(2);
face.spec(3) = face.spec(2);
face.absorpt(3) = face.absorpt(2);

face.v(:,4) = [-1 0 0]';
face.s(4) = 0.1 * 0.2;
face.cp(:,4) = [-0.05 0 0]' - r_cm;
face.diffuse(4) = 0.5*arr_diff + 0.5*almn_diff;
face.spec(4) = 0.5*arr_spec + 0.5*almn_spec;
face.absorpt(4) = 0.5*arr_abs + 0.5*almn_abs;

face.v(:,5) = [-sqrt(2)/2 -sqrt(2)/2 0]';
face.s(5) = sqrt(2) * 0.1 * 0.2;
face.cp(:,5) = [0 -0.1 0]' - r_cm;
face.diffuse(5) = 0.5*mli_diff + 0.45*arr_diff + 0.05*almn_diff;
face.spec(5) = 0.5*mli_spec + 0.45*arr_spec + 0.05*almn_spec;
face.absorpt(5) = 0.5*mli_abs + 0.45*arr_abs + 0.05*almn_abs;

face.v(:,6) = [-sqrt(2)/2 sqrt(2)/2 0]';
face.s(6) = face.s(5);
face.cp(:,6) = [0 0.1 0]' - r_cm;
face.diffuse(6) = face.diffuse(5);
face.spec(6) = face.spec(5);
face.absorpt(6) = face.absorpt(5);