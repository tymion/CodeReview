function e2s_unit = earth2sunVector(t)
% Calculating vector from earth to sun for a given simulation time t
% earth2sun vector is expressed in ECI frame

global date;

% Julian Date
jd = 1721013.5 + 367 * date.year - floor(7/4 * (date.year + floor((date.month + 9) / 12))) ...
    + floor(275 * date.month / 9) + date.day + (60 * date.hour + date.min + (date.sec + t) / 60) / 1440;

% Number of Julian centuries from epoch J2000
T_uti = (jd - 2451545) / 36525;

sun_long = 280.460 + 36000.771 * T_uti; % Mean Sun longitude
sun_anom = 357.5277233 + 35999.05034 * T_uti; % Mean anomaly of the Sun

% Converting mean sun longitude and mean sun anomaly to the range <0,360>
% [deg]
sun_long = sun_long - floor(sun_long / 360) * 360;
sun_anom = sun_anom - floor(sun_anom / 360) * 360;

ecl_long = (sun_long + 1.914666471 * sin(sun_anom * pi / 180) + 0.019994643 * sin(2 * sun_anom * pi / 180)) * pi / 180; % Longitude of ecliptic [rad]
eps = (23.439291 - 0.0130042 * T_uti) * pi / 180; % Obliquity of the ecliptic [rad]

e2s_unit = [cos(ecl_long), cos(eps) * sin(ecl_long), sin(eps) * sin(ecl_long)]'; % earth to sun unit vector
e2s_unit = e2s_unit / vectorNorm(e2s_unit);
