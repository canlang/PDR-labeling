function [lat, lon, alt] = ned2geo(xn, lat0, lon0, alt0, a, e)
%  [lat, lon, alt] = ned2geo(xn, lat0, lon0, alt0, a, e)
%  Transforms position from the NED to geodetic frame
%
%   Input arguments:
%   xn -  Position in NED frame [3,1]
%   lat0, lon0, alt0 - Latitude, Longitude and Altitide of NED frame origin
%   a  -  Ellipsoid semi-major axis length, m
%   e  -  Ellipsoid eccentricity
%
%   Output arguments:
%   lat, lon, alt - Latitude, Longitude and Altitide in geodetic frame

xe0  = geo2ecef(lat0, lon0, alt0, a, e);
Cen = [...
    -sin(lat0)*cos(lon0) -sin(lat0)*sin(lon0)  cos(lat0)
              -sin(lon0)            cos(lon0)          0
    -cos(lat0)*cos(lon0) -cos(lat0)*sin(lon0) -sin(lat0)
    ];
xe = xe0 + Cen'*xn;
[lat, lon, alt] = ecef2geo(xe, a, e);
end