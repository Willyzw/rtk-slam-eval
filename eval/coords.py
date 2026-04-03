"""
Coordinate conversion utilities for RTK-SLAM evaluation.

Ellipsoid conventions:
  enu_to_latlon  : WGS84  (a=6378137, 1/f=298.257223563)
  latlon_to_utm  : GRS80 / ETRS89  (a=6378137, 1/f=298.257222101)

Geoid corrections applied here (Geoid46 approx for Stuttgart area):
  ENU → UTM height : enu[2] -= 48.22   # slam frame offset
  GNSS → UTM height: utm_height -= 48.5
"""

import math
import numpy as np


def enu_to_latlon(enu_coords, ref_lat, ref_lon, ref_height):
    """Convert ENU coordinates to geodetic (lat, lon, height)."""
    east, north, up = enu_coords

    a  = 6378137.0
    f  = 1 / 298.257223563
    e2 = 2 * f - f * f

    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    N_ref = a / math.sqrt(1 - e2 * math.sin(ref_lat_rad) ** 2)
    x_ref = (N_ref + ref_height) * math.cos(ref_lat_rad) * math.cos(ref_lon_rad)
    y_ref = (N_ref + ref_height) * math.cos(ref_lat_rad) * math.sin(ref_lon_rad)
    z_ref = (N_ref * (1 - e2) + ref_height) * math.sin(ref_lat_rad)

    sin_lat = math.sin(ref_lat_rad)
    cos_lat = math.cos(ref_lat_rad)
    sin_lon = math.sin(ref_lon_rad)
    cos_lon = math.cos(ref_lon_rad)

    x = x_ref - sin_lon * east + cos_lon * north * (-sin_lat) + cos_lon * up * cos_lat
    y = y_ref + cos_lon * east + sin_lon * north * (-sin_lat) + sin_lon * up * cos_lat
    z = z_ref + north * cos_lat + up * sin_lat

    lon_rad = math.atan2(y, x)
    lon     = math.degrees(lon_rad)

    p       = math.sqrt(x * x + y * y)
    lat_rad = math.atan2(z, p * (1 - e2))

    for _ in range(10):
        N       = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)
        height  = p / math.cos(lat_rad) - N
        lat_rad_new = math.atan2(z, p * (1 - e2 * N / (N + height)))
        if abs(lat_rad_new - lat_rad) < 1e-12:
            break
        lat_rad = lat_rad_new

    lat = math.degrees(lat_rad)
    return lat, lon, height


def latlon_to_utm(lat, lon, height, central_meridian_deg=9.0):
    """Convert geodetic to UTM using ETRS89/GRS80 ellipsoid."""
    a        = 6378137.0
    f        = 1 / 298.257222101
    e2       = 2 * f - f * f
    e_prime2 = e2 / (1 - e2)

    lat_rad              = math.radians(lat)
    lon_rad              = math.radians(lon)
    central_meridian_rad = math.radians(central_meridian_deg)

    N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)
    T = math.tan(lat_rad) ** 2
    C = e_prime2 * math.cos(lat_rad) ** 2
    A = math.cos(lat_rad) * (lon_rad - central_meridian_rad)

    M = a * (
        (1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256) * lat_rad
        - (3 * e2 / 8 + 3 * e2 ** 2 / 32 + 45 * e2 ** 3 / 1024) * math.sin(2 * lat_rad)
        + (15 * e2 ** 2 / 256 + 45 * e2 ** 3 / 1024) * math.sin(4 * lat_rad)
        - (35 * e2 ** 3 / 3072) * math.sin(6 * lat_rad)
    )

    easting = 500000 + 0.9996 * N * (
        A + (1 - T + C) * A ** 3 / 6
        + (5 - 18 * T + T ** 2 + 72 * C - 58 * e_prime2) * A ** 5 / 120
    )
    northing = 0.9996 * (
        M + N * math.tan(lat_rad) * (
            A ** 2 / 2
            + (5 - T + 9 * C + 4 * C ** 2) * A ** 4 / 24
            + (61 - 58 * T + T ** 2 + 600 * C - 330 * e_prime2) * A ** 6 / 720
        )
    )

    if lat < 0:
        northing += 10000000

    return easting, northing, height


def convert_extracted_points_to_utm(extracted_points, enu_origin,
                                     central_meridian_deg=9.0):
    """
    Convert ENU SLAM poses to UTM.
    Applies geoid correction: enu[2] -= 48.22.
    """
    ref_lat, ref_lon, ref_height = enu_origin
    converted = []
    for point in extracted_points:
        enu    = point['position'].copy()
        enu[2] -= 48.22
        lat, lon, height = enu_to_latlon(enu, ref_lat, ref_lon, ref_height)
        easting, northing, utm_height = latlon_to_utm(lat, lon, height,
                                                       central_meridian_deg)
        converted.append({'position': np.array([easting, northing, utm_height])})
    return converted


def convert_gnss_to_utm(gnss_cps, central_meridian_deg=9.0):
    """
    Convert GNSS lat/lon/height to UTM.
    Applies geoid correction: utm_height -= 48.5.
    """
    result = []
    for cp in gnss_cps:
        easting, northing, utm_height = latlon_to_utm(
            cp['gnss_lat'], cp['gnss_lon'], cp['gnss_height'],
            central_meridian_deg)
        utm_height -= 48.5
        cp_out = cp.copy()
        cp_out.update({
            'gnss_utm_easting':  easting,
            'gnss_utm_northing': northing,
            'gnss_utm_height':   utm_height,
        })
        result.append(cp_out)
    return result
