/**
 * UAV LOS Calculator - Geodesic, WGS84 UTM Converter & Earth Curvature Utilities
 */

export const EARTH_RADIUS_M = 6371000;
export const EARTH_RADIUS_KM = 6371;
export const MAX_RANGE_KM = 200;
export const M_TO_FT = 3.280839895;
export const FT_TO_M = 0.3048;

// WGS84 Ellipsoid Constants for UTM Conversion
const WGS84_A = 6378137.0;
const WGS84_F = 1.0 / 298.257223563;
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F;
const WGS84_EP2 = WGS84_E2 / (1 - WGS84_E2);
const UTM_K0 = 0.9996;

export function toRadians(deg) {
  return (deg * Math.PI) / 180;
}

export function toDegrees(rad) {
  return (rad * 180) / Math.PI;
}

/**
 * Great-circle distance between two coordinates using Haversine formula (returns km)
 */
export function haversineDistanceKm(lat1, lon1, lat2, lon2) {
  const dLat = toRadians(lat2 - lat1);
  const dLon = toRadians(lon2 - lon1);
  const rLat1 = toRadians(lat1);
  const rLat2 = toRadians(lat2);

  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(rLat1) * Math.cos(rLat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(Math.max(0, 1 - a)));
  return EARTH_RADIUS_KM * c;
}

/**
 * Forward azimuth (initial bearing) in degrees [0, 360) from point 1 to point 2
 */
export function initialBearingDeg(lat1, lon1, lat2, lon2) {
  const rLat1 = toRadians(lat1);
  const rLat2 = toRadians(lat2);
  const dLon = toRadians(lon2 - lon1);

  const y = Math.sin(dLon) * Math.cos(rLat2);
  const x =
    Math.cos(rLat1) * Math.sin(rLat2) -
    Math.sin(rLat1) * Math.cos(rLat2) * Math.cos(dLon);
  const brng = toDegrees(Math.atan2(y, x));
  return (brng + 360) % 360;
}

/**
 * Destination coordinate given starting point, distance (km), and bearing (degrees)
 */
export function destinationPoint(lat, lon, distanceKm, bearingDeg) {
  const angularDist = distanceKm / EARTH_RADIUS_KM;
  const brng = toRadians(bearingDeg);
  const lat1 = toRadians(lat);
  const lon1 = toRadians(lon);

  const sinLat1 = Math.sin(lat1);
  const cosLat1 = Math.cos(lat1);
  const sinAd = Math.sin(angularDist);
  const cosAd = Math.cos(angularDist);

  const sinLat2 = sinLat1 * cosAd + cosLat1 * sinAd * Math.cos(brng);
  const lat2 = Math.asin(Math.max(-1, Math.min(1, sinLat2)));
  const y = Math.sin(brng) * sinAd * cosLat1;
  const x = cosAd - sinLat1 * sinLat2;
  const lon2 = lon1 + Math.atan2(y, x);

  return {
    lat: toDegrees(lat2),
    lon: ((toDegrees(lon2) + 540) % 360) - 180
  };
}

/**
 * Convert WGS84 Geographic (lat, lon in degrees) to UTM WGS84
 * @returns {{ zone: number, hemisphere: 'N'|'S', easting: number, northing: number }}
 */
export function latLonToUtm(lat, lon, forceZone = null) {
  const clampedLat = Math.max(-80, Math.min(84, lat));
  const normLon = ((lon + 180) % 360 + 360) % 360 - 180;

  const zone = forceZone || Math.floor((normLon + 180) / 6) + 1;
  const lonOrigin = (zone - 1) * 6 - 180 + 3;
  const lonOriginRad = toRadians(lonOrigin);

  const latRad = toRadians(clampedLat);
  const lonRad = toRadians(normLon);

  const sinLat = Math.sin(latRad);
  const cosLat = Math.cos(latRad);
  const tanLat = Math.tan(latRad);

  const N = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat * sinLat);
  const T = tanLat * tanLat;
  const C = WGS84_EP2 * cosLat * cosLat;
  const A = cosLat * (lonRad - lonOriginRad);

  const e2 = WGS84_E2;
  const e4 = e2 * e2;
  const e6 = e4 * e2;

  const M =
    WGS84_A *
    ((1 - e2 / 4 - (3 * e4) / 64 - (5 * e6) / 256) * latRad -
      ((3 * e2) / 8 + (3 * e4) / 32 + (45 * e6) / 1024) * Math.sin(2 * latRad) +
      ((15 * e4) / 256 + (45 * e6) / 1024) * Math.sin(4 * latRad) -
      ((35 * e6) / 3072) * Math.sin(6 * latRad));

  const easting =
    UTM_K0 *
      N *
      (A +
        ((1 - T + C) * A * A * A) / 6 +
        ((5 - 18 * T + T * T + 72 * C - 58 * WGS84_EP2) * Math.pow(A, 5)) / 120) +
    500000.0;

  let northing =
    UTM_K0 *
    (M +
      N *
        tanLat *
        ((A * A) / 2 +
          ((5 - T + 9 * C + 4 * C * C) * Math.pow(A, 4)) / 24 +
          ((61 - 58 * T + T * T + 600 * C - 330 * WGS84_EP2) * Math.pow(A, 6)) / 720));

  const hemisphere = clampedLat < 0 ? 'S' : 'N';
  if (clampedLat < 0) {
    northing += 10000000.0;
  }

  return {
    zone,
    hemisphere,
    easting: Math.round(easting),
    northing: Math.round(northing)
  };
}

/**
 * Convert UTM WGS84 (zone, hemisphere, easting, northing) to Geographic (lat, lon in degrees)
 * @returns {{ lat: number, lon: number } | null}
 */
export function utmToLatLon(zone, hemisphere, easting, northing) {
  if (
    !Number.isFinite(zone) ||
    zone < 1 ||
    zone > 60 ||
    !Number.isFinite(easting) ||
    !Number.isFinite(northing)
  ) {
    return null;
  }

  const x = easting - 500000.0;
  const y = String(hemisphere).toUpperCase() === 'S' ? northing - 10000000.0 : northing;

  const lonOrigin = (zone - 1) * 6 - 180 + 3;
  const e2 = WGS84_E2;
  const e4 = e2 * e2;
  const e6 = e4 * e2;
  const e1 = (1 - Math.sqrt(1 - e2)) / (1 + Math.sqrt(1 - e2));

  const M = y / UTM_K0;
  const mu = M / (WGS84_A * (1 - e2 / 4 - (3 * e4) / 64 - (5 * e6) / 256));

  const phi1Rad =
    mu +
    ((3 * e1) / 2 - (27 * Math.pow(e1, 3)) / 32) * Math.sin(2 * mu) +
    ((21 * e1 * e1) / 16 - (55 * Math.pow(e1, 4)) / 32) * Math.sin(4 * mu) +
    ((151 * Math.pow(e1, 3)) / 96) * Math.sin(6 * mu) +
    ((1097 * Math.pow(e1, 4)) / 512) * Math.sin(8 * mu);

  const sinPhi1 = Math.sin(phi1Rad);
  const cosPhi1 = Math.cos(phi1Rad);
  const tanPhi1 = Math.tan(phi1Rad);

  const N1 = WGS84_A / Math.sqrt(1 - e2 * sinPhi1 * sinPhi1);
  const T1 = tanPhi1 * tanPhi1;
  const C1 = WGS84_EP2 * cosPhi1 * cosPhi1;
  const R1 = (WGS84_A * (1 - e2)) / Math.pow(1 - e2 * sinPhi1 * sinPhi1, 1.5);
  const D = x / (N1 * UTM_K0);

  const latRad =
    phi1Rad -
    ((N1 * tanPhi1) / R1) *
      ((D * D) / 2 -
        ((5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * WGS84_EP2) * Math.pow(D, 4)) / 24 +
        ((61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * WGS84_EP2 - 3 * C1 * C1) *
          Math.pow(D, 6)) /
          720);

  const lonRad =
    toRadians(lonOrigin) +
    (D -
      ((1 + 2 * T1 + C1) * Math.pow(D, 3)) / 6 +
      ((5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * WGS84_EP2 + 24 * T1 * T1) *
        Math.pow(D, 5)) /
        120) /
      cosPhi1;

  const lat = toDegrees(latRad);
  const lon = toDegrees(lonRad);
  if (!Number.isFinite(lat) || !Number.isFinite(lon) || Math.abs(lat) > 85) {
    return null;
  }
  return { lat, lon };
}
