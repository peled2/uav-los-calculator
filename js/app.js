/**
 * UAV LOS Calculator PWA - Main Tactical Controller
 * Connects Map (Topographic & Orthophoto ONLY), Collapsible Floating Parameters Menu,
 * Geodesic/Curvature Math Engine (RF & Optical up to 200 km), and Elevation Profile Chart.
 */

import {
  MAX_RANGE_KM,
  PROPAGATION_PRESETS,
  haversineDistanceKm,
  initialBearingDeg,
  destinationPoint,
  metersToFeet,
  feetToMeters,
  analyzeLosProfile,
  computeRangeHorizonCurves
} from './los-math.js';

import {
  fetchPathElevationProfile,
  getSinglePointElevation,
  compute360CoveragePolygon
} from './elevation-service.js';

import { LosChartRenderer } from './chart-renderer.js';

// Application State
const state = {
  mapType: 'topo', // Strictly 'topo' | 'ortho'
  clickTarget: 'uav', // 'uav' | 'gcs'
  paramsCollapsed: false,
  profileCollapsed: false,
  chartMode: 'profile', // 'profile' | 'horizon200'

  // Propagation & Earth Curvature
  propagationMode: 'rf', // 'rf' | 'optical' | 'optical_atm' | 'custom'
  kFactor: 4 / 3,
  useFresnel: true,
  freqMHz: 2400, // S-Band default
  fresnelClearancePct: 60,
  safetyMarginM: 15,

  // Ground Control Station (GCS) - Default: Central Israel (Palmachim / Coastal Plain)
  gcsLat: 31.898,
  gcsLon: 34.691,
  gcsGroundM: 32,
  gcsAutoGround: true,
  gcsAntennaAglM: 12,

  // UAV - Default: ~85 km NE (towards Galilee / Golan)
  uavLat: 32.545,
  uavLon: 35.185,
  uavGroundM: 310,
  uavAltUnit: 'ft', // 'ft' | 'm'
  uavAltRef: 'asl', // 'asl' | 'agl'
  uavAltitudeAslM: feetToMeters(12000), // 12,000 ft ASL (~3658 m)

  // Manual obstacles along path: [{ distanceKm, heightAglM }]
  manualObstacles: [],

  // Cached DEM samples along current GCS->UAV path
  pathSamples: [],
  demSource: 'dem-live',
  analysis: null,
  horizonCurves: [],
  coverage360Active: false
};

// Leaflet Map & Layer References
let map = null;
let topoLayerGroup = null;
let orthoLayerGroup = null;
let gcsMarker = null;
let uavMarker = null;
let clearLineLayer = null;
let blockedLineLayer = null;
let obstacleMarker = null;
let hoverMarker = null;
let range200Circle = null;
let smoothHorizonCircle = null;
let coverage360Polygon = null;
let chartRenderer = null;
let fetchDebounceTimer = null;
let deferredInstallPrompt = null;

/**
 * Initialize Map (Topographic & Orthophoto ONLY)
 */
function initMap() {
  map = L.map('map', {
    center: [32.22, 34.95],
    zoom: 9,
    zoomControl: false,
    attributionControl: true
  });

  L.control.zoom({ position: 'bottomleft' }).addTo(map);
  L.control.scale({ imperial: false, metric: true, position: 'bottomleft' }).addTo(map);

  // 1. Topographic Map Layer (Esri World Topo + OpenTopoMap Contours fallback)
  const esriTopo = L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}',
    {
      maxZoom: 19,
      attribution: 'נתוני מפה טופוגרפית: &copy; Esri, USGS, NOAA, OpenTopoMap'
    }
  );
  const openTopoOverlay = L.tileLayer(
    'https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',
    {
      maxZoom: 17,
      opacity: 0.55,
      attribution: '&copy; OpenTopoMap (CC-BY-SA)'
    }
  );
  topoLayerGroup = L.layerGroup([esriTopo, openTopoOverlay]);

  // 2. Orthophoto Map Layer (Esri World Imagery + Hybrid Reference Boundaries)
  const esriImagery = L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    {
      maxZoom: 19,
      attribution: 'אורתופוטו לוויין: &copy; Esri, Maxar, Earthstar Geographics'
    }
  );
  const esriLabels = L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}',
    {
      maxZoom: 19,
      opacity: 0.75
    }
  );
  orthoLayerGroup = L.layerGroup([esriImagery, esriLabels]);

  // Activate default Topographic map
  topoLayerGroup.addTo(map);

  // 200 km maximum operational range boundary circle around GCS
  range200Circle = L.circle([state.gcsLat, state.gcsLon], {
    radius: MAX_RANGE_KM * 1000,
    color: '#38bdf8',
    weight: 1.3,
    opacity: 0.45,
    fillColor: '#0ea5e9',
    fillOpacity: 0.02,
    dashArray: '8 6',
    interactive: false
  }).addTo(map);

  // Theoretical smooth-Earth horizon circle
  smoothHorizonCircle = L.circle([state.gcsLat, state.gcsLon], {
    radius: 50000,
    color: '#10b981',
    weight: 1.4,
    opacity: 0.65,
    fill: false,
    dashArray: '4 5',
    interactive: false
  }).addTo(map);

  // LOS Path Lines
  clearLineLayer = L.polyline([], {
    color: '#10b981',
    weight: 3.5,
    opacity: 0.95
  }).addTo(map);

  blockedLineLayer = L.polyline([], {
    color: '#ef4444',
    weight: 3.5,
    opacity: 0.9,
    dashArray: '8 6'
  }).addTo(map);

  // Create GCS & UAV tactical markers
  gcsMarker = L.marker([state.gcsLat, state.gcsLon], {
    draggable: true,
    icon: createGcsIcon()
  }).addTo(map);

  uavMarker = L.marker([state.uavLat, state.uavLon], {
    draggable: true,
    icon: createUavIcon('CLEAR', 30)
  }).addTo(map);

  // Marker Drag Events
  gcsMarker.on('drag', (e) => {
    const pos = e.target.getLatLng();
    state.gcsLat = pos.lat;
    state.gcsLon = pos.lng;
    clampUavToMaxRange();
    syncInputsFromState();
    scheduleElevationRefresh(120);
  });

  gcsMarker.on('dragend', async () => {
    if (state.gcsAutoGround) {
      const { elevationM } = await getSinglePointElevation(state.gcsLat, state.gcsLon);
      state.gcsGroundM = elevationM;
      syncInputsFromState();
    }
    refreshPathElevations();
  });

  uavMarker.on('drag', (e) => {
    const pos = e.target.getLatLng();
    applyUavCoordinatesWithClamp(pos.lat, pos.lng);
    syncInputsFromState();
    scheduleElevationRefresh(120);
  });

  uavMarker.on('dragend', () => {
    refreshPathElevations();
  });

  // Map Click Placement
  map.on('click', async (e) => {
    const { lat, lng } = e.latlng;
    if (state.clickTarget === 'gcs') {
      state.gcsLat = lat;
      state.gcsLon = lng;
      gcsMarker.setLatLng([lat, lng]);
      clampUavToMaxRange();
      if (state.gcsAutoGround) {
        const { elevationM } = await getSinglePointElevation(lat, lng);
        state.gcsGroundM = elevationM;
      }
    } else {
      applyUavCoordinatesWithClamp(lat, lng);
    }
    syncInputsFromState();
    refreshPathElevations();
  });
}

function createGcsIcon() {
  return L.divIcon({
    className: 'tactical-marker-wrapper',
    iconSize: [40, 40],
    iconAnchor: [20, 20],
    html: `
      <div class="marker-pin gcs-pin" title="תחנה קרקעית (GCS) - גרור להזזה">
        <span class="marker-label-tag">תחנה קרקעית</span>
        <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="#38bdf8" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
          <path d="M4.9 19.1C1 15.2 1 8.8 4.9 4.9"/>
          <path d="M7.8 16.2c-2.3-2.3-2.3-6.1 0-8.5"/>
          <circle cx="12" cy="12" r="2"/>
          <path d="M16.2 7.8c2.3 2.3 2.3 6.1 0 8.5"/>
          <path d="M19.1 4.9C23 8.8 23 15.1 19.1 19"/>
          <line x1="12" y1="14" x2="12" y2="22"/>
          <line x1="8" y1="22" x2="16" y2="22"/>
        </svg>
      </div>
    `
  });
}

function createUavIcon(status = 'CLEAR', bearingDeg = 0) {
  const color =
    status === 'BLOCKED'
      ? '#ef4444'
      : status === 'WARNING'
      ? '#f59e0b'
      : '#10b981';

  return L.divIcon({
    className: 'tactical-marker-wrapper',
    iconSize: [40, 40],
    iconAnchor: [20, 20],
    html: `
      <div class="marker-pin uav-pin status-${status}" title="כטב&quot;מ (UAV) - גרור להזזה עד 200 ק&quot;מ">
        <span class="marker-label-tag">כטב"מ</span>
        <svg width="22" height="22" viewBox="0 0 24 24" fill="${color}" style="transform: rotate(${Math.round(
          bearingDeg
        )}deg); transition: transform 0.1s linear;">
          <path d="M12 2L14.5 9.5L22 12L14.5 14.5L12 22L9.5 14.5L2 12L9.5 9.5L12 2Z" />
        </svg>
      </div>
    `
  });
}

/**
 * Ensure UAV distance from GCS never exceeds 200 km (MAX_RANGE_KM)
 */
function applyUavCoordinatesWithClamp(targetLat, targetLon) {
  const distKm = haversineDistanceKm(state.gcsLat, state.gcsLon, targetLat, targetLon);
  const brng = initialBearingDeg(state.gcsLat, state.gcsLon, targetLat, targetLon);

  if (distKm > MAX_RANGE_KM) {
    const clamped = destinationPoint(state.gcsLat, state.gcsLon, MAX_RANGE_KM, brng);
    state.uavLat = clamped.lat;
    state.uavLon = clamped.lon;
  } else if (distKm < 0.2) {
    const minPt = destinationPoint(state.gcsLat, state.gcsLon, 0.2, brng);
    state.uavLat = minPt.lat;
    state.uavLon = minPt.lon;
  } else {
    state.uavLat = targetLat;
    state.uavLon = targetLon;
  }
  uavMarker.setLatLng([state.uavLat, state.uavLon]);
}

function clampUavToMaxRange() {
  applyUavCoordinatesWithClamp(state.uavLat, state.uavLon);
}

/**
 * Switch strictly between Topographic ('topo') and Orthophoto ('ortho') maps
 */
function setMapLayer(type) {
  state.mapType = type === 'ortho' ? 'ortho' : 'topo';
  if (state.mapType === 'ortho') {
    if (map.hasLayer(topoLayerGroup)) map.removeLayer(topoLayerGroup);
    if (!map.hasLayer(orthoLayerGroup)) orthoLayerGroup.addTo(map);
  } else {
    if (map.hasLayer(orthoLayerGroup)) map.removeLayer(orthoLayerGroup);
    if (!map.hasLayer(topoLayerGroup)) topoLayerGroup.addTo(map);
  }

  document.getElementById('btn-map-topo').classList.toggle('active', state.mapType === 'topo');
  document.getElementById('btn-map-ortho').classList.toggle('active', state.mapType === 'ortho');
}

function scheduleElevationRefresh(delayMs = 150) {
  if (fetchDebounceTimer) clearTimeout(fetchDebounceTimer);
  fetchDebounceTimer = setTimeout(() => {
    refreshPathElevations();
  }, delayMs);
}

/**
 * Fetch DEM elevations along current GCS -> UAV geodesic path and run analysis
 */
async function refreshPathElevations() {
  const distKm = haversineDistanceKm(state.gcsLat, state.gcsLon, state.uavLat, state.uavLon);
  const numSamples = Math.min(160, Math.max(60, Math.round(distKm * 1.1)));

  const { samples, source } = await fetchPathElevationProfile(
    state.gcsLat,
    state.gcsLon,
    state.uavLat,
    state.uavLon,
    numSamples,
    state.manualObstacles
  );

  state.pathSamples = samples;
  state.demSource = source;

  if (samples.length > 0) {
    if (state.gcsAutoGround) {
      state.gcsGroundM = samples[0].elevationM;
    }
    state.uavGroundM = samples[samples.length - 1].elevationM;
  }

  recalculateLos();
}

/**
 * Re-run LOS mathematical analysis using current pathSamples (instant on parameter changes)
 */
function recalculateLos() {
  if (!state.pathSamples || state.pathSamples.length < 2) return;

  // Resolve UAV ASL altitude if user is in AGL mode
  let effectiveUavAslM = state.uavAltitudeAslM;
  if (state.uavAltRef === 'agl') {
    effectiveUavAslM = state.uavGroundM + state.uavAltitudeAslM;
  }

  const analysis = analyzeLosProfile({
    samples: state.pathSamples,
    gcsGroundM: state.gcsGroundM,
    gcsAntennaAglM: state.gcsAntennaAglM,
    uavAltitudeAslM: effectiveUavAslM,
    kFactor: state.kFactor,
    useFresnel: state.useFresnel,
    freqMHz: state.freqMHz,
    fresnelClearancePct: state.fresnelClearancePct,
    safetyMarginM: state.safetyMarginM
  });

  state.analysis = analysis;

  // Compute 0-200 km Horizon & Min Required UAV Altitude curves
  state.horizonCurves = computeRangeHorizonCurves({
    gcsTotalAslM: state.gcsGroundM + state.gcsAntennaAglM,
    kFactor: state.kFactor,
    safetyMarginM: state.safetyMarginM,
    radialTerrainSamples: state.pathSamples
  });

  updateMapVisuals();
  updateTelemetryUI();
  if (chartRenderer) {
    chartRenderer.setAltUnit(state.uavAltUnit);
    chartRenderer.updateData(state.analysis, state.horizonCurves);
  }
}

/**
 * Update Map Polylines, Horizon Rings, Obstacle Pin & Marker Orientations
 */
function updateMapVisuals() {
  if (!state.analysis) return;

  const a = state.analysis;
  const bearing = initialBearingDeg(state.gcsLat, state.gcsLon, state.uavLat, state.uavLon);

  // Update markers
  gcsMarker.setLatLng([state.gcsLat, state.gcsLon]);
  uavMarker.setLatLng([state.uavLat, state.uavLon]);
  uavMarker.setIcon(createUavIcon(a.status, bearing));

  // Update 200 km boundary circle & smooth-Earth horizon circle
  range200Circle.setLatLng([state.gcsLat, state.gcsLon]);
  smoothHorizonCircle.setLatLng([state.gcsLat, state.gcsLon]);
  const horizonRadiusM = Math.min(MAX_RANGE_KM, a.smoothHorizonKm) * 1000;
  smoothHorizonCircle.setRadius(horizonRadiusM);
  smoothHorizonCircle.setStyle({
    color: state.propagationMode === 'optical' ? '#f59e0b' : '#0ea5e9'
  });

  // Update LOS path line (split at critical blocking point if blocked!)
  const pts = a.profilePoints;
  if (a.isHardBlocked && a.criticalPoint && a.criticalPoint.index > 0) {
    const splitIdx = a.criticalPoint.index;
    const clearLatLngs = pts.slice(0, splitIdx + 1).map((p) => [p.lat, p.lon]);
    const blockedLatLngs = pts.slice(splitIdx).map((p) => [p.lat, p.lon]);

    clearLineLayer.setLatLngs(clearLatLngs);
    clearLineLayer.setStyle({ color: '#10b981' });
    blockedLineLayer.setLatLngs(blockedLatLngs);

    // Show critical obstacle marker on map
    const cp = a.criticalPoint;
    if (!obstacleMarker) {
      obstacleMarker = L.marker([cp.lat, cp.lon], {
        icon: L.divIcon({
          className: 'tactical-marker-wrapper',
          iconSize: [16, 16],
          iconAnchor: [8, 8],
          html: '<div class="obstacle-marker-dot" title="נקודת חסימה קריטית"></div>'
        })
      }).addTo(map);
    } else {
      obstacleMarker.setLatLng([cp.lat, cp.lon]);
    }
    obstacleMarker.bindTooltip(
      `חסימת קו ראייה (${cp.distanceKm.toFixed(1)} ק"מ | קרקע: ${cp.rawSurfaceM} מ')`,
      { direction: 'top', offset: [0, -8] }
    );
  } else {
    const allLatLngs = pts.map((p) => [p.lat, p.lon]);
    clearLineLayer.setLatLngs(allLatLngs);
    clearLineLayer.setStyle({
      color: a.status === 'WARNING' ? '#f59e0b' : '#10b981'
    });
    blockedLineLayer.setLatLngs([]);

    if (obstacleMarker) {
      map.removeLayer(obstacleMarker);
      obstacleMarker = null;
    }
  }
}

/**
 * Highlight hovered chart sample point on the Leaflet map
 */
function handleChartHoverSample(samplePt) {
  if (!samplePt) {
    if (hoverMarker) {
      map.removeLayer(hoverMarker);
      hoverMarker = null;
    }
    return;
  }

  if (!hoverMarker) {
    hoverMarker = L.marker([samplePt.lat, samplePt.lon], {
      interactive: false,
      icon: L.divIcon({
        className: 'tactical-marker-wrapper',
        iconSize: [14, 14],
        iconAnchor: [7, 7],
        html: '<div class="hover-sample-dot"></div>'
      })
    }).addTo(map);
  } else {
    hoverMarker.setLatLng([samplePt.lat, samplePt.lon]);
  }
}

/**
 * Update all HUD Badges, Collapsed Summary Bar, and Telemetry Cards
 */
function updateTelemetryUI() {
  if (!state.analysis) return;
  const a = state.analysis;
  const bearing = initialBearingDeg(state.gcsLat, state.gcsLon, state.uavLat, state.uavLon);

  // Status badge in Floating Panel Header
  const badgeEl = document.getElementById('los-status-badge');
  badgeEl.className = `status-badge ${a.status}`;
  if (a.status === 'CLEAR') {
    badgeEl.textContent = 'קו ראייה תקין';
  } else if (a.status === 'WARNING') {
    badgeEl.textContent = 'חדירת פרנל / מרווח';
  } else {
    badgeEl.textContent = 'קו ראייה חסום';
  }

  // Collapsed summary subtitle in Floating Panel Header
  const presetInfo = PROPAGATION_PRESETS[state.propagationMode] || PROPAGATION_PRESETS.custom;
  document.getElementById('panel-summary-subtitle').textContent =
    `${a.totalDistanceKm.toFixed(1)} ק"מ | ${presetInfo.shortHe} | מינ': ${Math.round(
      a.minRequiredUavFt
    ).toLocaleString('en-US')} ft`;

  // DEM status indicator
  const demDot = document.getElementById('dem-status-dot');
  const demText = document.getElementById('dem-status-text');
  if (state.demSource === 'offline-model') {
    demDot.classList.add('offline');
    demText.textContent = 'מודל טופוגרפי לא-מקוון';
  } else {
    demDot.classList.remove('offline');
    demText.textContent = 'גובה קרקע: SRTM DEM';
  }

  // Section header badges
  document.getElementById('badge-curvature').textContent = `k=${state.kFactor.toFixed(2)} | R=${Math.round(
    a.effectiveEarthRadiusKm
  )} ק"מ`;
  document.getElementById('badge-gcs').textContent = `${Math.round(a.gcsTotalAslM)} מ' ASL`;
  document.getElementById('badge-uav').textContent = `${a.totalDistanceKm.toFixed(1)} ק"מ | ${Math.round(
    bearing
  )}°`;

  // Curvature helper note
  document.getElementById('curvature-Summary-note').innerHTML =
    `רדיוס כדה"א אפקטיבי: <strong>${Math.round(a.effectiveEarthRadiusKm).toLocaleString(
      'en-US'
    )} ק"מ</strong> | שקיעת עקמומיות בטווח הנוכחי: <strong>${Math.round(
      a.totalCurvatureDropAtUavM
    )} מ'</strong> (${Math.round(metersToFeet(a.totalCurvatureDropAtUavM)).toLocaleString('en-US')} ft)`;

  // Bottom Telemetry KPI Strip
  const kpiRange = document.getElementById('kpi-range');
  kpiRange.textContent = `${a.totalDistanceKm.toFixed(1)} km (${a.totalDistanceNm.toFixed(1)} NM) | ${Math.round(
    bearing
  )}°`;

  const kpiCurvature = document.getElementById('kpi-curvature');
  kpiCurvature.textContent = `-${Math.round(a.totalCurvatureDropAtUavM)} מ' (שיא קשת: +${Math.round(
    a.maxChordBulgeM
  )} מ')`;

  const kpiMinAlt = document.getElementById('kpi-min-alt');
  kpiMinAlt.textContent = `${Math.round(a.minRequiredUavFt).toLocaleString('en-US')} ft (${Math.round(
    a.minRequiredUavAslM
  )} מ')`;
  kpiMinAlt.className = `kpi-value ${
    a.status === 'CLEAR' ? 'clear' : a.status === 'WARNING' ? 'warn' : 'block'
  }`;

  const kpiClearance = document.getElementById('kpi-clearance');
  const clrSign = a.minEffectiveClearanceM >= 0 ? '+' : '';
  const critDist = a.criticalPoint ? a.criticalPoint.distanceKm.toFixed(1) : '0.0';
  kpiClearance.textContent = `${clrSign}${Math.round(a.minEffectiveClearanceM)} מ' (ב-${critDist} ק"מ)`;
  kpiClearance.className = `kpi-value ${
    a.minEffectiveClearanceM >= 0 ? 'clear' : 'block'
  }`;

  const kpiHorizon = document.getElementById('kpi-horizon');
  kpiHorizon.textContent = `RF: ${Math.min(200, a.smoothHorizonRfKm).toFixed(1)} ק"מ | אופטי: ${Math.min(
    200,
    a.smoothHorizonOpticalKm
  ).toFixed(1)} ק"מ`;

  const kpiRf = document.getElementById('kpi-rf');
  kpiRf.textContent = `${a.fsplDb.toFixed(1)} dB | F1: ${a.maxFresnelRadiusM.toFixed(1)}m | ${a.elevAngleDeg.toFixed(
    2
  )}°`;
}

/**
 * Synchronize DOM Input Controls with current state values
 */
function syncInputsFromState() {
  const distKm = Math.min(
    MAX_RANGE_KM,
    haversineDistanceKm(state.gcsLat, state.gcsLon, state.uavLat, state.uavLon)
  );
  const bearingDeg = initialBearingDeg(state.gcsLat, state.gcsLon, state.uavLat, state.uavLon);

  // Propagation mode buttons
  document.querySelectorAll('[data-prop-mode]').forEach((btn) => {
    btn.classList.toggle('active', btn.dataset.propMode === state.propagationMode);
  });

  document.getElementById('input-k-factor').value = state.kFactor.toFixed(3);
  document.getElementById('slider-k-factor').value = state.kFactor;
  document.getElementById('input-freq-mhz').value = state.freqMHz;
  document.getElementById('input-fresnel-pct').value = state.fresnelClearancePct;
  document.getElementById('input-safety-margin').value = state.safetyMarginM;
  document.getElementById('rf-settings-group').style.display = state.useFresnel ? 'flex' : 'none';

  // GCS fields
  document.getElementById('input-gcs-lat').value = state.gcsLat.toFixed(5);
  document.getElementById('input-gcs-lon').value = state.gcsLon.toFixed(5);
  document.getElementById('input-gcs-ground').value = Math.round(state.gcsGroundM);
  document.getElementById('input-gcs-mast').value = state.gcsAntennaAglM;
  document.getElementById('slider-gcs-mast').value = state.gcsAntennaAglM;

  // UAV Range & Azimuth (0 - 200 km)
  document.getElementById('input-uav-range').value = distKm.toFixed(1);
  document.getElementById('slider-uav-range').value = distKm.toFixed(1);
  document.getElementById('input-uav-azimuth').value = Math.round(bearingDeg);
  document.getElementById('slider-uav-azimuth').value = Math.round(bearingDeg);

  // UAV Coordinates & Altitude
  document.getElementById('input-uav-lat').value = state.uavLat.toFixed(5);
  document.getElementById('input-uav-lon').value = state.uavLon.toFixed(5);

  const dispAlt =
    state.uavAltUnit === 'ft'
      ? Math.round(metersToFeet(state.uavAltitudeAslM))
      : Math.round(state.uavAltitudeAslM);
  document.getElementById('input-uav-alt').value = dispAlt;
  const sliderAltEl = document.getElementById('slider-uav-alt');
  sliderAltEl.max = state.uavAltUnit === 'ft' ? 50000 : 15000;
  sliderAltEl.step = state.uavAltUnit === 'ft' ? 100 : 25;
  sliderAltEl.value = dispAlt;
  document.getElementById('uav-alt-unit-label').textContent =
    state.uavAltUnit === 'ft' ? 'רגל (ft)' : "מטר (m)";

  document.getElementById('btn-unit-ft').classList.toggle('active', state.uavAltUnit === 'ft');
  document.getElementById('btn-unit-m').classList.toggle('active', state.uavAltUnit === 'm');
  document.getElementById('btn-ref-asl').classList.toggle('active', state.uavAltRef === 'asl');
  document.getElementById('btn-ref-agl').classList.toggle('active', state.uavAltRef === 'agl');
}

/**
 * Bind all UI Event Listeners
 */
function bindUIEvents() {
  // 1. Strictly Topographic & Orthophoto Map Switcher
  document.getElementById('btn-map-topo').addEventListener('click', () => setMapLayer('topo'));
  document.getElementById('btn-map-ortho').addEventListener('click', () => setMapLayer('ortho'));

  // 2. Click Placement Target Switcher (UAV vs GCS)
  document.getElementById('btn-click-uav').addEventListener('click', () => {
    state.clickTarget = 'uav';
    document.getElementById('btn-click-uav').classList.add('active');
    document.getElementById('btn-click-gcs').classList.remove('active');
  });
  document.getElementById('btn-click-gcs').addEventListener('click', () => {
    state.clickTarget = 'gcs';
    document.getElementById('btn-click-gcs').classList.add('active');
    document.getElementById('btn-click-uav').classList.remove('active');
  });

  // 3. Collapsible Floating Parameters Menu Toggle
  const panelEl = document.getElementById('floating-params-panel');
  const toggleParamsPanel = () => {
    state.paramsCollapsed = !state.paramsCollapsed;
    panelEl.classList.toggle('is-collapsed', state.paramsCollapsed);
  };
  document.getElementById('panel-header-toggle').addEventListener('click', toggleParamsPanel);

  // Accordion section headers inside Floating Parameters Menu
  document.querySelectorAll('.section-header').forEach((hdr) => {
    hdr.addEventListener('click', () => {
      const parent = hdr.closest('.param-section');
      if (parent) parent.classList.toggle('sec-collapsed');
    });
  });

  // 4. Bottom Profile Drawer Collapse Toggle & Chart Mode Switcher
  const drawerEl = document.getElementById('profile-drawer');
  document.getElementById('btn-toggle-drawer').addEventListener('click', () => {
    state.profileCollapsed = !state.profileCollapsed;
    drawerEl.classList.toggle('is-collapsed', state.profileCollapsed);
    document.body.classList.toggle('profile-collapsed', state.profileCollapsed);
    setTimeout(() => {
      if (!state.profileCollapsed && chartRenderer) chartRenderer.render();
    }, 280);
  });

  document.getElementById('btn-chart-profile').addEventListener('click', () => {
    state.chartMode = 'profile';
    document.getElementById('btn-chart-profile').classList.add('active');
    document.getElementById('btn-chart-horizon').classList.remove('active');
    chartRenderer.setMode('profile');
  });

  document.getElementById('btn-chart-horizon').addEventListener('click', () => {
    state.chartMode = 'horizon200';
    document.getElementById('btn-chart-horizon').classList.add('active');
    document.getElementById('btn-chart-profile').classList.remove('active');
    chartRenderer.setMode('horizon200');
  });

  // 5. Propagation Mode Buttons (RF k=4/3 vs Optical k=1.0 vs Optical Atm vs Custom)
  document.querySelectorAll('[data-prop-mode]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const mode = btn.dataset.propMode;
      state.propagationMode = mode;
      const preset = PROPAGATION_PRESETS[mode];
      if (preset && mode !== 'custom') {
        state.kFactor = preset.kFactor;
        state.useFresnel = preset.useFresnel;
      }
      syncInputsFromState();
      recalculateLos();
    });
  });

  // Custom k-factor inputs
  const onKChange = (val) => {
    const num = parseFloat(val);
    if (!Number.isFinite(num)) return;
    state.kFactor = Math.max(0.5, Math.min(2.5, num));
    state.propagationMode = 'custom';
    syncInputsFromState();
    recalculateLos();
  };
  document.getElementById('input-k-factor').addEventListener('change', (e) => onKChange(e.target.value));
  document.getElementById('slider-k-factor').addEventListener('input', (e) => onKChange(e.target.value));

  // RF Frequency & Presets
  document.getElementById('input-freq-mhz').addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    if (Number.isFinite(val) && val > 10) {
      state.freqMHz = val;
      recalculateLos();
    }
  });
  document.querySelectorAll('[data-freq-preset]').forEach((chip) => {
    chip.addEventListener('click', () => {
      state.freqMHz = parseFloat(chip.dataset.freqPreset);
      syncInputsFromState();
      recalculateLos();
    });
  });

  document.getElementById('input-fresnel-pct').addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    if (Number.isFinite(val)) {
      state.fresnelClearancePct = Math.max(0, Math.min(100, val));
      recalculateLos();
    }
  });

  document.getElementById('input-safety-margin').addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    if (Number.isFinite(val)) {
      state.safetyMarginM = Math.max(0, Math.min(1000, val));
      recalculateLos();
    }
  });

  // 6. GCS Controls
  const onGcsCoordChange = () => {
    const lat = parseFloat(document.getElementById('input-gcs-lat').value);
    const lon = parseFloat(document.getElementById('input-gcs-lon').value);
    if (Number.isFinite(lat) && Number.isFinite(lon)) {
      state.gcsLat = Math.max(-85, Math.min(85, lat));
      state.gcsLon = Math.max(-180, Math.min(180, lon));
      clampUavToMaxRange();
      refreshPathElevations();
    }
  };
  document.getElementById('input-gcs-lat').addEventListener('change', onGcsCoordChange);
  document.getElementById('input-gcs-lon').addEventListener('change', onGcsCoordChange);

  document.getElementById('input-gcs-ground').addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    if (Number.isFinite(val)) {
      state.gcsGroundM = val;
      state.gcsAutoGround = false;
      recalculateLos();
    }
  });

  const onMastChange = (val) => {
    const num = parseFloat(val);
    if (Number.isFinite(num)) {
      state.gcsAntennaAglM = Math.max(0, Math.min(300, num));
      document.getElementById('input-gcs-mast').value = state.gcsAntennaAglM;
      document.getElementById('slider-gcs-mast').value = state.gcsAntennaAglM;
      recalculateLos();
    }
  };
  document.getElementById('input-gcs-mast').addEventListener('input', (e) => onMastChange(e.target.value));
  document.getElementById('slider-gcs-mast').addEventListener('input', (e) => onMastChange(e.target.value));

  // GPS Locate GCS Button
  document.getElementById('btn-gcs-gps').addEventListener('click', () => {
    if (!navigator.geolocation) return;
    navigator.geolocation.getCurrentPosition(
      async (pos) => {
        state.gcsLat = pos.coords.latitude;
        state.gcsLon = pos.coords.longitude;
        state.gcsAutoGround = true;
        const brng = initialBearingDeg(state.gcsLat, state.gcsLon, state.uavLat, state.uavLon);
        const dest = destinationPoint(state.gcsLat, state.gcsLon, 50, brng);
        state.uavLat = dest.lat;
        state.uavLon = dest.lon;
        map.setView([state.gcsLat, state.gcsLon], 9);
        syncInputsFromState();
        refreshPathElevations();
      },
      () => {}
    );
  });

  // 7. UAV Range (0.2 - 200 km) & Azimuth (0 - 360 deg) Controls
  const updateUavByPolar = (rangeKm, azimuthDeg) => {
    const clampedRange = Math.max(0.2, Math.min(MAX_RANGE_KM, rangeKm));
    const normAz = ((azimuthDeg % 360) + 360) % 360;
    const dest = destinationPoint(state.gcsLat, state.gcsLon, clampedRange, normAz);
    state.uavLat = dest.lat;
    state.uavLon = dest.lon;
    uavMarker.setLatLng([state.uavLat, state.uavLon]);
    syncInputsFromState();
    scheduleElevationRefresh(90);
  };

  document.getElementById('slider-uav-range').addEventListener('input', (e) => {
    const r = parseFloat(e.target.value);
    const az = parseFloat(document.getElementById('input-uav-azimuth').value) || 0;
    updateUavByPolar(r, az);
  });
  document.getElementById('input-uav-range').addEventListener('change', (e) => {
    const r = parseFloat(e.target.value);
    const az = parseFloat(document.getElementById('input-uav-azimuth').value) || 0;
    if (Number.isFinite(r)) updateUavByPolar(r, az);
  });

  // Quick range preset chips (25, 50, 100, 150, 200 km)
  document.querySelectorAll('[data-range-preset]').forEach((chip) => {
    chip.addEventListener('click', () => {
      const r = parseFloat(chip.dataset.rangePreset);
      const az = parseFloat(document.getElementById('input-uav-azimuth').value) || 0;
      updateUavByPolar(r, az);
    });
  });

  document.getElementById('slider-uav-azimuth').addEventListener('input', (e) => {
    const az = parseFloat(e.target.value);
    const r = parseFloat(document.getElementById('input-uav-range').value) || 50;
    updateUavByPolar(r, az);
  });
  document.getElementById('input-uav-azimuth').addEventListener('change', (e) => {
    const az = parseFloat(e.target.value);
    const r = parseFloat(document.getElementById('input-uav-range').value) || 50;
    if (Number.isFinite(az)) updateUavByPolar(r, az);
  });

  // UAV Direct Lat/Lon
  const onUavCoordChange = () => {
    const lat = parseFloat(document.getElementById('input-uav-lat').value);
    const lon = parseFloat(document.getElementById('input-uav-lon').value);
    if (Number.isFinite(lat) && Number.isFinite(lon)) {
      applyUavCoordinatesWithClamp(lat, lon);
      syncInputsFromState();
      refreshPathElevations();
    }
  };
  document.getElementById('input-uav-lat').addEventListener('change', onUavCoordChange);
  document.getElementById('input-uav-lon').addEventListener('change', onUavCoordChange);

  // UAV Altitude & Units (ft vs m, ASL vs AGL)
  const onUavAltInput = (val) => {
    const num = parseFloat(val);
    if (!Number.isFinite(num)) return;
    state.uavAltitudeAslM =
      state.uavAltUnit === 'ft' ? feetToMeters(num) : num;
    document.getElementById('input-uav-alt').value = Math.round(num);
    document.getElementById('slider-uav-alt').value = Math.round(num);
    recalculateLos();
  };
  document.getElementById('input-uav-alt').addEventListener('input', (e) => onUavAltInput(e.target.value));
  document.getElementById('slider-uav-alt').addEventListener('input', (e) => onUavAltInput(e.target.value));

  document.querySelectorAll('[data-alt-preset-ft]').forEach((chip) => {
    chip.addEventListener('click', () => {
      const ft = parseFloat(chip.dataset.altPresetFt);
      state.uavAltitudeAslM = feetToMeters(ft);
      syncInputsFromState();
      recalculateLos();
    });
  });

  document.getElementById('btn-unit-ft').addEventListener('click', () => {
    state.uavAltUnit = 'ft';
    syncInputsFromState();
    recalculateLos();
  });
  document.getElementById('btn-unit-m').addEventListener('click', () => {
    state.uavAltUnit = 'm';
    syncInputsFromState();
    recalculateLos();
  });

  document.getElementById('btn-ref-asl').addEventListener('click', () => {
    state.uavAltRef = 'asl';
    syncInputsFromState();
    recalculateLos();
  });
  document.getElementById('btn-ref-agl').addEventListener('click', () => {
    state.uavAltRef = 'agl';
    syncInputsFromState();
    recalculateLos();
  });

  // 8. 360° Coverage Scan up to 200 km & Fit Map Bounds
  document.getElementById('btn-scan-360').addEventListener('click', async () => {
    const btn = document.getElementById('btn-scan-360');
    if (state.coverage360Active && coverage360Polygon) {
      map.removeLayer(coverage360Polygon);
      coverage360Polygon = null;
      state.coverage360Active = false;
      btn.textContent = 'מעטפת כיסוי 360° (עד 200 ק"מ)';
      return;
    }

    btn.disabled = true;
    btn.textContent = 'מחשב כיסוי 360°...';
    try {
      const effectiveUavAslM =
        state.uavAltRef === 'agl'
          ? state.uavGroundM + state.uavAltitudeAslM
          : state.uavAltitudeAslM;

      const { polygonCoords } = await compute360CoveragePolygon({
        gcsLat: state.gcsLat,
        gcsLon: state.gcsLon,
        gcsTotalAslM: state.gcsGroundM + state.gcsAntennaAglM,
        uavAltitudeAslM: effectiveUavAslM,
        kFactor: state.kFactor,
        safetyMarginM: state.safetyMarginM,
        maxRangeKm: MAX_RANGE_KM
      });

      if (coverage360Polygon) map.removeLayer(coverage360Polygon);
      coverage360Polygon = L.polygon(polygonCoords, {
        color: '#10b981',
        weight: 2,
        opacity: 0.85,
        fillColor: '#10b981',
        fillOpacity: 0.14
      }).addTo(map);

      state.coverage360Active = true;
      btn.textContent = 'הסתר מעטפת 360°';
    } finally {
      btn.disabled = false;
    }
  });

  document.getElementById('btn-fit-link').addEventListener('click', () => {
    const bounds = L.latLngBounds([
      [state.gcsLat, state.gcsLon],
      [state.uavLat, state.uavLon]
    ]);
    map.fitBounds(bounds, { padding: [70, 70] });
  });

  document.getElementById('btn-export-geojson').addEventListener('click', exportGeoJsonReport);

  window.addEventListener('resize', () => {
    if (chartRenderer) chartRenderer.render();
  });
}

/**
 * Export LOS Link Profile & Metadata as GeoJSON
 */
function exportGeoJsonReport() {
  if (!state.analysis) return;
  const a = state.analysis;
  const geojson = {
    type: 'FeatureCollection',
    metadata: {
      generator: 'UAV LOS Calculator PWA',
      timestamp: new Date().toISOString(),
      status: a.status,
      propagationMode: state.propagationMode,
      kFactor: state.kFactor,
      totalDistanceKm: Number(a.totalDistanceKm.toFixed(3)),
      minRequiredUavAslM: Math.round(a.minRequiredUavAslM),
      minRequiredUavFt: Math.round(a.minRequiredUavFt)
    },
    features: [
      {
        type: 'Feature',
        properties: {
          role: 'GCS',
          groundAslM: state.gcsGroundM,
          antennaAglM: state.gcsAntennaAglM,
          totalAslM: a.gcsTotalAslM
        },
        geometry: {
          type: 'Point',
          coordinates: [state.gcsLon, state.gcsLat, a.gcsTotalAslM]
        }
      },
      {
        type: 'Feature',
        properties: {
          role: 'UAV',
          groundAslM: state.uavGroundM,
          altitudeAslM: Math.round(a.uavAltitudeAslM),
          altitudeFt: Math.round(metersToFeet(a.uavAltitudeAslM))
        },
        geometry: {
          type: 'Point',
          coordinates: [state.uavLon, state.uavLat, Math.round(a.uavAltitudeAslM)]
        }
      },
      {
        type: 'Feature',
        properties: {
          role: 'LOS_PATH',
          status: a.status,
          minClearanceM: Math.round(a.minEffectiveClearanceM)
        },
        geometry: {
          type: 'LineString',
          coordinates: a.profilePoints.map((p) => [
            p.lon,
            p.lat,
            Math.round(p.losBeamM)
          ])
        }
      }
    ]
  };

  const blob = new Blob([JSON.stringify(geojson, null, 2)], {
    type: 'application/geo+json'
  });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = `uav-los-${a.totalDistanceKm.toFixed(0)}km-${a.status.toLowerCase()}.geojson`;
  link.click();
  URL.revokeObjectURL(url);
}

/**
 * Setup PWA Service Worker & Install Prompt
 */
function initPwaSupport() {
  if ('serviceWorker' in navigator) {
    window.addEventListener('load', () => {
      navigator.serviceWorker.register('/sw.js').catch(() => {});
    });
  }

  const installBtn = document.getElementById('btn-install-pwa');
  window.addEventListener('beforeinstallprompt', (e) => {
    e.preventDefault();
    deferredInstallPrompt = e;
    installBtn.style.display = 'inline-flex';
  });

  installBtn.addEventListener('click', async () => {
    if (!deferredInstallPrompt) return;
    deferredInstallPrompt.prompt();
    await deferredInstallPrompt.userChoice;
    deferredInstallPrompt = null;
    installBtn.style.display = 'none';
  });
}

/**
 * Boot Application
 */
document.addEventListener('DOMContentLoaded', () => {
  initMap();
  const canvas = document.getElementById('los-chart-canvas');
  chartRenderer = new LosChartRenderer(canvas, {
    onHoverSample: handleChartHoverSample
  });

  bindUIEvents();
  syncInputsFromState();
  initPwaSupport();
  refreshPathElevations();
});
