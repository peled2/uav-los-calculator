/**
 * UAV LOS Calculator - Main Controller for 360° Viewshed / Max Range Envelope PWA
 */

import {
  FT_TO_M,
  initialBearingDeg,
  destinationPoint,
  haversineDistanceKm
} from './los-math.js';
import { LosChartRenderer } from './chart-renderer.js';

const state = {
  mapType: 'topo', // 'topo' | 'ortho'
  panelCollapsed: false,
  drawerCollapsed: true,

  // Ground Station (Default: Central Israel)
  lat0: 31.8980,
  lon0: 34.6910,
  hGroundM: 32,
  hAntM: 15, // Antenna AGL in meters

  // Aircraft Altitude in Feet ASL (500 .. 40,000 ft)
  hUavFt: 10000,

  // Max Calculation Range in km (10 .. 200 km)
  rMaxKm: 200,

  // Earth Curvature Model: 'rf' (k = 4/3) | 'opt' (k = 1.0)
  curvatureMode: 'rf',
  kFactor: 4 / 3,

  // Latest Viewshed Output
  avgRangeKm: 0,
  minRangeKm: 0,
  maxObservedRangeKm: 0,
  raySummaries: [],

  // Inspection Ray State
  inspectAzimuthDeg: 45,
  inspectLocked: false,
  lastProfileReqTime: 0
};

let map = null;
let topoLayerGroup = null;
let orthoLayerGroup = null;
let stationMarker = null;
let ringsLayerGroup = null;
let polygonHaloLayer = null;
let polygonMainLayer = null;
let inspectRayLayer = null;
let inspectShadowLayer = null;
let ridgeBlockerMarker = null;

let worker = null;
let chartRenderer = null;

/**
 * Initialize Full-Viewport Leaflet Map with Strictly 2 Map Layers:
 * 1) Topographic: OpenTopoMap + Israel Hiking Map (Hebrew)
 * 2) Orthophoto: Esri World Imagery + Esri Boundaries & Places
 */
function initMap() {
  map = L.map('map', {
    center: [state.lat0, state.lon0],
    zoom: 8,
    zoomControl: false,
    attributionControl: true
  });

  L.control.zoom({ position: 'bottomleft' }).addTo(map);
  L.control.scale({ imperial: false, metric: true, position: 'bottomleft' }).addTo(map);

  // Layer 1: Topographic Map (OpenTopoMap + Israel Hiking Map Hebrew tiles)
  const openTopo = L.tileLayer('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png', {
    maxZoom: 17,
    attribution: '&copy; OpenTopoMap (CC-BY-SA)'
  });
  const israelHikingHebrew = L.tileLayer(
    'https://israelhiking.osm.org.il/Hebrew/Tiles/{z}/{x}/{y}.png',
    {
      maxNativeZoom: 16,
      maxZoom: 18,
      attribution: '&copy; Israel Hiking Map (OSM)'
    }
  );
  topoLayerGroup = L.layerGroup([openTopo, israelHikingHebrew]);

  // Layer 2: Orthophoto Map (Esri World Imagery + Boundaries & Places)
  const esriImagery = L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    {
      maxZoom: 19,
      attribution: '&copy; Esri World Imagery'
    }
  );
  const esriPlaces = L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}',
    {
      maxZoom: 19,
      opacity: 0.85
    }
  );
  orthoLayerGroup = L.layerGroup([esriImagery, esriPlaces]);

  // Default: Topographic Map
  topoLayerGroup.addTo(map);

  // Concentric Range Rings Layer Group (50, 100, 150, R_max km)
  ringsLayerGroup = L.layerGroup().addTo(map);

  // 360° Viewshed Polygon with Double Border (Dark Halo underneath + Glowing Main Line)
  polygonHaloLayer = L.polygon([], {
    color: '#050b12',
    weight: 6.5,
    opacity: 0.88,
    fill: false,
    interactive: false,
    lineJoin: 'round'
  }).addTo(map);

  polygonMainLayer = L.polygon([], {
    color: '#22c55e',
    weight: 2.8,
    opacity: 1.0,
    fillColor: '#22c55e',
    fillOpacity: 0.18,
    interactive: false,
    lineJoin: 'round'
  }).addTo(map);

  // Hover / Inspection Ray Layers
  inspectRayLayer = L.polyline([], {
    color: '#38bdf8',
    weight: 2.4,
    opacity: 0.92,
    interactive: false
  }).addTo(map);

  inspectShadowLayer = L.polyline([], {
    color: '#ef4444',
    weight: 1.8,
    opacity: 0.65,
    dashArray: '5 5',
    interactive: false
  }).addTo(map);

  // Draggable Ground Station Marker
  stationMarker = L.marker([state.lat0, state.lon0], {
    draggable: true,
    icon: L.divIcon({
      className: 'station-marker-wrap',
      iconSize: [36, 36],
      iconAnchor: [18, 18],
      html: `
        <div class="station-pin" title="תחנה קרקעית (גרור או לחץ במפה להזזה)">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#38bdf8" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
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
    })
  }).addTo(map);

  stationMarker.on('drag', (e) => {
    const pos = e.target.getLatLng();
    state.lat0 = pos.lat;
    state.lon0 = pos.lng;
    syncInputsFromState();
    drawConcentricRangeRings();
  });

  stationMarker.on('dragend', () => {
    triggerFullViewshedCalculation();
  });

  // Clicking anywhere on the map places the Ground Station and recalculates 360° LOS
  map.on('click', (e) => {
    state.lat0 = e.latlng.lat;
    state.lon0 = e.latlng.lng;
    stationMarker.setLatLng([state.lat0, state.lon0]);
    syncInputsFromState();
    drawConcentricRangeRings();
    triggerFullViewshedCalculation();
  });

  // Hovering around the station updates the Inspection Ray & Cross-Section
  map.on('mousemove', (e) => {
    if (state.inspectLocked || !state.raySummaries.length) return;
    const distKm = haversineDistanceKm(state.lat0, state.lon0, e.latlng.lat, e.latlng.lng);
    if (distKm < 0.3) return;
    const az = initialBearingDeg(state.lat0, state.lon0, e.latlng.lat, e.latlng.lng);
    updateInspectionRay(az, false);
  });

  // Right-click locks/unlocks the Inspection Ray and opens the Cross-Section Drawer
  map.on('contextmenu', (e) => {
    const az = initialBearingDeg(state.lat0, state.lon0, e.latlng.lat, e.latlng.lng);
    state.inspectLocked = !state.inspectLocked;
    if (state.inspectLocked) {
      updateInspectionRay(az, true);
      if (state.drawerCollapsed) {
        toggleDrawer(false);
      }
    } else {
      updateInspectionRay(az, false);
    }
  });

  drawConcentricRangeRings();
}

/**
 * Draw Concentric Range Rings (50, 100, 150, and R_max km) around the Ground Station
 */
function drawConcentricRangeRings() {
  if (!ringsLayerGroup) return;
  ringsLayerGroup.clearLayers();

  const standardRings = [50, 100, 150];
  const ringDistances = standardRings.filter((r) => r < state.rMaxKm);
  ringDistances.push(state.rMaxKm);

  for (const rKm of ringDistances) {
    const isOuter = rKm === state.rMaxKm;
    L.circle([state.lat0, state.lon0], {
      radius: rKm * 1000.0,
      color: isOuter ? '#38bdf8' : '#94a3b8',
      weight: isOuter ? 1.5 : 1.0,
      opacity: isOuter ? 0.65 : 0.42,
      fill: false,
      dashArray: isOuter ? '6 5' : '4 6',
      interactive: false
    }).addTo(ringsLayerGroup);

    // Distance label at North (0°) of each ring
    const labelPt = destinationPoint(state.lat0, state.lon0, rKm, 0);
    L.marker([labelPt.lat, labelPt.lon], {
      interactive: false,
      icon: L.divIcon({
        className: 'station-marker-wrap',
        iconSize: [0, 0],
        html: `<div class="ring-label-badge">${rKm} ק"מ</div>`
      })
    }).addTo(ringsLayerGroup);
  }
}

/**
 * Initialize Web Worker (`js/viewshed-worker.js`)
 */
function initWorker() {
  worker = new Worker('/js/viewshed-worker.js');

  worker.onmessage = async (e) => {
    const msg = e.data;
    if (!msg || !msg.type) return;

    // Fallback PNG decoding on main thread if Worker lacks OffscreenCanvas
    if (msg.type === 'DECODE_PNG_REQ') {
      const rgbaBuffer = await decodePngOnMainThread(msg.pngBuffer);
      worker.postMessage({ type: 'DECODE_PNG_RES', id: msg.id, rgbaBuffer }, [rgbaBuffer]);
      return;
    }

    if (msg.type === 'PROGRESS') {
      const wrap = document.getElementById('worker-progress-wrap');
      const bar = document.getElementById('worker-progress-bar');
      const txt = document.getElementById('worker-progress-text');
      wrap.style.display = 'block';
      bar.style.width = `${msg.progressPct}%`;
      txt.textContent = msg.label;
      return;
    }

    if (msg.type === 'VIEWSHED_RESULT') {
      const wrap = document.getElementById('worker-progress-wrap');
      if (msg.isFinal) {
        wrap.style.display = 'none';
      } else {
        wrap.style.display = 'block';
        document.getElementById('worker-progress-bar').style.width = '65%';
        document.getElementById('worker-progress-text').textContent =
          'משלים רזולוציה מלאה (720+ קרניים)...';
      }

      state.hGroundM = msg.hGroundM;
      state.avgRangeKm = msg.avgRangeKm;
      state.minRangeKm = msg.minRangeKm;
      state.maxObservedRangeKm = msg.maxObservedRangeKm;
      state.raySummaries = msg.raySummaries || [];

      renderViewshedPolygon(msg.polygonLatLons);
      updateReadoutsUI();
      updateInspectionRay(state.inspectAzimuthDeg, true);
      return;
    }

    if (msg.type === 'RAY_PROFILE_RESULT') {
      const p = msg.profile;
      renderInspectionRayGeometry(p);
      if (chartRenderer) {
        chartRenderer.setProfile(p);
      }
    }
  };
}

async function decodePngOnMainThread(pngArrayBuffer) {
  try {
    const blob = new Blob([pngArrayBuffer], { type: 'image/png' });
    const bmp = await createImageBitmap(blob);
    const canvas = document.getElementById('decode-fallback-canvas');
    const ctx = canvas.getContext('2d', { willReadFrequently: true });
    ctx.clearRect(0, 0, 256, 256);
    ctx.drawImage(bmp, 0, 0, 256, 256);
    bmp.close();
    return ctx.getImageData(0, 0, 256, 256).data.buffer;
  } catch (err) {
    return new Uint8ClampedArray(256 * 256 * 4).buffer;
  }
}

/**
 * Draw the 360° Viewshed Polygon on the Map with Double Border
 */
function renderViewshedPolygon(polygonLatLons) {
  if (!polygonLatLons || !polygonLatLons.length) return;

  const mainColor = state.curvatureMode === 'rf' ? '#22c55e' : '#f59e0b';
  polygonHaloLayer.setLatLngs(polygonLatLons);
  polygonMainLayer.setLatLngs(polygonLatLons);
  polygonMainLayer.setStyle({
    color: mainColor,
    fillColor: mainColor
  });
}

/**
 * Trigger full progressive 360° Viewshed calculation in Worker
 * (Called when station position, antenna height h_ant, or curvature k changes)
 */
function triggerFullViewshedCalculation() {
  if (!worker) return;
  const hUavM = state.hUavFt * FT_TO_M;
  worker.postMessage({
    type: 'COMPUTE_VIEWSHED',
    payload: {
      lat0: state.lat0,
      lon0: state.lon0,
      hAntM: state.hAntM,
      hUavM,
      maxRangeKm: state.rMaxKm,
      kFactor: state.kFactor
    }
  });
}

/**
 * Trigger instant 60 FPS analytical update when Aircraft Altitude (H_UAV) or Max Range (R_max) slider moves
 */
function triggerFast60FpsUpdate() {
  if (!worker) return;
  const hUavM = state.hUavFt * FT_TO_M;
  worker.postMessage({
    type: 'FAST_UPDATE_ALT_RANGE',
    payload: {
      hUavM,
      maxRangeKm: state.rMaxKm
    }
  });
}

/**
 * Update Inspection Ray & Request Cross-Section Profile for Azimuth `azimuthDeg`
 */
function updateInspectionRay(azimuthDeg, forceImmediate = false) {
  state.inspectAzimuthDeg = ((azimuthDeg % 360) + 360) % 360;

  // Immediate visual feedback using cached raySummaries
  if (state.raySummaries.length > 0) {
    let bestRay = state.raySummaries[0];
    let minDiff = 360;
    for (const r of state.raySummaries) {
      let diff = Math.abs(r.azimuthDeg - state.inspectAzimuthDeg);
      if (diff > 180) diff = 360 - diff;
      if (diff < minDiff) {
        minDiff = diff;
        bestRay = r;
      }
    }

    inspectRayLayer.setLatLngs([
      [state.lat0, state.lon0],
      [bestRay.lat, bestRay.lon]
    ]);

    const farEnd = destinationPoint(state.lat0, state.lon0, state.rMaxKm, bestRay.azimuthDeg);
    if (bestRay.rangeKm < state.rMaxKm - 0.5) {
      inspectShadowLayer.setLatLngs([
        [bestRay.lat, bestRay.lon],
        [farEnd.lat, farEnd.lon]
      ]);
    } else {
      inspectShadowLayer.setLatLngs([]);
    }

    const ridgeInfo =
      bestRay.blockingDistKm > 0.1
        ? `${bestRay.blockingDistKm.toFixed(1)} ק"מ (${Math.round(bestRay.blockingHeightM)} מ')`
        : 'אין חסימת רכס';
    const lockTag = state.inspectLocked ? ' [נעול 🔒]' : '';
    document.getElementById('ray-telemetry-text').textContent =
      `אזימוט: ${bestRay.azimuthDeg.toFixed(1)}°${lockTag} | טווח LOS: ${bestRay.rangeKm.toFixed(
        1
      )} ק"מ | רכס חוסם: ${ridgeInfo}`;
  }

  // Throttle detailed cross-section requests during rapid mouse movement
  const now = performance.now();
  if (!forceImmediate && now - state.lastProfileReqTime < 45) return;
  state.lastProfileReqTime = now;

  if (worker) {
    worker.postMessage({
      type: 'GET_RAY_PROFILE',
      payload: {
        azimuthDeg: state.inspectAzimuthDeg,
        hUavM: state.hUavFt * FT_TO_M,
        maxRangeKm: state.rMaxKm
      }
    });
  }
}

function renderInspectionRayGeometry(profile) {
  if (profile.blockLat !== null && profile.blockLon !== null && profile.blockingDistKm > 0.1) {
    if (!ridgeBlockerMarker) {
      ridgeBlockerMarker = L.marker([profile.blockLat, profile.blockLon], {
        interactive: false,
        icon: L.divIcon({
          className: 'station-marker-wrap',
          iconSize: [12, 12],
          iconAnchor: [6, 6],
          html: '<div class="ridge-blocker-dot" title="הרכס החוסם"></div>'
        })
      }).addTo(map);
    } else {
      ridgeBlockerMarker.setLatLng([profile.blockLat, profile.blockLon]);
    }
  } else if (ridgeBlockerMarker) {
    map.removeLayer(ridgeBlockerMarker);
    ridgeBlockerMarker = null;
  }
}

/**
 * Update DOM Readouts & Collapsed Header Summary
 */
function updateReadoutsUI() {
  const H0 = state.hGroundM + state.hAntM;
  const hUavM = Math.round(state.hUavFt * FT_TO_M);

  document.getElementById('readout-h-ground').textContent = `${state.hGroundM} מ'`;
  document.getElementById('readout-h0').textContent = `${H0} מ' ASL`;
  document.getElementById('readout-uav-m').textContent = `${hUavM.toLocaleString('en-US')} מ' ASL`;

  document.getElementById('stat-avg-range').textContent = `${state.avgRangeKm.toFixed(1)} ק"מ`;
  document.getElementById('stat-minmax-range').textContent = `${state.minRangeKm.toFixed(
    1
  )} / ${state.maxObservedRangeKm.toFixed(1)} ק"מ`;

  // Collapsed header compact summary: Aircraft altitude + Average range
  document.getElementById('collapsed-summary').textContent = `${state.hUavFt.toLocaleString(
    'en-US'
  )} ft | ממוצע: ${state.avgRangeKm.toFixed(1)} ק"מ`;
}

function syncInputsFromState() {
  document.getElementById('input-lat').value = state.lat0.toFixed(4);
  document.getElementById('input-lon').value = state.lon0.toFixed(4);
  document.getElementById('input-h-ant').value = state.hAntM;
  document.getElementById('slider-h-ant').value = Math.min(150, state.hAntM);
  document.getElementById('input-h-uav-ft').value = state.hUavFt;
  document.getElementById('slider-h-uav-ft').value = state.hUavFt;
  document.getElementById('input-r-max').value = state.rMaxKm;
  document.getElementById('slider-r-max').value = state.rMaxKm;
}

function toggleDrawer(forceState) {
  state.drawerCollapsed =
    typeof forceState === 'boolean' ? forceState : !state.drawerCollapsed;
  const drawerEl = document.getElementById('cross-section-drawer');
  const btnEl = document.getElementById('toggle-drawer-btn');
  drawerEl.classList.toggle('is-collapsed', state.drawerCollapsed);
  btnEl.textContent = state.drawerCollapsed ? 'חתך גבהים ▴' : 'סגור חתך ▾';

  if (!state.drawerCollapsed && chartRenderer) {
    setTimeout(() => chartRenderer.render(), 250);
  }
}

/**
 * Bind UI Controls
 */
function bindUIEvents() {
  // Collapse / Expand Floating Panel (`צמצם ▴` / `הרחב ▾`)
  const panelEl = document.getElementById('floating-panel');
  const toggleBtn = document.getElementById('toggle-panel-btn');
  document.getElementById('panel-header').addEventListener('click', () => {
    state.panelCollapsed = !state.panelCollapsed;
    panelEl.classList.toggle('is-collapsed', state.panelCollapsed);
    toggleBtn.textContent = state.panelCollapsed ? 'הרחב ▾' : 'צמצם ▴';
  });

  // Collapse / Expand Bottom Cross-Section Drawer
  document.getElementById('drawer-header').addEventListener('click', () => {
    toggleDrawer();
  });

  // Strictly 2 Map Layers: Topographic vs Orthophoto
  document.getElementById('btn-layer-topo').addEventListener('click', () => {
    state.mapType = 'topo';
    if (map.hasLayer(orthoLayerGroup)) map.removeLayer(orthoLayerGroup);
    if (!map.hasLayer(topoLayerGroup)) topoLayerGroup.addTo(map);
    document.getElementById('btn-layer-topo').classList.add('active');
    document.getElementById('btn-layer-ortho').classList.remove('active');
  });

  document.getElementById('btn-layer-ortho').addEventListener('click', () => {
    state.mapType = 'ortho';
    if (map.hasLayer(topoLayerGroup)) map.removeLayer(topoLayerGroup);
    if (!map.hasLayer(orthoLayerGroup)) orthoLayerGroup.addTo(map);
    document.getElementById('btn-layer-ortho').classList.add('active');
    document.getElementById('btn-layer-topo').classList.remove('active');
  });

  // 1. Station Lat/Lon & GPS
  const onLatLonInput = () => {
    const lat = parseFloat(document.getElementById('input-lat').value);
    const lon = parseFloat(document.getElementById('input-lon').value);
    if (Number.isFinite(lat) && Number.isFinite(lon)) {
      state.lat0 = Math.max(-85, Math.min(85, lat));
      state.lon0 = Math.max(-180, Math.min(180, lon));
      stationMarker.setLatLng([state.lat0, state.lon0]);
      map.panTo([state.lat0, state.lon0]);
      drawConcentricRangeRings();
      triggerFullViewshedCalculation();
    }
  };
  document.getElementById('input-lat').addEventListener('change', onLatLonInput);
  document.getElementById('input-lon').addEventListener('change', onLatLonInput);

  document.getElementById('btn-gps').addEventListener('click', () => {
    if (!navigator.geolocation) return;
    navigator.geolocation.getCurrentPosition(
      (pos) => {
        state.lat0 = pos.coords.latitude;
        state.lon0 = pos.coords.longitude;
        stationMarker.setLatLng([state.lat0, state.lon0]);
        map.setView([state.lat0, state.lon0], 9);
        syncInputsFromState();
        drawConcentricRangeRings();
        triggerFullViewshedCalculation();
      },
      () => {}
    );
  });

  // 2. Antenna Height AGL (h_ant)
  let antDebounce = null;
  const onAntChange = (val) => {
    const num = parseFloat(val);
    if (!Number.isFinite(num)) return;
    state.hAntM = Math.max(0, Math.min(500, Math.round(num)));
    document.getElementById('input-h-ant').value = state.hAntM;
    document.getElementById('slider-h-ant').value = Math.min(150, state.hAntM);
    updateReadoutsUI();
    if (antDebounce) clearTimeout(antDebounce);
    antDebounce = setTimeout(() => triggerFullViewshedCalculation(), 80);
  };
  document.getElementById('slider-h-ant').addEventListener('input', (e) => onAntChange(e.target.value));
  document.getElementById('input-h-ant').addEventListener('change', (e) => onAntChange(e.target.value));

  // 3. Aircraft Altitude in Feet (500 to 40,000 ft) -> 60 FPS Analytical Update!
  const onUavFtChange = (val) => {
    const num = parseFloat(val);
    if (!Number.isFinite(num)) return;
    state.hUavFt = Math.max(500, Math.min(40000, Math.round(num)));
    document.getElementById('input-h-uav-ft').value = state.hUavFt;
    document.getElementById('slider-h-uav-ft').value = state.hUavFt;
    updateReadoutsUI();
    triggerFast60FpsUpdate();
  };
  document.getElementById('slider-h-uav-ft').addEventListener('input', (e) => onUavFtChange(e.target.value));
  document.getElementById('input-h-uav-ft').addEventListener('change', (e) => onUavFtChange(e.target.value));

  // 4. Max Calculation Range (R_max: 10 to 200 km) -> 60 FPS Analytical Update!
  const onRMaxChange = (val) => {
    const num = parseFloat(val);
    if (!Number.isFinite(num)) return;
    state.rMaxKm = Math.max(10, Math.min(200, Math.round(num)));
    document.getElementById('input-r-max').value = state.rMaxKm;
    document.getElementById('slider-r-max').value = state.rMaxKm;
    drawConcentricRangeRings();
    triggerFast60FpsUpdate();
  };
  document.getElementById('slider-r-max').addEventListener('input', (e) => onRMaxChange(e.target.value));
  document.getElementById('input-r-max').addEventListener('change', (e) => onRMaxChange(e.target.value));

  // 5. Earth Curvature Model: RF (k = 4/3) vs Optical (k = 1.0)
  document.getElementById('btn-k-rf').addEventListener('click', () => {
    if (state.curvatureMode === 'rf') return;
    state.curvatureMode = 'rf';
    state.kFactor = 4 / 3;
    document.getElementById('btn-k-rf').classList.add('active');
    document.getElementById('btn-k-opt').classList.remove('active');
    triggerFullViewshedCalculation();
  });

  document.getElementById('btn-k-opt').addEventListener('click', () => {
    if (state.curvatureMode === 'opt') return;
    state.curvatureMode = 'opt';
    state.kFactor = 1.0;
    document.getElementById('btn-k-opt').classList.add('active');
    document.getElementById('btn-k-rf').classList.remove('active');
    triggerFullViewshedCalculation();
  });

  window.addEventListener('resize', () => {
    if (chartRenderer && !state.drawerCollapsed) {
      chartRenderer.render();
    }
  });
}

function initServiceWorker() {
  if ('serviceWorker' in navigator) {
    window.addEventListener('load', () => {
      navigator.serviceWorker.register('/sw.js').catch(() => {});
    });
  }
}

document.addEventListener('DOMContentLoaded', () => {
  initMap();
  chartRenderer = new LosChartRenderer(document.getElementById('los-chart-canvas'));
  initWorker();
  bindUIEvents();
  syncInputsFromState();
  initServiceWorker();
  triggerFullViewshedCalculation();
});
