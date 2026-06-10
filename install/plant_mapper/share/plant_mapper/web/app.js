// Configuration loaded dynamically from server
let mapConfig = {
    resolution: 0.05,
    origin_x: -1.325,
    origin_y: -4.239,
    width_pixels: 88,
    height_pixels: 112,
    map_rotation: 0
};

// Planned waypoints (local list before launch)
let plannedWaypoints = [];
let scannedPlants = [];
let robotPose = { x: 0.0, y: 0.0, yaw: 0.0 };
let currentWpIndex = -1;
let spreadCoef = 10.0; // Spread probability default 10%
let scanPestChance = 80; // Default chance scanned plant has pest (80%)
let pestGrid = [];
let timeTicks = 0;
let processedPlants = new Set(); // Tracks row,col coordinates of processed scan points to lock their rolled pest state
let waypointMode = 'scan'; // 'scan' or 'spray'
let autonomousSprayEnabled = true;
let isAutonomousMissionActive = false;

// DOM Elements
const mapContainer = document.getElementById('map-container');
const mapOverlay = document.getElementById('map-overlay');
const mapImage = document.getElementById('map-image');
const coordsTooltip = document.getElementById('coords-tooltip');
const waypointList = document.getElementById('waypoint-list');
const terminalLogs = document.getElementById('terminal-logs');
const selectRotation = document.getElementById('select-rotation');
const heatmapCanvas = document.getElementById('heatmap-canvas');
const inputSpreadCoef = document.getElementById('input-spread-coef');
const valSpreadCoef = document.getElementById('val-spread-coef');
const inputScanPestChance = document.getElementById('input-scan-pest-chance');
const valScanPestChance = document.getElementById('val-scan-pest-chance');
const btnTick1 = document.getElementById('btn-tick-1');
const btnTick10 = document.getElementById('btn-tick-10');
const btnTick100 = document.getElementById('btn-tick-100');
const valTimeTicks = document.getElementById('val-time-ticks');
const checkAutonomousSprayEl = document.getElementById('check-autonomous-spray');
const checkSprayModeEl = document.getElementById('check-spray-mode');

// Telemetry Elements
const telX = document.getElementById('tel-x');
const telY = document.getElementById('tel-y');
const telYaw = document.getElementById('tel-yaw');
const telCell = document.getElementById('tel-cell');
const navStatusText = document.getElementById('nav-status-text');

// Buttons
const btnLaunch = document.getElementById('btn-launch');
const btnClearWps = document.getElementById('btn-clear-wps');
const btnManualScan = document.getElementById('btn-manual-scan');
const btnReset = document.getElementById('btn-reset');

// Initialize Dashboard
document.addEventListener('DOMContentLoaded', () => {
    mapImage.onload = () => {
        adjustMapContainerSize();
        redrawOverlay();
    };
    if (mapImage.complete && mapImage.naturalWidth !== 0) {
        adjustMapContainerSize();
        redrawOverlay();
    }
    window.addEventListener('resize', adjustMapContainerSize);
    connectToEventStream();
    setupEventHandlers();
    addLog('System', 'Map Dashboard initialized. Listening for telemetry stream...', 'system');
});

// Map width and height in physical meters
function getMapMeters() {
    return {
        width: mapConfig.width_pixels * mapConfig.resolution,
        height: mapConfig.height_pixels * mapConfig.resolution
    };
}

// Adjust map container size dynamically maintaining aspect ratio
function adjustMapContainerSize() {
    if (!mapConfig || !mapConfig.width_pixels) return;
    
    const wrapper = document.querySelector('.map-wrapper');
    if (!wrapper) return;
    
    const wrapperRect = wrapper.getBoundingClientRect();
    const maxW = wrapperRect.width - 20;
    const maxH = wrapperRect.height - 20;
    
    const aspect = mapConfig.width_pixels / mapConfig.height_pixels;
    let targetW = maxW;
    let targetH = maxW / aspect;
    
    if (targetH > maxH) {
        targetH = maxH;
        targetW = maxH * aspect;
    }
    
    mapContainer.style.width = `${targetW}px`;
    mapContainer.style.height = `${targetH}px`;
}

// Convert click percentage on map overlay to physical (x, y) coordinates
function pctToPhysical(pctX, pctY) {
    const meters = getMapMeters();
    const rotation = mapConfig.map_rotation || 0;
    
    let x, y;
    
    if (rotation === 90) {
        // 90° CW: Horizontal is Y, Vertical is X
        x = mapConfig.origin_x + pctY * meters.height;
        y = mapConfig.origin_y + pctX * meters.width;
    } else if (rotation === 180) {
        // 180°: Horizontal is -X, Vertical is Y
        x = mapConfig.origin_x + (1 - pctX) * meters.width;
        y = mapConfig.origin_y + pctY * meters.height;
    } else if (rotation === 270) {
        // 90° CCW (270° CW): Horizontal is -Y, Vertical is -X
        x = mapConfig.origin_x + (1 - pctY) * meters.height;
        y = mapConfig.origin_y + (1 - pctX) * meters.width;
    } else {
        // 0°: Horizontal is X, Vertical is -Y
        x = mapConfig.origin_x + pctX * meters.width;
        y = mapConfig.origin_y + (1 - pctY) * meters.height;
    }
    
    return {
        x: Number(x.toFixed(3)),
        y: Number(y.toFixed(3))
    };
}

// Convert physical (x, y) to percentage coordinates (0.0 to 1.0)
function physicalToPct(x, y) {
    const meters = getMapMeters();
    const rotation = mapConfig.map_rotation || 0;
    
    let pctX, pctY;
    
    if (rotation === 90) {
        // 90° CW: X maps to vertical, Y maps to horizontal
        pctX = (y - mapConfig.origin_y) / meters.width;
        pctY = (x - mapConfig.origin_x) / meters.height;
    } else if (rotation === 180) {
        // 180°
        pctX = 1.0 - ((x - mapConfig.origin_x) / meters.width);
        pctY = (y - mapConfig.origin_y) / meters.height;
    } else if (rotation === 270) {
        // 90° CCW (270° CW)
        pctX = 1.0 - ((y - mapConfig.origin_y) / meters.width);
        pctY = 1.0 - ((x - mapConfig.origin_x) / meters.height);
    } else {
        // 0°
        pctX = (x - mapConfig.origin_x) / meters.width;
        pctY = 1.0 - ((y - mapConfig.origin_y) / meters.height);
    }
    
    return {
        x: Math.max(0, Math.min(1.0, pctX)),
        y: Math.max(0, Math.min(1.0, pctY))
    };
}

// Set up UI Event Handlers
function setupEventHandlers() {
    // Rotation change event handler
    selectRotation.addEventListener('change', (e) => {
        const angle = parseInt(e.target.value);
        addLog('UI', `Requesting map rotation to ${angle}°...`, 'info');
        
        fetch('/api/set_rotation', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ rotation: angle })
        })
        .then(res => res.json())
        .then(data => {
            if (data.status === 'ok') {
                addLog('Server', `Map rotation successfully set to ${angle}°`, 'success');
            } else {
                addLog('Server', 'Failed to update map rotation: ' + data.message, 'error');
            }
        })
        .catch(err => {
            addLog('Server', 'Network error setting rotation: ' + err, 'error');
        });
    });

    // Hover coordinate calculation with plant detection
    mapOverlay.addEventListener('mousemove', (e) => {
        const rect = mapOverlay.getBoundingClientRect();
        const pctX = (e.clientX - rect.left) / rect.width;
        const pctY = (e.clientY - rect.top) / rect.height;
        
        const coords = pctToPhysical(pctX, pctY);
        
        // Find if cursor is close to any scanned plant (within 0.25 meters)
        let hoveredPlant = null;
        for (const plant of scannedPlants) {
            const dist = Math.hypot(coords.x - plant.x, coords.y - plant.y);
            if (dist < 0.25) {
                hoveredPlant = plant;
                break;
            }
        }
        
        if (hoveredPlant) {
            coordsTooltip.innerText = `Plant Details -> Pest: ${hoveredPlant.pest.toFixed(1)}%, Dryness: ${hoveredPlant.dryness.toFixed(1)}% | Pose -> X: ${coords.x.toFixed(2)}m, Y: ${coords.y.toFixed(2)}m`;
        } else {
            coordsTooltip.innerText = `Cursor Pose -> X: ${coords.x.toFixed(2)}m, Y: ${coords.y.toFixed(2)}m (Click to place WP)`;
        }
    });

    mapOverlay.addEventListener('mouseleave', () => {
        coordsTooltip.innerText = 'Click anywhere on the map to place a waypoint';
    });

    // Click map overlay -> Add planned waypoint
    mapOverlay.addEventListener('click', (e) => {
        if (e.target !== mapOverlay) return;

        const rect = mapOverlay.getBoundingClientRect();
        const pctX = (e.clientX - rect.left) / rect.width;
        const pctY = (e.clientY - rect.top) / rect.height;
        
        const coords = pctToPhysical(pctX, pctY);
        
        plannedWaypoints.push({ x: coords.x, y: coords.y, type: waypointMode });
        const typeLabel = waypointMode === 'spray' ? 'Spray' : 'Scan';
        addLog('UI', `${typeLabel} waypoint planned at physical coords: (x: ${coords.x}, y: ${coords.y})`, 'info');
        
        updateWaypointsUI();
        redrawOverlay();
    });

    // Launch Mission
    btnLaunch.addEventListener('click', () => {
        if (plannedWaypoints.length === 0) {
            addLog('UI', 'Cannot launch mission: No waypoints planned.', 'warn');
            alert('Please click on the map to add waypoints first.');
            return;
        }
        
        addLog('UI', 'Sending waypoint mission to Navigator node...', 'info');
        
        fetch('/api/send_waypoints', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(plannedWaypoints)
        })
        .then(res => res.json())
        .then(data => {
            if (data.status === 'ok') {
                addLog('Server', 'Mission started. Navigator routing waypoints.', 'success');
            } else {
                addLog('Server', 'Failed to start mission: ' + data.message, 'error');
            }
        })
        .catch(err => {
            addLog('Server', 'Network error launching mission: ' + err, 'error');
        });
    });

    // Clear waypoints
    btnClearWps.addEventListener('click', () => {
        plannedWaypoints = [];
        updateWaypointsUI();
        redrawOverlay();
        addLog('UI', 'Cleared planned waypoints.', 'info');
    });

    // Manual Scan
    btnManualScan.addEventListener('click', () => {
        addLog('UI', 'Requesting manual crop scan at current pose...', 'info');
        fetch('/api/trigger_scan', { method: 'POST' })
        .then(res => res.json())
        .then(data => {
            if (data.status === 'ok') {
                addLog('Server', `Manual scan successful at robot location: (x: ${data.x.toFixed(2)}, y: ${data.y.toFixed(2)})`, 'success');
            } else {
                addLog('Server', 'Failed to trigger scan: ' + data.message, 'error');
            }
        })
        .catch(err => {
            addLog('Server', 'Network error triggering scan: ' + err, 'error');
        });
    });

    // Reset Farm Twin
    btnReset.addEventListener('click', () => {
        if (!confirm('Are you sure you want to reset the digital twin model? All plant scan records will be deleted.')) return;
        
        addLog('UI', 'Requesting twin reset...', 'info');
        fetch('/api/clear', { method: 'POST' })
        .then(res => res.json())
        .then(data => {
            if (data.status === 'ok') {
                plannedWaypoints = [];
                scannedPlants = [];
                pestGrid = []; // Reset simulation grid too!
                timeTicks = 0; // Reset time ticks counter
                processedPlants.clear(); // Clear processed scans
                valTimeTicks.innerText = '0 T';
                updateWaypointsUI();
                redrawOverlay();
                addLog('Server', 'Twin model reset completed.', 'success');
            }
        })
        .catch(err => {
            addLog('Server', 'Network error resetting twin: ' + err, 'error');
        });
    });



    // Sim tick buttons handlers
    btnTick1.addEventListener('click', () => {
        runSimulationTicks(1);
    });

    btnTick10.addEventListener('click', () => {
        runSimulationTicks(10);
    });

    btnTick100.addEventListener('click', () => {
        runSimulationTicks(100);
    });

    // Spread coefficient slider input handler
    inputSpreadCoef.addEventListener('input', (e) => {
        spreadCoef = parseFloat(e.target.value);
        valSpreadCoef.innerText = `${spreadCoef.toFixed(1)} %`;
    });

    // Scan pest chance slider input handler
    inputScanPestChance.addEventListener('input', (e) => {
        scanPestChance = parseInt(e.target.value);
        valScanPestChance.innerText = `${scanPestChance} %`;
    });

    // Spray mode and autonomous checkboxes change handlers
    checkAutonomousSprayEl.addEventListener('change', (e) => {
        autonomousSprayEnabled = e.target.checked;
        addLog('UI', `Autonomous Spray ${autonomousSprayEnabled ? 'Enabled' : 'Disabled'}.`, 'info');
        if (autonomousSprayEnabled) {
            checkAutonomousSpray();
        }
    });

    checkSprayModeEl.addEventListener('change', (e) => {
        waypointMode = e.target.checked ? 'spray' : 'scan';
        addLog('UI', `Waypoint Mode switched to ${waypointMode.toUpperCase()}`, 'info');
    });

    // Initialize UI states explicitly
    inputSpreadCoef.value = spreadCoef;
    valSpreadCoef.innerText = `${spreadCoef.toFixed(1)} %`;
    inputScanPestChance.value = scanPestChance;
    valScanPestChance.innerText = `${scanPestChance} %`;
    valTimeTicks.innerText = `${timeTicks} T`;
    checkAutonomousSprayEl.checked = autonomousSprayEnabled;
    checkSprayModeEl.checked = (waypointMode === 'spray');
}

// Redraw all markers on top of the map image overlay
function redrawOverlay() {
    mapOverlay.querySelectorAll('.waypoint-marker, .plant-marker, .robot-marker').forEach(el => el.remove());

    // 1. Draw planned waypoints
    plannedWaypoints.forEach((wp, index) => {
        const pct = physicalToPct(wp.x, wp.y);
        const marker = document.createElement('div');
        marker.className = 'waypoint-marker';
        if (wp.type === 'spray') {
            marker.classList.add('spray');
        }
        if (index === currentWpIndex) {
            marker.classList.add('active');
        }
        marker.style.left = `${pct.x * 100}%`;
        marker.style.top = `${pct.y * 100}%`;
        marker.innerText = index + 1;
        const typeLabel = wp.type === 'spray' ? 'Spray' : 'Scan';
        marker.title = `Waypoint ${index + 1} (${typeLabel}): (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})`;
        
        mapOverlay.appendChild(marker);
    });

    // 3. Draw robot pose
    const pctRobot = physicalToPct(robotPose.x, robotPose.y);
    const robotMarker = document.createElement('div');
    robotMarker.className = 'robot-marker';
    robotMarker.style.left = `${pctRobot.x * 100}%`;
    robotMarker.style.top = `${pctRobot.y * 100}%`;
    
    // Rotate indicator based on yaw orientation and map rotation
    const rotation = mapConfig.map_rotation || 0;
    let screenYaw = robotPose.yaw;
    if (rotation === 90) {
        screenYaw = Math.PI / 2 - robotPose.yaw;
    } else if (rotation === 180) {
        screenYaw = Math.PI + robotPose.yaw;
    } else if (rotation === 270) {
        screenYaw = -Math.PI / 2 - robotPose.yaw;
    }
    
    robotMarker.style.transform = `translate(-50%, -50%) rotate(${screenYaw}rad)`;
    
    mapOverlay.appendChild(robotMarker);

    drawHeatmap();
}

// Update waypoints list UI in sidebar
function updateWaypointsUI() {
    waypointList.innerHTML = '';
    
    if (plannedWaypoints.length === 0) {
        const empty = document.createElement('li');
        empty.className = 'empty-state';
        empty.innerText = 'No waypoints planned. Click on the map to plan a route.';
        waypointList.appendChild(empty);
        return;
    }
    
    plannedWaypoints.forEach((wp, index) => {
        const li = document.createElement('li');
        li.className = 'waypoint-item';
        const typeLabel = wp.type === 'spray' ? 'Spray' : 'Scan';
        
        li.innerHTML = `
            <div class="waypoint-info">
                <span class="wp-badge ${wp.type === 'spray' ? 'spray-badge' : ''}">${index + 1}</span>
                <span class="wp-coords">WP ${index + 1} (${typeLabel}): (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})</span>
            </div>
            <button class="btn-remove-wp" onclick="removeWaypoint(${index})">✕</button>
        `;
        
        waypointList.appendChild(li);
    });
}

// Remove waypoint by index
window.removeWaypoint = function(index) {
    plannedWaypoints.splice(index, 1);
    addLog('UI', `Removed waypoint ${index + 1}`, 'info');
    updateWaypointsUI();
    redrawOverlay();
};

// Update navigation status in sidebar
function updateNavStatus(status, activeIndex) {
    currentWpIndex = activeIndex;
    
    navStatusText.innerText = status.toUpperCase();
    navStatusText.className = 'status-banner';
    
    const statusLower = status.toLowerCase();
    if (statusLower.includes('idle')) {
        navStatusText.classList.add('idle');
        isAutonomousMissionActive = false;
        setTimeout(checkAutonomousSpray, 500);
    } else if (statusLower.includes('navigating')) {
        navStatusText.classList.add('navigating');
    } else if (statusLower.includes('scanning') || statusLower.includes('arrived') || statusLower.includes('spraying')) {
        navStatusText.classList.add('scanning');
    } else if (statusLower.includes('complete') || statusLower.includes('finished')) {
        navStatusText.classList.add('complete');
        isAutonomousMissionActive = false;
        setTimeout(checkAutonomousSpray, 500);
    } else {
        navStatusText.classList.add('idle');
    }

    if (activeIndex !== undefined && activeIndex !== null && activeIndex >= 0 && activeIndex < plannedWaypoints.length) {
        const wp = plannedWaypoints[activeIndex];
        const typeLabel = wp && wp.type === 'spray' ? 'Spray' : 'WP';
        telCell.innerText = `${typeLabel} ${activeIndex + 1}`;
    } else {
        telCell.innerText = 'None';
    }

    redrawOverlay();
}

// Add logs to local terminal UI
function addLog(source, message, level = 'info') {
    const timestamp = new Date().toLocaleTimeString();
    const line = document.createElement('div');
    line.className = `log-line ${level}`;
    line.innerHTML = `<span>[${timestamp}]</span> <strong>[${source}]</strong> ${message}`;
    
    terminalLogs.appendChild(line);
    terminalLogs.scrollTop = terminalLogs.scrollHeight;
    
    while (terminalLogs.childElementCount > 100) {
        terminalLogs.removeChild(terminalLogs.firstChild);
    }
}

// Set up SSE EventStream Connection
function connectToEventStream() {
    const eventSource = new EventSource('/api/stream');
    
    eventSource.onopen = () => {
        document.querySelector('.status-dot').className = 'status-dot online';
        addLog('Server', 'Live telemetry stream connected.', 'success');
    };
    
    eventSource.onerror = () => {
        document.querySelector('.status-dot').className = 'status-dot';
        addLog('Server', 'Stream disconnected. Reconnecting...', 'error');
    };
    
    eventSource.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            let needsRedraw = false;
            
            // Sync map dimensions and metadata
            if (data.config) {
                const rotationChanged = data.config.map_rotation !== mapConfig.map_rotation;
                const sizeChanged = data.config.width_pixels !== mapConfig.width_pixels || data.config.height_pixels !== mapConfig.height_pixels;
                
                if (sizeChanged || rotationChanged || data.config.resolution !== mapConfig.resolution) {
                    mapConfig = data.config;
                    // Append cache-buster to reload PNG
                    mapImage.src = `map.png?t=${Date.now()}`;
                    selectRotation.value = mapConfig.map_rotation;
                    addLog('System', `Map updated: ${mapConfig.width_pixels}x${mapConfig.height_pixels} px @ ${mapConfig.resolution}m/px (Rotation: ${mapConfig.map_rotation}°)`, 'system');
                }
                mapConfig = data.config;
                selectRotation.value = mapConfig.map_rotation;
            }
            
            // Sync robot position
            if (data.robot_pose) {
                robotPose = data.robot_pose;
                telX.innerText = `${robotPose.x.toFixed(2)} m`;
                telY.innerText = `${robotPose.y.toFixed(2)} m`;
                telYaw.innerText = `${(robotPose.yaw * 180 / Math.PI).toFixed(1)}°`;
                needsRedraw = true;
            }
            
            // Sync scanned plants
            if (data.plants) {
                scannedPlants = data.plants;
                syncPlantsToPestGrid(true); // Force sync new scans to grid
                needsRedraw = true;
                checkAutonomousSpray();
            }

            // Handle pesticide spray event
            if (data.spray_event) {
                const cols = mapConfig.width_pixels;
                const rows = mapConfig.height_pixels;
                const pct = physicalToPct(data.spray_event.x, data.spray_event.y);
                const col = Math.round(pct.x * cols);
                const row = Math.round(pct.y * rows);

                if (col >= 0 && col < cols && row >= 0 && row < rows) {
                    const cleared = clearPestInRadius(row, col);
                    addLog('Robot', `Sprayed pesticide at (x: ${data.spray_event.x.toFixed(2)}, y: ${data.spray_event.y.toFixed(2)}). Cleared ${cleared} infected pixels.`, 'success');
                }
                needsRedraw = true;
            }
            
            // Sync navigation status and index
            if (data.nav_status !== undefined) {
                updateNavStatus(data.nav_status, data.current_waypoint_index);
            }
            
            // Print log lines
            if (data.log) {
                addLog(data.log.source, data.log.message, data.log.level);
            }
            
            if (needsRedraw) {
                redrawOverlay();
            }
        } catch (e) {
            console.error('Error handling SSE event data:', e);
        }
    };
}

// Helper to sync newly scanned plants into pestGrid
function syncPlantsToPestGrid(force = false) {
    if (!mapConfig || !mapConfig.width_pixels) return;
    const cols = mapConfig.width_pixels;
    const rows = mapConfig.height_pixels;

    if (pestGrid.length !== rows || (pestGrid[0] && pestGrid[0].length !== cols)) {
        pestGrid = Array.from({ length: rows }, () => new Uint8Array(cols));
    }

    for (const plant of scannedPlants) {
        const pct = physicalToPct(plant.x, plant.y);
        const col = Math.round(pct.x * cols);
        const row = Math.round(pct.y * rows);
        if (col >= 0 && col < cols && row >= 0 && row < rows) {
            const key = `${row},${col}`;
            if (!processedPlants.has(key)) {
                processedPlants.add(key);
                if (Math.random() * 100 < scanPestChance) {
                    pestGrid[row][col] = 1;
                } else {
                    pestGrid[row][col] = 0;
                }
            }
        }
    }
}



// Perform a specified number of simulation steps and redraw at the end
function runSimulationTicks(n) {
    if (!mapConfig || !mapConfig.width_pixels) return;

    const cols = mapConfig.width_pixels;
    const rows = mapConfig.height_pixels;

    // Ensure pestGrid is allocated and matches map dimensions
    if (pestGrid.length !== rows || (pestGrid[0] && pestGrid[0].length !== cols)) {
        pestGrid = Array.from({ length: rows }, () => new Uint8Array(cols));
    }

    // Sync scanned plants first
    syncPlantsToPestGrid(false);

    // Perform n steps of grid cellular updates
    for (let step = 0; step < n; step++) {
        // Create next state grid
        const nextPestGrid = Array.from({ length: rows }, () => new Uint8Array(cols));
        for (let r = 0; r < rows; r++) {
            for (let c = 0; c < cols; c++) {
                nextPestGrid[r][c] = pestGrid[r][c];
            }
        }

        for (let r = 0; r < rows; r++) {
            for (let c = 0; c < cols; c++) {
                if (pestGrid[r][c] === 1) {
                    // Already has pest: stays infected
                    nextPestGrid[r][c] = 1;
                } else {
                    // Healthy pixel: check if any of the 8 neighbors has pest
                    let hasInfectedNeighbor = false;
                    for (let dr = -1; dr <= 1; dr++) {
                        for (let dc = -1; dc <= 1; dc++) {
                            if (dr === 0 && dc === 0) continue;
                            const nr = r + dr;
                            const nc = c + dc;
                            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                                if (pestGrid[nr][nc] === 1) {
                                    hasInfectedNeighbor = true;
                                    break;
                                }
                            }
                        }
                        if (hasInfectedNeighbor) break;
                    }

                    if (hasInfectedNeighbor) {
                        // Spread probability: spreadCoef % (from the slider, 0 to 20%)
                        if (Math.random() * 100 < spreadCoef) {
                            nextPestGrid[r][c] = 1;
                        }
                    }
                }
            }
        }
        pestGrid = nextPestGrid;
    }

    // Increment time ticks counter and update label
    timeTicks += n;
    valTimeTicks.innerText = `${timeTicks} T`;

    // Trigger autonomous spray evaluation
    checkAutonomousSpray();

    // Trigger single redraw at the end of all steps
    drawHeatmap();
}

// Draw map pixels, pest spread, and mapped plants on canvas
function drawHeatmap() {
    const ctx = heatmapCanvas.getContext('2d');
    
    // Disable image smoothing to ensure crisp pixel rendering
    ctx.imageSmoothingEnabled = false;
    ctx.mozImageSmoothingEnabled = false;
    ctx.webkitImageSmoothingEnabled = false;
    ctx.msImageSmoothingEnabled = false;
    
    ctx.clearRect(0, 0, heatmapCanvas.width, heatmapCanvas.height);

    const cols = mapConfig.width_pixels;
    const rows = mapConfig.height_pixels;

    if (heatmapCanvas.width !== cols || heatmapCanvas.height !== rows) {
        heatmapCanvas.width = cols;
        heatmapCanvas.height = rows;
    }

    // 1. Draw the map pixels from mapImage
    if (mapImage.complete && mapImage.naturalWidth !== 0) {
        ctx.drawImage(mapImage, 0, 0, cols, rows);
    }

    // Initialize/sync pestGrid size if needed
    if (pestGrid.length !== rows || (pestGrid[0] && pestGrid[0].length !== cols)) {
        pestGrid = Array.from({ length: rows }, () => new Uint8Array(cols));
    }

    // Ensure all scanned plants are synchronized into pestGrid
    syncPlantsToPestGrid(false);

    // 2. Render pest pixels (infected areas)
    ctx.fillStyle = '#00ff66';
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            if (pestGrid[r][c] === 1) {
                ctx.fillRect(c, r, 1, 1);
            }
        }
    }

    // 3. Render scanned plants: green if has pest, yellow if does not have pest
    for (const plant of scannedPlants) {
        const pct = physicalToPct(plant.x, plant.y);
        const col = Math.round(pct.x * cols);
        const row = Math.round(pct.y * rows);

        if (col >= 0 && col < cols && row >= 0 && row < rows) {
            if (pestGrid[row][col] === 1) {
                ctx.fillStyle = '#00ff66';
            } else {
                ctx.fillStyle = '#ffff00';
            }
            ctx.fillRect(col, row, 1, 1);
        }
    }
}

// Clear pest in 7x7 square around a grid coordinate
function clearPestInRadius(row, col) {
    const cols = mapConfig.width_pixels;
    const rows = mapConfig.height_pixels;
    let clearedCount = 0;

    for (let dr = -3; dr <= 3; dr++) {
        for (let dc = -3; dc <= 3; dc++) {
            const r = row + dr;
            const c = col + dc;
            if (r >= 0 && r < rows && c >= 0 && c < cols) {
                if (pestGrid[r][c] === 1) {
                    pestGrid[r][c] = 0;
                    clearedCount++;
                }
            }
        }
    }
    return clearedCount;
}

// Find window of 7x7 with > 50% pest density (>= 25 out of 49 cells)
function findInfectionCluster() {
    const cols = mapConfig.width_pixels;
    const rows = mapConfig.height_pixels;
    let maxCount = 0;
    let targetRow = -1;
    let targetCol = -1;

    for (let r = 3; r < rows - 3; r++) {
        for (let c = 3; c < cols - 3; c++) {
            let count = 0;
            for (let dr = -3; dr <= 3; dr++) {
                for (let dc = -3; dc <= 3; dc++) {
                    if (pestGrid[r + dr][c + dc] === 1) {
                        count++;
                    }
                }
            }
            if (count >= 25 && count > maxCount) {
                maxCount = count;
                targetRow = r;
                targetCol = c;
            }
        }
    }
    if (targetRow !== -1 && targetCol !== -1) {
        return { row: targetRow, col: targetCol, count: maxCount };
    }
    return null;
}

// Check if there is any infection cluster and trigger autonomous spray
function checkAutonomousSpray() {
    if (!autonomousSprayEnabled) return;
    if (isAutonomousMissionActive) return;

    const cluster = findInfectionCluster();
    if (cluster) {
        const cols = mapConfig.width_pixels;
        const rows = mapConfig.height_pixels;
        
        // Convert center cell of cluster to physical coordinates
        const pctX = (cluster.col + 0.5) / cols;
        const pctY = (cluster.row + 0.5) / rows;
        const phys = pctToPhysical(pctX, pctY);

        addLog('System', `Autonomous Trigger: Infection cluster detected (density: ${((cluster.count / 49) * 100).toFixed(1)}%) at (x: ${phys.x.toFixed(2)}, y: ${phys.y.toFixed(2)}). Dispatched robot to spray.`, 'warning');
        
        isAutonomousMissionActive = true;
        const sprayWp = { x: phys.x, y: phys.y, type: 'spray' };

        fetch('/api/send_waypoints', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify([sprayWp])
        })
        .then(res => res.json())
        .then(data => {
            if (data.status === 'ok') {
                addLog('Server', 'Autonomous spray mission launched.', 'success');
            } else {
                addLog('Server', 'Autonomous spray launch failed: ' + data.message, 'error');
                isAutonomousMissionActive = false;
            }
        })
        .catch(err => {
            addLog('Server', 'Autonomous spray launch network error: ' + err, 'error');
            isAutonomousMissionActive = false;
        });
    }
}
