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
let pestVariance = 0.5;
let spreadCoef = 1.0;
let pestGrid = [];
let timeTicks = 0;

// DOM Elements
const mapContainer = document.getElementById('map-container');
const mapOverlay = document.getElementById('map-overlay');
const mapImage = document.getElementById('map-image');
const coordsTooltip = document.getElementById('coords-tooltip');
const waypointList = document.getElementById('waypoint-list');
const terminalLogs = document.getElementById('terminal-logs');
const selectRotation = document.getElementById('select-rotation');
const heatmapCanvas = document.getElementById('heatmap-canvas');
const inputVariance = document.getElementById('input-variance');
const valVariance = document.getElementById('val-variance');
const inputSpreadCoef = document.getElementById('input-spread-coef');
const valSpreadCoef = document.getElementById('val-spread-coef');
const btnTick1 = document.getElementById('btn-tick-1');
const btnTick10 = document.getElementById('btn-tick-10');
const btnTick100 = document.getElementById('btn-tick-100');
const valTimeTicks = document.getElementById('val-time-ticks');

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
        
        plannedWaypoints.push({ x: coords.x, y: coords.y });
        addLog('UI', `Waypoint planned at physical coords: (x: ${coords.x}, y: ${coords.y})`, 'info');
        
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

    // Pest variance slider input handler
    inputVariance.addEventListener('input', (e) => {
        pestVariance = parseFloat(e.target.value);
        valVariance.innerText = `${pestVariance.toFixed(2)} m²`;
        drawHeatmap();
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
        valSpreadCoef.innerText = `${spreadCoef.toFixed(2)} %`;
    });

    // Initialize UI states explicitly
    inputVariance.value = pestVariance;
    valVariance.innerText = `${pestVariance.toFixed(2)} m²`;
    inputSpreadCoef.value = spreadCoef;
    valSpreadCoef.innerText = `${spreadCoef.toFixed(2)} %`;
    valTimeTicks.innerText = `${timeTicks} T`;
}

// Redraw all markers on top of the map image overlay
function redrawOverlay() {
    mapOverlay.querySelectorAll('.waypoint-marker, .plant-marker, .robot-marker').forEach(el => el.remove());

    // 1. Draw planned waypoints
    plannedWaypoints.forEach((wp, index) => {
        const pct = physicalToPct(wp.x, wp.y);
        const marker = document.createElement('div');
        marker.className = 'waypoint-marker';
        if (index === currentWpIndex) {
            marker.classList.add('active');
        }
        marker.style.left = `${pct.x * 100}%`;
        marker.style.top = `${pct.y * 100}%`;
        marker.innerText = index + 1;
        marker.title = `Waypoint ${index + 1}: (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})`;
        
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
        
        li.innerHTML = `
            <div class="waypoint-info">
                <span class="wp-badge">${index + 1}</span>
                <span class="wp-coords">WP ${index + 1}: (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})</span>
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
    
    if (status.toLowerCase().includes('idle')) {
        navStatusText.classList.add('idle');
    } else if (status.toLowerCase().includes('navigating')) {
        navStatusText.classList.add('navigating');
    } else if (status.toLowerCase().includes('scanning') || status.toLowerCase().includes('arrived')) {
        navStatusText.classList.add('scanning');
    } else if (status.toLowerCase().includes('complete') || status.toLowerCase().includes('finished')) {
        navStatusText.classList.add('complete');
    } else {
        navStatusText.classList.add('idle');
    }

    if (activeIndex !== undefined && activeIndex !== null && activeIndex >= 0 && activeIndex < plannedWaypoints.length) {
        telCell.innerText = `Waypoint ${activeIndex + 1}`;
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
            }

            // Sync pest grid from Python simulation
            if (data.pest_grid) {
                pestGrid = data.pest_grid;
                needsRedraw = true;
            }

            // Sync simulation time ticks increment
            if (data.time_ticks_increment !== undefined) {
                timeTicks += data.time_ticks_increment;
                valTimeTicks.innerText = `${timeTicks} T`;
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
        pestGrid = Array.from({ length: rows }, () => new Float32Array(cols));
    }

    for (const plant of scannedPlants) {
        const pct = physicalToPct(plant.x, plant.y);
        const col = Math.round(pct.x * cols);
        const row = Math.round(pct.y * rows);
        if (col >= 0 && col < cols && row >= 0 && row < rows) {
            // Force override if true, or set if currently empty
            if (force || pestGrid[row][col] === 0) {
                pestGrid[row][col] = plant.pest;
            }
        }
    }
}

// Calculate standard 2D Gaussian spread at each cell of the grid using active pest pixels as sources
function calculateGaussianSpread(cols, rows) {
    const grid = Array.from({ length: rows }, () => new Float32Array(cols));
    
    // Ensure all scanned plants are synchronized into pestGrid
    syncPlantsToPestGrid();

    // Iterate through all cells to find sources of pest
    for (let r_src = 0; r_src < rows; r_src++) {
        for (let c_src = 0; c_src < cols; c_src++) {
            const pest = pestGrid[r_src][c_src];
            if (pest <= 0.01) continue; // Only cells with pest act as sources

            // Convert source cell coordinates to physical (xp, yp)
            const pctX_src = (c_src + 0.5) / cols;
            const pctY_src = (r_src + 0.5) / rows;
            const phys_src = pctToPhysical(pctX_src, pctY_src);
            const xp = phys_src.x;
            const yp = phys_src.y;

            // Cutoff radius where relative distribution value y >= 0.05
            const r_cutoff = Math.sqrt(-2.0 * pestVariance * Math.log(0.05));

            // Get corners in grid space to bound destination iteration
            const corners = [
                physicalToPct(xp - r_cutoff, yp - r_cutoff),
                physicalToPct(xp + r_cutoff, yp - r_cutoff),
                physicalToPct(xp - r_cutoff, yp + r_cutoff),
                physicalToPct(xp + r_cutoff, yp + r_cutoff)
            ];

            let minPctX = 1.0, maxPctX = 0.0;
            let minPctY = 1.0, maxPctY = 0.0;
            for (const p of corners) {
                if (p.x < minPctX) minPctX = p.x;
                if (p.x > maxPctX) maxPctX = p.x;
                if (p.y < minPctY) minPctY = p.y;
                if (p.y > maxPctY) maxPctY = p.y;
            }

            const minCol = Math.max(0, Math.floor(minPctX * cols));
            const maxCol = Math.min(cols - 1, Math.ceil(maxPctX * cols));
            const minRow = Math.max(0, Math.floor(minPctY * rows));
            const maxRow = Math.min(rows - 1, Math.ceil(maxPctY * rows));

            for (let r_dest = minRow; r_dest <= maxRow; r_dest++) {
                for (let c_dest = minCol; c_dest <= maxCol; c_dest++) {
                    const pctX_dest = (c_dest + 0.5) / cols;
                    const pctY_dest = (r_dest + 0.5) / rows;
                    const phys_dest = pctToPhysical(pctX_dest, pctY_dest);

                    const dx = phys_dest.x - xp;
                    const dy = phys_dest.y - yp;
                    const d2 = dx * dx + dy * dy;

                    const val = Math.exp(-d2 / (2.0 * pestVariance));
                    if (val >= 0.05) {
                        grid[r_dest][c_dest] += pest * val;
                    }
                }
            }
        }
    }
    return grid;
}

// Perform a specified number of simulation steps and redraw at the end
function runSimulationTicks(n) {
    if (!mapConfig || !mapConfig.width_pixels) return;

    fetch('/api/tick_simulation', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            steps: n,
            variance: pestVariance,
            spread_coef: spreadCoef
        })
    })
    .then(res => res.json())
    .then(data => {
        if (data.status === 'ok') {
            pestGrid = data.pest_grid;
            drawHeatmap();
        } else {
            addLog('Server', 'Simulation error: ' + data.message, 'error');
        }
    })
    .catch(err => {
        addLog('Server', 'Network error running simulation: ' + err, 'error');
    });
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
        pestGrid = Array.from({ length: rows }, () => new Float32Array(cols));
    }

    // Get current Gaussian spread
    const gaussianPest = calculateGaussianSpread(cols, rows);

    // 2. Render pest spread (green gradients based on combined active & background spread)
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            const val = Math.max(pestGrid[r][c], gaussianPest[r][c]);
            if (val > 0.01) {
                const opacity = Math.min(1.0, val / 100.0);
                ctx.fillStyle = `rgba(0, 255, 102, ${opacity.toFixed(3)})`;
                ctx.fillRect(c, r, 1, 1);
            }
        }
    }

    // 3. Draw mapped plants as 1 orange pixel on the canvas
    ctx.fillStyle = '#ff7300';
    for (const plant of scannedPlants) {
        const pct = physicalToPct(plant.x, plant.y);
        const col = Math.round(pct.x * cols);
        const row = Math.round(pct.y * rows);

        if (col >= 0 && col < cols && row >= 0 && row < rows) {
            ctx.fillRect(col, row, 1, 1);
        }
    }
}
