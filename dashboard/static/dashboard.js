const CELL_COLORS = {
  EMPTY: "#f4f4f6", WALL: "#2b2f38", SHELF: "#8a5a2b",
  PICKUP: "#2e9e5b", DROPOFF: "#2b6fce", CHARGING_STATION: "#e0b400",
  INTERSECTION: "#efe3ff",
};
const STATUS_COLORS = {
  IDLE: "#9aa0a6", MOVING: "#2e9e5b", WAITING: "#e0a300",
  NEGOTIATING: "#c77dff", REROUTING: "#ff8c42", BLOCKED: "#d1483c",
  CHARGING: "#e0b400", COMPLETED: "#5b6472", FAILED: "#000000",
};
const ROBOT_PALETTE = ["#e63946", "#1d3557", "#2a9d8f", "#f4a261", "#6a4c93",
                        "#457b9d", "#ff006e", "#43aa8b"];

let warehouse = null;
let robotColor = {};
const CELL = 28;
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

function cellTypeAt(x, y) {
  return (warehouse.cells[`${x},${y}`]) || "EMPTY";
}

function drawWarehouse() {
  for (let y = 0; y < warehouse.height; y++) {
    for (let x = 0; x < warehouse.width; x++) {
      const t = cellTypeAt(x, y);
      ctx.fillStyle = CELL_COLORS[t] || CELL_COLORS.EMPTY;
      ctx.fillRect(x * CELL, y * CELL, CELL, CELL);
      ctx.strokeStyle = 'rgba(0,0,0,0.06)';
      ctx.strokeRect(x * CELL, y * CELL, CELL, CELL);
    }
  }
  ctx.fillStyle = 'rgba(220,60,50,0.45)';
  (warehouse.blocked_aisles || []).forEach(([x, y]) => {
    ctx.fillRect(x * CELL, y * CELL, CELL, CELL);
  });
}

function drawRobots(robots) {
  Object.entries(robots).forEach(([rid, r]) => {
    const [x, y] = r.position;
    const cx = x * CELL + CELL / 2;
    const cy = y * CELL + CELL / 2;

    ctx.beginPath();
    ctx.arc(cx, cy, CELL * 0.38, 0, Math.PI * 2);
    ctx.fillStyle = robotColor[rid];
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = STATUS_COLORS[r.status] || '#fff';
    ctx.stroke();

    ctx.fillStyle = '#fff';
    ctx.font = 'bold 11px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(rid, cx, cy);
  });
}

function buildLegend() {
  const legend = document.getElementById('legend');
  const items = [
    ['Wall', CELL_COLORS.WALL], ['Shelf', CELL_COLORS.SHELF],
    ['Pickup', CELL_COLORS.PICKUP], ['Dropoff', CELL_COLORS.DROPOFF],
    ['Charger', CELL_COLORS.CHARGING_STATION], ['Intersection', CELL_COLORS.INTERSECTION],
    ['Blocked aisle', 'rgba(220,60,50,0.7)'],
  ];
  items.forEach(([label, color]) => {
    const span = document.createElement('span');
    span.innerHTML = `<span class="swatch" style="background:${color}"></span>${label}`;
    legend.appendChild(span);
  });
}

function renderStats(status) {
  const row = document.getElementById('statRow');
  const cards = [
    ['Sim time', `${status.sim_time}s`],
    ['Tick', status.tick_count],
    ['Active robots', status.active_robots],
    ['Completed', status.completed_robots],
    ['Avg battery', `${status.average_battery}%`],
    ['Total distance', status.total_distance],
    ['Collisions', status.total_collisions],
    ['State', status.paused ? 'PAUSED' : 'RUNNING'],
  ];
  row.innerHTML = cards.map(([label, value]) =>
    `<div class="stat-card"><div class="label">${label}</div><div class="value">${value}</div></div>`
  ).join('');
}

function renderRobotTable(robots) {
  const tbody = document.getElementById('robotTable');
  tbody.innerHTML = '';
  Object.entries(robots).forEach(([rid, r]) => {
    const tr = document.createElement('tr');
    const statusColor = STATUS_COLORS[r.status] || '#888';
    tr.innerHTML = `
      <td>${rid}</td>
      <td>(${r.position[0]}, ${r.position[1]})</td>
      <td><span class="badge" style="background:${statusColor}">${r.status}</span></td>
      <td>${r.battery}%</td>
      <td>${r.distance_travelled}</td>
      <td>${r.collision_count}</td>`;
    tbody.appendChild(tr);
  });
}

async function poll() {
  try {
    const [statusRes, robotsRes] = await Promise.all([
      fetch('/api/status'), fetch('/api/robots'),
    ]);
    const status = await statusRes.json();
    const robots = await robotsRes.json();

    if (!warehouse) {
      const whRes = await fetch('/api/warehouse');
      warehouse = await whRes.json();
      canvas.width = warehouse.width * CELL;
      canvas.height = warehouse.height * CELL;
      Object.keys(robots).forEach((rid, i) => robotColor[rid] = ROBOT_PALETTE[i % ROBOT_PALETTE.length]);
      buildLegend();
    } else {
      const whRes = await fetch('/api/warehouse');
      const fresh = await whRes.json();
      warehouse.blocked_aisles = fresh.blocked_aisles;
    }

    drawWarehouse();
    drawRobots(robots);
    renderStats(status);
    renderRobotTable(robots);
    document.getElementById('frameInfo').textContent =
      `Polling every ${POLL_INTERVAL_MS}ms - not controlling movement, just reading state.`;
  } catch (err) {
    document.getElementById('frameInfo').textContent = 'Connection lost - simulation may still be running independently.';
  }
}

document.getElementById('pauseBtn').addEventListener('click', () => fetch('/api/simulation/pause', { method: 'POST' }));
document.getElementById('resumeBtn').addEventListener('click', () => fetch('/api/simulation/resume', { method: 'POST' }));

poll();
setInterval(poll, POLL_INTERVAL_MS);
