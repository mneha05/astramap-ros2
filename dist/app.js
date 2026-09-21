(() => {
  "use strict";

  const canvas = document.getElementById("gl-canvas");
  const gl = canvas.getContext("webgl", { antialias: true, alpha: false });
  if (!gl) {
    canvas.outerHTML = '<div class="webgl-error">WebGL is required to run the AstraMap simulation.</div>';
    return;
  }

  const WORLD = { minX: -10, maxX: 10, minY: -7, maxY: 7 };
  const GRID = { width: 160, height: 112, resolution: 0.125 };
  const state = {
    running: true,
    speed: 1,
    lidar: true,
    radar: true,
    trail: true,
    time: 0,
    last: performance.now(),
    accumulator: 0,
    pose: { x: 5.1, y: 0, yaw: Math.PI / 2 },
    scan: [],
    trailPoints: [],
    radarTargets: [],
    occupied: new Float32Array(GRID.width * GRID.height),
    observed: new Uint8Array(GRID.width * GRID.height),
    userObstacles: []
  };

  const fixedBoxes = [
    [-9, -6, 9, 6], [-2.5, -1.2, -0.7, 1.8],
    [2, -4.2, 3.3, -1], [3.7, 1.5, 6.2, 3.6]
  ];

  const vertexShader = `
    attribute vec2 a_position;
    attribute vec3 a_color;
    varying vec3 v_color;
    uniform float u_pointSize;
    void main() {
      gl_Position = vec4(a_position, 0.0, 1.0);
      gl_PointSize = u_pointSize;
      v_color = a_color;
    }`;
  const fragmentShader = `
    precision mediump float;
    varying vec3 v_color;
    uniform float u_roundPoints;
    void main() {
      if (u_roundPoints > 0.5) {
        vec2 d = gl_PointCoord - vec2(0.5);
        if (dot(d,d) > 0.25) discard;
      }
      gl_FragColor = vec4(v_color, 1.0);
    }`;

  function shader(type, source) {
    const item = gl.createShader(type);
    gl.shaderSource(item, source);
    gl.compileShader(item);
    if (!gl.getShaderParameter(item, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(item));
    return item;
  }

  const program = gl.createProgram();
  gl.attachShader(program, shader(gl.VERTEX_SHADER, vertexShader));
  gl.attachShader(program, shader(gl.FRAGMENT_SHADER, fragmentShader));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
  gl.useProgram(program);

  const buffer = gl.createBuffer();
  const positionLocation = gl.getAttribLocation(program, "a_position");
  const colorLocation = gl.getAttribLocation(program, "a_color");
  const pointSizeLocation = gl.getUniformLocation(program, "u_pointSize");
  const roundPointsLocation = gl.getUniformLocation(program, "u_roundPoints");
  gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
  gl.enableVertexAttribArray(positionLocation);
  gl.enableVertexAttribArray(colorLocation);
  gl.vertexAttribPointer(positionLocation, 2, gl.FLOAT, false, 20, 0);
  gl.vertexAttribPointer(colorLocation, 3, gl.FLOAT, false, 20, 8);

  function clip(x, y) {
    return [2 * (x - WORLD.minX) / (WORLD.maxX - WORLD.minX) - 1,
            2 * (y - WORLD.minY) / (WORLD.maxY - WORLD.minY) - 1];
  }

  function vertex(out, x, y, color) {
    const p = clip(x, y);
    out.push(p[0], p[1], color[0], color[1], color[2]);
  }

  function draw(vertices, mode, pointSize = 1, round = false) {
    if (!vertices.length) return;
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(vertices), gl.DYNAMIC_DRAW);
    gl.uniform1f(pointSizeLocation, pointSize * Math.min(devicePixelRatio, 2));
    gl.uniform1f(roundPointsLocation, round ? 1 : 0);
    gl.drawArrays(mode, 0, vertices.length / 5);
  }

  function segments() {
    const output = [];
    for (const [x0, y0, x1, y1] of fixedBoxes.concat(state.userObstacles)) {
      output.push([x0,y0,x1,y0],[x1,y0,x1,y1],[x1,y1,x0,y1],[x0,y1,x0,y0]);
    }
    return output;
  }

  function rayDistance(ox, oy, angle, segment) {
    const dx = Math.cos(angle), dy = Math.sin(angle);
    const ex = segment[2] - segment[0], ey = segment[3] - segment[1];
    const qx = segment[0] - ox, qy = segment[1] - oy;
    const cross = dx * ey - dy * ex;
    if (Math.abs(cross) < 1e-8) return Infinity;
    const distance = (qx * ey - qy * ex) / cross;
    const t = (qx * dy - qy * dx) / cross;
    return distance >= 0 && t >= 0 && t <= 1 ? distance : Infinity;
  }

  function gridPoint(x, y) {
    const gx = Math.floor((x - WORLD.minX) / GRID.resolution);
    const gy = Math.floor((y - WORLD.minY) / GRID.resolution);
    return gx >= 0 && gy >= 0 && gx < GRID.width && gy < GRID.height ? [gx, gy] : null;
  }

  function addOdds(x, y, amount) {
    if (x < 0 || y < 0 || x >= GRID.width || y >= GRID.height) return;
    const i = y * GRID.width + x;
    state.occupied[i] = Math.max(-4, Math.min(4, state.occupied[i] + amount));
    state.observed[i] = 1;
  }

  function integrateRay(ox, oy, ex, ey) {
    const a = gridPoint(ox, oy), b = gridPoint(ex, ey);
    if (!a || !b) return;
    let [x0, y0] = a;
    const [x1, y1] = b;
    const dx = Math.abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    const dy = -Math.abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    let error = dx + dy;
    while (x0 !== x1 || y0 !== y1) {
      addOdds(x0, y0, -0.035);
      const e2 = 2 * error;
      if (e2 >= dy) { error += dy; x0 += sx; }
      if (e2 <= dx) { error += dx; y0 += sy; }
    }
    addOdds(x1, y1, 0.18);
  }

  function simulationStep(dt) {
    state.time += dt * state.speed;
    const phase = 0.12 * state.time + 0.18;
    const x = 5.1 * Math.cos(phase);
    const y = 3.3 * Math.sin(phase);
    const yaw = Math.atan2(0.396 * Math.cos(phase), -0.612 * Math.sin(phase));
    state.pose = { x, y, yaw };
    state.trailPoints.push([x, y]);
    if (state.trailPoints.length > 360) state.trailPoints.shift();

    const walls = segments();
    const beamCount = 180;
    state.scan.length = 0;
    for (let i = 0; i < beamCount; i++) {
      const angle = yaw - Math.PI + i * (Math.PI * 2 / beamCount);
      let range = 14;
      for (const wall of walls) range = Math.min(range, rayDistance(x, y, angle, wall));
      const ex = x + Math.cos(angle) * range;
      const ey = y + Math.sin(angle) * range;
      state.scan.push([ex, ey, range < 13.9]);
      if (range < 13.9) integrateRay(x, y, ex, ey);
    }
    state.radarTargets = [
      [1.5 + 1.6 * Math.sin(.35 * state.time), .4 + 1.1 * Math.cos(.35 * state.time), .56 * Math.cos(.35 * state.time)],
      [-4 + .9 * Math.cos(.55 * state.time), -2.7 + .7 * Math.sin(.55 * state.time), -.495 * Math.sin(.55 * state.time)]
    ];
  }

  function resize() {
    const ratio = Math.min(devicePixelRatio, 2);
    const width = Math.floor(canvas.clientWidth * ratio);
    const height = Math.floor(canvas.clientHeight * ratio);
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width; canvas.height = height;
    }
    gl.viewport(0, 0, canvas.width, canvas.height);
  }

  function render() {
    resize();
    gl.clearColor(.025, .055, .095, 1);
    gl.clear(gl.COLOR_BUFFER_BIT);

    const gridLines = [];
    for (let x = -10; x <= 10; x++) { vertex(gridLines,x,-7,[.07,.15,.24]); vertex(gridLines,x,7,[.07,.15,.24]); }
    for (let y = -7; y <= 7; y++) { vertex(gridLines,-10,y,[.07,.15,.24]); vertex(gridLines,10,y,[.07,.15,.24]); }
    draw(gridLines, gl.LINES);

    const map = [];
    let occupiedCount = 0;
    for (let gy = 0; gy < GRID.height; gy += 2) {
      for (let gx = 0; gx < GRID.width; gx += 2) {
        const i = gy * GRID.width + gx;
        if (!state.observed[i] || state.occupied[i] < .3) continue;
        occupiedCount++;
        const confidence = Math.min(1, .5 + state.occupied[i] / 8);
        vertex(map, WORLD.minX + (gx + .5) * GRID.resolution,
          WORLD.minY + (gy + .5) * GRID.resolution,[.12,.55 + .35*confidence,.85]);
      }
    }
    draw(map, gl.POINTS, 2.6, true);

    const walls = [];
    for (const s of segments()) { vertex(walls,s[0],s[1],[.18,.56,.64]); vertex(walls,s[2],s[3],[.18,.56,.64]); }
    draw(walls, gl.LINES);

    if (state.trail) {
      const trail = [];
      for (let i = 1; i < state.trailPoints.length; i++) {
        const fade = i / state.trailPoints.length;
        vertex(trail,state.trailPoints[i-1][0],state.trailPoints[i-1][1],[.08,.25+.35*fade,.43]);
        vertex(trail,state.trailPoints[i][0],state.trailPoints[i][1],[.08,.25+.35*fade,.43]);
      }
      draw(trail, gl.LINES);
    }

    if (state.lidar) {
      const rays = [], hits = [];
      for (let i = 0; i < state.scan.length; i += 3) {
        const p = state.scan[i];
        vertex(rays,state.pose.x,state.pose.y,[.03,.25,.34]); vertex(rays,p[0],p[1],[.12,.48,.62]);
        if (p[2]) vertex(hits,p[0],p[1],[.18,.9,1]);
      }
      draw(rays, gl.LINES); draw(hits, gl.POINTS, 4.2, true);
    }

    if (state.radar) {
      const radarLines = [], radarPoints = [];
      for (const p of state.radarTargets) {
        vertex(radarLines,state.pose.x,state.pose.y,[.28,.08,.15]); vertex(radarLines,p[0],p[1],[.72,.12,.28]);
        vertex(radarPoints,p[0],p[1],[1,.28,.42]);
      }
      draw(radarLines, gl.LINES); draw(radarPoints, gl.POINTS, 13, true);
    }

    const robot = [];
    const shape = [[.42,0],[-.26,.24],[-.26,-.24]];
    for (const p of shape) {
      const c = Math.cos(state.pose.yaw), s = Math.sin(state.pose.yaw);
      vertex(robot,state.pose.x+c*p[0]-s*p[1],state.pose.y+s*p[0]+c*p[1],[.75,1,.94]);
    }
    draw(robot, gl.TRIANGLES);

    document.getElementById("map-cells").textContent = occupiedCount.toLocaleString();
    document.getElementById("coords").innerHTML = `X ${state.pose.x >= 0 ? "+" : ""}${state.pose.x.toFixed(2)}&nbsp;&nbsp;Y ${state.pose.y >= 0 ? "+" : ""}${state.pose.y.toFixed(2)}&nbsp;&nbsp;θ ${(state.pose.yaw*180/Math.PI).toFixed(1)}°`;
    document.getElementById("match-score").textContent = (0.91 + .045 * Math.abs(Math.sin(state.time*.21))).toFixed(2);
  }

  function loop(now) {
    const elapsed = Math.min(.05, (now - state.last) / 1000);
    state.last = now;
    if (state.running) {
      state.accumulator += elapsed;
      while (state.accumulator >= .05) { simulationStep(.05); state.accumulator -= .05; }
    }
    render();
    requestAnimationFrame(loop);
  }

  function updateRunButton() {
    const button = document.getElementById("toggle-run");
    button.innerHTML = state.running ? '<span class="pause-icon">Ⅱ</span> Pause' : '<span class="pause-icon">▶</span> Resume';
    document.getElementById("run-status").textContent = state.running ? "STREAMING" : "PAUSED";
  }

  function reset() {
    state.time = 0; state.accumulator = 0; state.scan = []; state.trailPoints = [];
    state.userObstacles = []; state.occupied.fill(0); state.observed.fill(0);
    for (let i = 0; i < 4; i++) simulationStep(.05);
  }

  document.getElementById("toggle-run").addEventListener("click", () => { state.running = !state.running; updateRunButton(); });
  document.getElementById("reset").addEventListener("click", reset);
  document.getElementById("speed").addEventListener("input", event => {
    state.speed = Number(event.target.value);
    document.getElementById("speed-value").textContent = `${state.speed.toFixed(2).replace(/0$/,"")}×`;
    document.getElementById("slam-rate").textContent = (20 * state.speed).toFixed(1);
  });
  document.getElementById("lidar-toggle").addEventListener("change", e => state.lidar = e.target.checked);
  document.getElementById("radar-toggle").addEventListener("change", e => state.radar = e.target.checked);
  document.getElementById("trail-toggle").addEventListener("change", e => state.trail = e.target.checked);
  document.addEventListener("keydown", event => {
    if (event.code === "Space") { event.preventDefault(); state.running = !state.running; updateRunButton(); }
    if (event.key.toLowerCase() === "r") reset();
  });
  canvas.addEventListener("click", event => {
    const rect = canvas.getBoundingClientRect();
    const x = WORLD.minX + (event.clientX - rect.left) / rect.width * (WORLD.maxX - WORLD.minX);
    const y = WORLD.maxY - (event.clientY - rect.top) / rect.height * (WORLD.maxY - WORLD.minY);
    state.userObstacles.push([x-.45,y-.35,x+.45,y+.35]);
    document.getElementById("hint").textContent = `OBSTACLE ${state.userObstacles.length} ADDED`;
    setTimeout(() => document.getElementById("hint").textContent = "+ CLICK TO PLACE OBSTACLE", 1400);
  });

  reset();
  requestAnimationFrame(loop);
})();
