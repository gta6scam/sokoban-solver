// Sokoban solver: push-space A* with a Hungarian (optimal-assignment)
// heuristic over precomputed push distances, dead-square pruning, and
// room decomposition (articulation-point analysis splits the level into
// independent rooms joined by a hallway; each room is solved push-optimally
// on its own and the move strings are concatenated — walking is free, so
// per-room push optima sum to the level push optimum).
//
// Plain JavaScript in a .ts file (Node 22 type stripping). No clock reads,
// no randomness: search budgets are fixed expansion counts, so two runs
// always produce byte-identical output.

import { readdirSync, readFileSync, writeFileSync, mkdirSync } from "node:fs";
import { join } from "node:path";

const DIRS = [
  { dx: 0, dy: -1, ch: "u" },
  { dx: 0, dy: 1, ch: "d" },
  { dx: -1, dy: 0, ch: "l" },
  { dx: 1, dy: 0, ch: "r" },
];
const INF = 1e9;
const ROOM_EXPANSION_CAP = 600000;
const WHOLE_EXPANSION_CAP = 300000;

// ---------------------------------------------------------------- parsing

function parseLevel(text) {
  const rows = text.replace(/\r/g, "").split("\n");
  while (rows.length && rows[rows.length - 1] === "") rows.pop();
  const H = rows.length;
  let W = 0;
  for (const r of rows) W = Math.max(W, r.length);
  const N = W * H;
  const floor = new Uint8Array(N);
  const goals = [];
  const goalSet = new Set();
  let boxes = [];
  let player = -1;
  for (let y = 0; y < H; y++) {
    const row = rows[y];
    for (let x = 0; x < W; x++) {
      const ch = x < row.length ? row[x] : "#";
      if (ch === "#") continue;
      const c = y * W + x;
      floor[c] = 1;
      if (ch === "." || ch === "*" || ch === "+") { goals.push(c); goalSet.add(c); }
      if (ch === "$" || ch === "*") boxes.push(c);
      if (ch === "@" || ch === "+") player = c;
    }
  }
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  // neighbour table: nbr[cell*4+dir] = cell index or -1
  const nbr = new Int32Array(N * 4).fill(-1);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    const c = y * W + x;
    if (!floor[c]) continue;
    for (let d = 0; d < 4; d++) {
      const nx = x + DIRS[d].dx, ny = y + DIRS[d].dy;
      if (nx >= 0 && nx < W && ny >= 0 && ny < H && floor[ny * W + nx]) nbr[c * 4 + d] = ny * W + nx;
    }
  }
  return { W, H, N, floor, goals, goalSet, boxes, player, nbr };
}

// ------------------------------------------------------- graph utilities

// Articulation points of the floor graph (iterative Tarjan).
function articulationPoints(L) {
  const { N, floor, nbr } = L;
  const disc = new Int32Array(N).fill(-1);
  const low = new Int32Array(N);
  const parent = new Int32Array(N).fill(-1);
  const nchild = new Int32Array(N);
  const ap = new Uint8Array(N);
  let timer = 0;
  const stack = [];
  for (let root = 0; root < N; root++) {
    if (!floor[root] || disc[root] !== -1) continue;
    disc[root] = low[root] = timer++;
    stack.push({ u: root, di: 0 });
    while (stack.length) {
      const top = stack[stack.length - 1];
      const u = top.u;
      let advanced = false;
      while (top.di < 4) {
        const v = nbr[u * 4 + top.di++];
        if (v < 0) continue;
        if (disc[v] === -1) {
          parent[v] = u; nchild[u]++;
          disc[v] = low[v] = timer++;
          stack.push({ u: v, di: 0 });
          advanced = true;
          break;
        } else if (v !== parent[u]) {
          if (disc[v] < low[u]) low[u] = disc[v];
        }
      }
      if (!advanced && top.di >= 4) {
        stack.pop();
        const p = parent[u];
        if (p >= 0) {
          if (low[u] < low[p]) low[p] = low[u];
          if (parent[p] !== -1 && low[u] >= disc[p]) ap[p] = 1;
          if (parent[p] === -1 && nchild[p] > 1) ap[p] = 1;
        }
      }
    }
  }
  return ap;
}

// Connected components of floor cells not in `removed`.
function components(L, removed) {
  const { N, floor, nbr } = L;
  const seen = new Uint8Array(N);
  const comps = [];
  for (let s = 0; s < N; s++) {
    if (!floor[s] || removed[s] || seen[s]) continue;
    const comp = [];
    const stack = [s];
    seen[s] = 1;
    while (stack.length) {
      const u = stack.pop();
      comp.push(u);
      for (let d = 0; d < 4; d++) {
        const v = nbr[u * 4 + d];
        if (v >= 0 && !removed[v] && !seen[v]) { seen[v] = 1; stack.push(v); }
      }
    }
    comps.push(comp);
  }
  return comps;
}

// Split the level into rooms. Returns an array of {cells:Set, boxes, goals}
// or null when the structure does not decompose cleanly.
function decompose(L) {
  const ap = articulationPoints(L);
  const boxSet = new Set(L.boxes);
  const comps = components(L, ap);
  const coreOf = new Int32Array(L.N).fill(-1);
  const cores = [];
  for (const comp of comps) {
    if (!comp.some((c) => boxSet.has(c))) continue;
    const idx = cores.length;
    cores.push(comp);
    for (const c of comp) coreOf[c] = idx;
  }
  if (cores.length === 0) return null;
  // Hallway: the longer straight run of floor cells through the player.
  function run(dx, dy) {
    const cells = [L.player];
    const px = L.player % L.W, py = (L.player / L.W) | 0;
    for (const s of [1, -1]) {
      let x = px, y = py;
      for (;;) {
        x += dx * s; y += dy * s;
        if (x < 0 || x >= L.W || y < 0 || y >= L.H || !L.floor[y * L.W + x]) break;
        cells.push(y * L.W + x);
      }
    }
    return cells;
  }
  const hr = run(1, 0), vr = run(0, 1);
  const hallway = new Set(hr.length >= vr.length ? hr : vr);
  // Label every non-core, non-hallway floor cell with the room it belongs
  // to: multi-source BFS outward from the cores through corridor cells.
  const label = new Int32Array(L.N).fill(-1);
  const queue = [];
  for (let c = 0; c < L.N; c++) {
    if (!L.floor[c] || coreOf[c] !== -1 || hallway.has(c)) continue;
    for (let d = 0; d < 4; d++) {
      const v = L.nbr[c * 4 + d];
      if (v >= 0 && coreOf[v] !== -1) { label[c] = coreOf[v]; queue.push(c); break; }
    }
  }
  for (let qi = 0; qi < queue.length; qi++) {
    const u = queue[qi];
    for (let d = 0; d < 4; d++) {
      const v = L.nbr[u * 4 + d];
      if (v >= 0 && L.floor[v] && coreOf[v] === -1 && !hallway.has(v) && label[v] === -1) {
        label[v] = label[u]; queue.push(v);
      }
    }
  }
  const rooms = cores.map((comp, i) => {
    const cells = new Set(comp);
    for (let c = 0; c < L.N; c++) if (label[c] === i) cells.add(c);
    const boxes = L.boxes.filter((b) => cells.has(b));
    const goals = L.goals.filter((g) => cells.has(g));
    return { cells, boxes, goals };
  });
  // Merge pass: an articulation point inside one generated room can split
  // it into two cores that are only balanced together (observed on drawn
  // sets: a 3-room level whose lowest room came apart into a 4/4 core and
  // a 2-box/1-goal fragment). Repeatedly merge an empty or unbalanced room
  // with the neighbouring room whose merger most reduces total imbalance.
  const imbalance = (r) => Math.abs(r.boxes.length - r.goals.length) + (r.boxes.length === 0 ? 100 : 0);
  const totalImbalance = () => rooms.reduce((a, r) => a + imbalance(r), 0);
  const adjacent = (a, b) => {
    for (const c of a.cells) {
      for (let d = 0; d < 4; d++) {
        const v = L.nbr[c * 4 + d];
        if (v >= 0 && b.cells.has(v)) return true;
        if (v >= 0 && L.floor[v]) {
          for (let e = 0; e < 4; e++) {
            const u = L.nbr[v * 4 + e];
            if (u >= 0 && b.cells.has(u)) return true;
          }
        }
      }
    }
    return false;
  };
  let mergeGuard = 0;
  while (totalImbalance() > 0 && rooms.length > 1 && mergeGuard++ < 64) {
    let bi = -1, bj = -1, bestGain = 0;
    for (let i = 0; i < rooms.length; i++) {
      if (imbalance(rooms[i]) === 0) continue;
      for (let j = 0; j < rooms.length; j++) {
        if (i === j || !adjacent(rooms[i], rooms[j])) continue;
        const mergedImbalance = Math.abs((rooms[i].boxes.length + rooms[j].boxes.length) - (rooms[i].goals.length + rooms[j].goals.length)) + (rooms[i].boxes.length + rooms[j].boxes.length === 0 ? 100 : 0);
        const gain = imbalance(rooms[i]) + imbalance(rooms[j]) - mergedImbalance;
        if (gain > bestGain) { bestGain = gain; bi = i; bj = j; }
      }
    }
    if (bi < 0) break;
    const merged = {
      cells: new Set([...rooms[bi].cells, ...rooms[bj].cells]),
      boxes: [...rooms[bi].boxes, ...rooms[bj].boxes],
      goals: [...rooms[bi].goals, ...rooms[bj].goals],
    };
    rooms.splice(Math.max(bi, bj), 1);
    rooms.splice(Math.min(bi, bj), 1);
    rooms.push(merged);
  }
  // Validity: disjoint, balanced, and covering every box and goal.
  const covered = new Set();
  for (const r of rooms) {
    if (r.boxes.length === 0 || r.boxes.length !== r.goals.length) return null;
    for (const c of r.cells) { if (covered.has(c)) return null; covered.add(c); }
  }
  for (const b of L.boxes) if (!covered.has(b)) return null;
  for (const g of L.goals) if (!covered.has(g)) return null;
  rooms.sort((a, b) => a.boxes[0] - b.boxes[0]);
  return rooms;
}

// Fallback decomposition: remove the player's hallway row; each remaining
// floor component is a room. Cruder than the articulation split (it cannot
// subdivide a pinched room), but it never splits one either, so it stays
// balanced exactly when the generator's rooms are.
function decomposeByRow(L) {
  const prow = (L.player / L.W) | 0;
  const removed = new Uint8Array(L.N);
  for (let x = 0; x < L.W; x++) removed[prow * L.W + x] = 1;
  const comps = components(L, removed);
  const rooms = [];
  const covered = new Set();
  for (const comp of comps) {
    const cells = new Set(comp);
    const boxes = L.boxes.filter((b) => cells.has(b));
    if (boxes.length === 0) continue;
    const goals = L.goals.filter((g) => cells.has(g));
    if (boxes.length !== goals.length) return null;
    for (const c of cells) covered.add(c);
    rooms.push({ cells, boxes, goals });
  }
  if (rooms.length === 0) return null;
  for (const b of L.boxes) if (!covered.has(b)) return null;
  for (const g of L.goals) if (!covered.has(g)) return null;
  rooms.sort((a, b) => a.boxes[0] - b.boxes[0]);
  return rooms;
}

// ------------------------------------------------------------ heuristics

// Push distance from every cell to each goal: reverse BFS in pull space
// over box positions confined to `region` (stance cells may be any floor).
function pushDistances(L, goals, region) {
  const dists = [];
  for (const g of goals) {
    const dist = new Int32Array(L.N).fill(INF);
    if (region.has(g)) {
      dist[g] = 0;
      const queue = [g];
      for (let qi = 0; qi < queue.length; qi++) {
        const b = queue[qi];
        for (let d = 0; d < 4; d++) {
          const opp = d ^ 1;
          const prev = L.nbr[b * 4 + opp];       // box position before the push
          if (prev < 0 || !region.has(prev) || dist[prev] !== INF) continue;
          const stance = L.nbr[prev * 4 + opp];  // where the player stood
          if (stance < 0) continue;
          dist[prev] = dist[b] + 1;
          queue.push(prev);
        }
      }
    }
    dists.push(dist);
  }
  return dists;
}

// Hungarian algorithm (min-cost square assignment), O(n^3), deterministic.
function hungarian(cost, n) {
  const u = new Float64Array(n + 1);
  const v = new Float64Array(n + 1);
  const p = new Int32Array(n + 1).fill(-1);
  const way = new Int32Array(n + 1).fill(-1);
  for (let i = 1; i <= n; i++) {
    p[0] = i - 1;
    let j0 = 0;
    const minv = new Float64Array(n + 1).fill(Infinity);
    const used = new Uint8Array(n + 1);
    do {
      used[j0] = 1;
      const i0 = p[j0];
      let delta = Infinity, j1 = -1;
      for (let j = 1; j <= n; j++) {
        if (used[j]) continue;
        const cur = cost[i0 * n + (j - 1)] - u[i0 + 1] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }
      for (let j = 0; j <= n; j++) {
        if (used[j]) { u[p[j] + 1] += delta; v[j] -= delta; }
        else minv[j] -= delta;
      }
      j0 = j1;
    } while (p[j0] !== -1);
    do {
      const j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 !== 0);
  }
  return -v[0];
}

function makeHeuristic(L, goals, dists) {
  const m = goals.length;
  return function (boxes) {
    const n = boxes.length;
    const size = Math.max(n, m);
    if (size === 0) return 0;
    const cost = new Float64Array(size * size);
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < m; j++) {
        const d = dists[j][boxes[i]];
        cost[i * size + j] = d >= INF ? 1e6 : d;
      }
    }
    const total = hungarian(cost, size);
    return total >= 1e6 ? INF : total;
  };
}

// ------------------------------------------------------------------- A*

function bfsWalk(L, start, blocked) {
  // BFS over floor avoiding `blocked` cells. Returns {prev, dirFrom} maps.
  const prev = new Int32Array(L.N).fill(-2);
  const dirFrom = new Int32Array(L.N).fill(-1);
  prev[start] = -1;
  const queue = [start];
  for (let qi = 0; qi < queue.length; qi++) {
    const u = queue[qi];
    for (let d = 0; d < 4; d++) {
      const v = L.nbr[u * 4 + d];
      if (v >= 0 && prev[v] === -2 && !blocked.has(v)) {
        prev[v] = u; dirFrom[v] = d; queue.push(v);
      }
    }
  }
  return { prev, dirFrom };
}

function walkPath(bfs, start, target) {
  if (bfs.prev[target] === -2) return null;
  const chars = [];
  let cur = target;
  while (cur !== start) {
    chars.push(DIRS[bfs.dirFrom[cur]].ch);
    cur = bfs.prev[cur];
  }
  chars.reverse();
  return chars.join("");
}

// Push-space A*. dynBoxes move only inside `region`; staticBoxes are frozen
// obstacles. Returns a move string or null.
function solvePushSpace(L, dynBoxes0, goals, region, staticBoxes, player0, cap) {
  const goalSet = new Set(goals);
  const dists = pushDistances(L, goals, region);
  const heuristic = makeHeuristic(L, goals, dists);
  const dead = new Uint8Array(L.N);
  for (const c of region) {
    if (goalSet.has(c)) continue;
    let reachable = false;
    for (let j = 0; j < goals.length; j++) if (dists[j][c] < INF) { reachable = true; break; }
    if (!reachable) dead[c] = 1;
  }
  for (const b of dynBoxes0) if (dead[b]) return null;
  const h0 = heuristic(dynBoxes0);
  if (h0 >= INF) return null;

  // Node store.
  const nBoxes = [];
  const nPlayer = [];
  const nG = [];
  const nParent = [];
  const nSeg = [];
  const best = new Map();
  // Binary heap of node indices keyed by (f, h, index).
  const heap = [];
  const fOf = [];
  const hOf = [];
  function heapPush(idx) {
    heap.push(idx);
    let i = heap.length - 1;
    while (i > 0) {
      const par = (i - 1) >> 1;
      if (less(heap[i], heap[par])) { const t = heap[i]; heap[i] = heap[par]; heap[par] = t; i = par; }
      else break;
    }
  }
  function less(a, b) {
    if (fOf[a] !== fOf[b]) return fOf[a] < fOf[b];
    if (hOf[a] !== hOf[b]) return hOf[a] < hOf[b];
    return a < b;
  }
  function heapPop() {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < heap.length && less(heap[l], heap[m])) m = l;
        if (r < heap.length && less(heap[r], heap[m])) m = r;
        if (m === i) break;
        const t = heap[i]; heap[i] = heap[m]; heap[m] = t; i = m;
      }
    }
    return top;
  }
  function addNode(boxes, player, g, parent, seg, h) {
    const idx = nBoxes.length;
    nBoxes.push(boxes); nPlayer.push(player); nG.push(g);
    nParent.push(parent); nSeg.push(seg);
    fOf.push(g + h); hOf.push(h);
    heapPush(idx);
    return idx;
  }

  const startBoxes = [...dynBoxes0].sort((a, b) => a - b);
  {
    const blocked = new Set([...startBoxes, ...staticBoxes]);
    const bfs = bfsWalk(L, player0, blocked);
    let canon = player0;
    for (let c = 0; c < L.N; c++) if (bfs.prev[c] !== -2) { canon = c; break; }
    best.set(startBoxes.join(",") + "|" + canon, 0);
  }
  addNode(startBoxes, player0, 0, -1, "", h0);
  let expansions = 0;

  while (heap.length) {
    const idx = heapPop();
    const boxes = nBoxes[idx];
    const g = nG[idx];
    let solved = true;
    for (const b of boxes) if (!goalSet.has(b)) { solved = false; break; }
    if (solved) {
      const parts = [];
      let cur = idx;
      while (cur !== -1) { parts.push(nSeg[cur]); cur = nParent[cur]; }
      parts.reverse();
      return parts.join("");
    }
    if (++expansions > cap) return null;
    const boxSetNow = new Set(boxes);
    const blocked = new Set([...boxes, ...staticBoxes]);
    const bfs = bfsWalk(L, nPlayer[idx], blocked);
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      for (let d = 0; d < 4; d++) {
        const dest = L.nbr[b * 4 + d];
        if (dest < 0 || !region.has(dest) || boxSetNow.has(dest) || staticBoxes.has(dest)) continue;
        if (dead[dest]) continue;
        const stance = L.nbr[b * 4 + (d ^ 1)];
        if (stance < 0) continue;
        const walk = walkPath(bfs, nPlayer[idx], stance);
        if (walk === null) continue;
        const nb = boxes.slice();
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        const ng = g + 1;
        // Canonical player position: smallest cell reachable from the
        // post-push player cell (the pushed box's old cell).
        const nBlocked = new Set([...nb, ...staticBoxes]);
        const nbfs = bfsWalk(L, b, nBlocked);
        let canon = b;
        for (let c = 0; c < L.N; c++) if (nbfs.prev[c] !== -2) { canon = c; break; }
        const key = nb.join(",") + "|" + canon;
        const prevBest = best.get(key);
        if (prevBest !== undefined && prevBest <= ng) continue;
        const h = heuristic(nb);
        if (h >= INF) continue;
        best.set(key, ng);
        addNode(nb, b, ng, idx, walk + DIRS[d].ch, h);
      }
    }
  }
  return null;
}

// ------------------------------------------------------------- simulation

function simulate(L, boxes, player, moves) {
  const boxSet = new Set(boxes);
  let p = player;
  for (const ch of moves) {
    const d = DIRS.findIndex((dd) => dd.ch === ch);
    const nxt = L.nbr[p * 4 + d];
    if (nxt < 0) throw new Error("illegal walk in own solution");
    if (boxSet.has(nxt)) {
      const beyond = L.nbr[nxt * 4 + d];
      if (beyond < 0 || boxSet.has(beyond)) throw new Error("illegal push in own solution");
      boxSet.delete(nxt); boxSet.add(beyond);
    }
    p = nxt;
  }
  return { boxes: [...boxSet].sort((a, b) => a - b), player: p };
}

// ---------------------------------------------------------------- solver

function solveByRooms(L, rooms) {
  let boxes = L.boxes;
  let player = L.player;
  let out = "";
  for (const room of rooms) {
    const roomBoxSet = new Set(room.boxes);
    const dyn = boxes.filter((b) => roomBoxSet.has(b));
    const statics = new Set(boxes.filter((b) => !roomBoxSet.has(b)));
    if (dyn.length !== room.boxes.length) return null;
    const sol = solvePushSpace(L, dyn, room.goals, room.cells, statics, player, ROOM_EXPANSION_CAP);
    if (sol === null) return null;
    const after = simulate(L, boxes, player, sol);
    boxes = after.boxes; player = after.player;
    out += sol;
  }
  return boxes.every((b) => L.goalSet.has(b)) ? out : null;
}

function solveLevel(L) {
  if (L.boxes.every((b) => L.goalSet.has(b))) return "";
  for (const rooms of [decompose(L), decomposeByRow(L)]) {
    if (!rooms) continue;
    const out = solveByRooms(L, rooms);
    if (out !== null) return out;
  }
  // Fallback: whole level in one push-space search.
  const allCells = new Set();
  for (let c = 0; c < L.N; c++) if (L.floor[c]) allCells.add(c);
  const sol = solvePushSpace(L, L.boxes, L.goals, allCells, new Set(), L.player, WHOLE_EXPANSION_CAP);
  return sol === null ? "" : sol;
}

function main() {
  const inDir = process.env.MAKEFUN_INPUT_DIR ?? "/input";
  const outDir = process.env.MAKEFUN_OUTPUT_DIR ?? "/output";
  let ids = [];
  try {
    ids = readdirSync(inDir, { withFileTypes: true })
      .filter((e) => e.isDirectory())
      .map((e) => e.name)
      .sort();
  } catch {
    ids = [];
  }
  for (const id of ids) {
    let solution = "";
    try {
      const text = readFileSync(join(inDir, id, "level.txt"), "utf8");
      solution = solveLevel(parseLevel(text));
    } catch {
      solution = "";
    }
    const dir = join(outDir, id);
    mkdirSync(dir, { recursive: true });
    writeFileSync(join(dir, "solution.txt"), solution + "\n");
  }
}

main();
