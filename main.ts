// Sokoban solver: per-room push-optimal A* with deadlock pruning.
//
// Approach (my own implementation):
//   * A level is a hallway plus 1-4 rooms. A box pushed out of a room into the
//     hallway can never return, so every room is solved independently and the
//     push counts add up to the level optimum.
//   * Each room is solved in PUSH space: a state is the sorted box squares plus
//     the player's reachable region (named by its lowest square). Expanding a
//     state tries every push the player can reach. A* orders states by
//     pushes-so-far + an admissible heuristic (the minimum-cost assignment of
//     boxes to goals under shortest-path floor distances), so the first solved
//     state found is push-optimal.
//   * Pruning (both sound: they never remove a solvable state):
//       - dead squares: a box square from which no sequence of pulls (ignoring
//         other boxes) can reach a goal is never worth pushing onto;
//       - frozen 2x2 blocks: four wall/box squares with a box off its goal.
//   * Moves are rebuilt by stitching shortest walks (BFS) between pushes.
//   * Work budgets are counts of expanded states, never the wall clock, so two
//     runs write identical bytes. Stdlib only, no network, no clock reads.
//
// Design notes: push-space search (not move-space) because the score counts
// pushes, not moves; the assignment heuristic is what makes 4-box rooms
// tractable where a plain BFS would drown; the room split is what makes the
// whole level tractable (rooms multiply, they do not add, in state space).
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Work budgets (expanded states). Deterministic: no clock, no randomness.
const ROOM_BUDGET = 600_000;
const WHOLE_BUDGET = 1_200_000;

const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const LETTER = ['u', 'd', 'l', 'r'];
const PUSH_LETTER = ['U', 'D', 'L', 'R'];

// ---------------------------------------------------------------- parsing
function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals = [];
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++)
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// ------------------------------------------------------- dead-square table
// Squares from which a box can be pulled onto some goal, ignoring other
// boxes: reverse BFS from the goals. A box anywhere else is dead forever.
function liveSquares(level) {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue = [];
  for (const g of goals) {
    live[g] = 1;
    queue.push(g);
  }
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi];
    for (let k = 0; k < 4; k++) {
      const d = DY[k] * w + DX[k];
      const y = x - d; // box stands here before the pull
      if (y < 0 || y >= wall.length) continue;
      // the player must stand one step further back to pull
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

// ------------------------------------------- shortest floor distances (BFS)
// dist[g][c]: shortest walk from goal g to cell c over open floor, ignoring
// boxes. Used by the assignment heuristic.
function goalDistances(level) {
  const { w, wall, goals } = level;
  const n = wall.length;
  const dists = [];
  for (const g of goals) {
    const d = new Int32Array(n).fill(-1);
    d[g] = 0;
    const q = [g];
    for (let qi = 0; qi < q.length; qi++) {
      const p = q[qi];
      for (let k = 0; k < 4; k++) {
        const c = p + DY[k] * w + DX[k];
        if (!wall[c] && d[c] < 0) {
          d[c] = d[p] + 1;
          q.push(c);
        }
      }
    }
    dists.push(d);
  }
  return dists;
}

// Minimum-cost assignment of boxes to goals under the distance tables.
// Boxes <= 4, so enumerate permutations. Admissible: one push moves one box
// by one cell, changing this sum by at most 1, and it is 0 at the goal.
function assignmentHeuristic(boxes, dists, nGoals) {
  const n = boxes.length;
  if (n === 0) return 0;
  // Permutations of goal indices, taken n at a time (nGoals == n here).
  let best = Infinity;
  const used = new Uint8Array(nGoals);
  const perm = new Array(n);
  const rec = (i, sum) => {
    if (sum >= best) return;
    if (i === n) {
      best = sum;
      return;
    }
    for (let g = 0; g < nGoals; g++) {
      if (used[g]) continue;
      const d = dists[g][boxes[i]];
      if (d < 0) continue; // unreachable goal for this box
      used[g] = 1;
      perm[i] = g;
      rec(i + 1, sum + d);
      used[g] = 0;
    }
  };
  rec(0, 0);
  return best === Infinity ? 1e9 : best;
}

// ------------------------------------------------------------ reachability
// Flood-fill the player's reachable region from `from` with `occupied` in
// the way. Marks `seen` with `stamp`, returns the region's lowest square
// (the state's canonical region name) and leaves `seen` stamped.
function reachRegion(level, occupied, from, seen, stamp, stack) {
  const { w, wall } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    for (let k = 0; k < 4; k++) {
      const q = p + DY[k] * w + DX[k];
      if (!wall[q] && !occupied[q] && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

// ------------------------------------------------------- frozen 2x2 check
function frozenBlock(level, occupied, goalSet, t) {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (a < 0 || d >= wall.length) continue;
    if (!(wall[a] || occupied[a])) continue;
    if (!(wall[b] || occupied[b])) continue;
    if (!(wall[c] || occupied[c])) continue;
    if (!(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goalSet[a]) || (occupied[b] && !goalSet[b]) ||
        (occupied[c] && !goalSet[c]) || (occupied[d] && !goalSet[d])) return true;
  }
  return false;
}

// ------------------------------------------------------------------ walk
// Shortest walk from `from` to `to` around walls and boxes, lower-case
// letters, deterministic direction order U, D, L, R.
function walkTo(level, occupied, from, to) {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (let k = 0; k < 4; k++) {
      const q = p + DY[k] * w + DX[k];
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = LETTER[k];
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: push square unreachable');
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

// ------------------------------------------------------- push-space A*
// level: {w,h,wall,goals,boxes,player}. Returns {pushes, moves, endPlayer}
// with a push-optimal solution, or null when the budget runs out / unsolved.
function solvePushAStar(level, maxExpand) {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goalSet = new Uint8Array(size);
  for (const g of goals) goalSet[g] = 1;
  const live = liveSquares(level);
  const dists = goalDistances(level);
  const nGoals = goals.length;
  if (level.boxes.some((b) => !live[b])) return null;

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const solved = (boxes) => boxes.every((b) => goalSet[b]);
  const keyOf = (boxes, low) => {
    let k = low;
    for (const b of boxes) k = k * size + b;
    return k;
  };

  // State arrays: boxes, player cell (stands on the last pushed box's old
  // cell), parent index, push that created it (box from -> to).
  const sBoxes = [level.boxes.slice()];
  const sPlayer = [level.player];
  const sParent = [-1];
  const sFrom = [-1];
  const sTo = [-1];
  const sG = [0];
  const best = new Map(); // key -> state index with smallest g seen

  for (const b of level.boxes) occupied[b] = 1;
  const low0 = reachRegion(level, occupied, level.player, seen, ++stamp, stack);
  for (const b of level.boxes) occupied[b] = 0;
  best.set(keyOf(level.boxes, low0), 0);

  // Binary heap on (f, h, seq): smallest f first, then smallest h, then
  // earliest inserted. Deterministic.
  const heap = [];
  let seq = 0;
  const h0 = assignmentHeuristic(level.boxes, dists, nGoals);
  const heapLess = (a, b) =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e) => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i], heap[p])) break;
      const t = heap[i]; heap[i] = heap[p]; heap[p] = t;
      i = p;
    }
  };
  const heapPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && heapLess(heap[c + 1], heap[c])) c++;
        if (!heapLess(heap[c], heap[i])) break;
        const t = heap[i]; heap[i] = heap[c]; heap[c] = t;
        i = c;
      }
    }
    return top;
  };
  heapPush([h0, h0, seq++, 0]);

  let expanded = 0;
  let found = solved(level.boxes) ? 0 : -1;
  while (heap.length > 0 && found < 0) {
    const [, , , s] = heapPop();
    const boxes = sBoxes[s];
    const g = sG[s];
    for (const b of boxes) occupied[b] = 1;
    const low = reachRegion(level, occupied, sPlayer[s], seen, ++stamp, stack);
    if (best.get(keyOf(boxes, low)) !== s) {
      // A better path to this state was found since this entry was queued.
      for (const b of boxes) occupied[b] = 0;
      continue;
    }
    const here = stamp; // seen[] now stamps this state's reachable region
    expanded++;
    if (expanded > maxExpand) return null;
    for (let bi = 0; bi < boxes.length && found < 0; bi++) {
      const b = boxes[bi];
      for (let k = 0; k < 4; k++) {
        const d = DY[k] * w + DX[k];
        const t = b + d;
        // player must stand on the far side of the box to push it
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goalSet, t);
        let key = -1;
        let next = null;
        let h = 0;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          const low = reachRegion(level, occupied, b, mark, ++stamp, stack);
          key = keyOf(next, low);
          h = assignmentHeuristic(next, dists, nGoals);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || h >= 1e9) continue;
        const prevIdx = best.get(key);
        if (prevIdx !== undefined && sG[prevIdx] <= g + 1) continue;
        const idx = sBoxes.length;
        best.set(key, idx);
        sBoxes.push(next);
        sPlayer.push(b);
        sParent.push(s);
        sFrom.push(b);
        sTo.push(t);
        sG.push(g + 1);
        if (solved(next)) {
          found = idx;
          break;
        }
        heapPush([g + 1 + h, h, seq++, idx]);
      }
    }
    for (const b of boxes) occupied[b] = 0;
  }
  if (found < 0) return null;

  // Rebuild the moves: walk to each push square, then push.
  const path = [];
  for (let s = found; s > 0; s = sParent[s]) path.push(s);
  path.reverse();
  let moves = '';
  let player = level.player;
  let boxes = level.boxes.slice();
  for (const s of path) {
    const from = sFrom[s];
    const to = sTo[s];
    const k = DX.findIndex((_, kk) => DY[kk] * w + DX[kk] === to - from);
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(level, occupied, player, from - (DY[k] * w + DX[k]));
    for (const b of boxes) occupied[b] = 0;
    moves += PUSH_LETTER[k];
    player = from;
    boxes = sBoxes[s].slice();
  }
  return { pushes: path.length, moves, endPlayer: player };
}

// ------------------------------------------------------------ room split
// Cut the hallway row's dead squares (a box there reaches no goal) out of
// the floor; each remaining component holding boxes is a room.
function roomsOf(level) {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (let k = 0; k < 4; k++) {
        const q = squares[i] + DY[k] * w + DX[k];
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [] });
  }
  for (const b of boxes) rooms[part[b]].boxes.push(b);
  for (const g of goals) if (part[g] >= 0) rooms[part[g]].goals.push(g);
  if (goals.some((g) => part[g] < 0)) return null;
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

// ------------------------------------------------------------- whole level
function solveLevel(text) {
  const level = parseLevel(text);
  const rooms = roomsOf(level);
  if (rooms !== null) {
    let player = level.player;
    let moves = '';
    let pushes = 0;
    // Current box positions, updated as rooms are solved (a solved room's
    // boxes sit on its goals and act as walls for the rooms still to come).
    const curBoxes = level.boxes.slice();
    for (const room of rooms) {
      const wall = level.wall.slice();
      for (const b of curBoxes) if (!room.boxes.includes(b)) wall[b] = 1;
      const sub = {
        w: level.w, h: level.h, wall,
        goals: room.goals.slice().sort((a, b) => a - b),
        boxes: room.boxes.slice().sort((a, b) => a - b),
        player,
      };
      const solved = solvePushAStar(sub, ROOM_BUDGET);
      if (solved === null) return solveWhole(level);
      pushes += solved.pushes;
      moves += solved.moves;
      player = solved.endPlayer;
      // This room's boxes are now on its goals.
      const remaining = curBoxes.filter((b) => !room.boxes.includes(b));
      curBoxes.length = 0;
      curBoxes.push(...remaining, ...room.goals.slice().sort((a, b) => a - b));
    }
    const out = { pushes, moves };
    if (verify(level, moves)) return out;
    return solveWhole(level);
  }
  return solveWhole(level);
}

function solveWhole(level) {
  const solved = solvePushAStar(level, WHOLE_BUDGET);
  if (solved === null) return null;
  return verify(level, solved.moves) ? solved : null;
}

// Replay moves on the level; true iff every box ends on a goal.
function verify(level, moves) {
  const { w, wall, goals } = level;
  const goalSet = new Set(goals);
  const boxes = new Set(level.boxes);
  let p = level.player;
  for (const ch of moves) {
    const l = ch.toLowerCase();
    const k = l === 'u' ? 0 : l === 'd' ? 1 : l === 'l' ? 2 : 3;
    const d = DY[k] * w + DX[k];
    const q = p + d;
    if (wall[q]) return false;
    if (boxes.has(q)) {
      const t = q + d;
      if (wall[t] || boxes.has(t)) return false;
      boxes.delete(q);
      boxes.add(t);
    }
    p = q;
  }
  for (const b of boxes) if (!goalSet.has(b)) return false;
  return true;
}

// ------------------------------------------------------------------- main
for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    const solved = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
    if (solved) moves = solved.moves;
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
