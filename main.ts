// Sokoban solver for sokoban-solver-01-run-20261121 (make.fun bounty).
//
// Strategy: the generator builds every level as a one-square hallway with
// independent rooms off it, and a level's push optimum is the sum of its
// rooms' optima. This solver recovers that decomposition itself (it cuts the
// dead squares of the player's hallway row out of the floor; each remaining
// component holding boxes is one room), then solves each room push-optimally
// with an A* search over pushes:
//
//   state      = the room's boxes (sorted) + the player's position, with the
//                transposition key normalised to the lowest square of the
//                player's flood-fill reachable region;
//   heuristic  = minimum-cost bipartite matching of boxes to goals over
//                precomputed single-box push distances (exact by permutation
//                enumeration at 3-4 boxes; admissible, so A* stays optimal);
//   pruning    = a static dead-square table (reverse pull-BFS from the
//                goals) and frozen 2x2 blocks (walls/boxes with a box off
//                its goal).
//
// Rooms are solved in scan order; the other rooms' boxes stand as walls.
// The room's push sequence is then stitched into player moves by BFS walks
// to each push's stand square (walks lower-case, pushes upper-case).
//
// Determinism: no clock, no randomness anywhere. Search budgets are fixed
// expansion counters, move order and heap tie-breaking are fixed, so two
// runs on the same set write byte-identical solutions.
//
// Local run: MAKEFUN_INPUT_DIR=<in> MAKEFUN_OUTPUT_DIR=<out> node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

/** Directions in a fixed order: [dx, dy, walk letter, push letter]. */
const DIRS: [number, number, string, string][] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

/** Fixed per-room expansion budget (work, never the clock), scaled by boxes. */
const budgetFor = (boxes: number): number => 400_000 + 400_000 * boxes;

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x] as string;
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

/**
 * Reverse pull-BFS from every goal over `wall`: dist[i * nGoals + gi] is the
 * fewest pushes that take a lone box from square i to goal gi (the player
 * teleports between stand squares, so this never overestimates), or -1 when
 * no push sequence reaches that goal. A square live for some goal has a
 * non-negative entry; a square dead for all goals can never hold a box in a
 * solution (the static dead-square table).
 */
function pushDistances(w: number, wall: Uint8Array, goals: number[]): Int32Array {
  const size = wall.length;
  const n = goals.length;
  const dist = new Int32Array(size * n).fill(-1);
  const queue: number[] = [];
  goals.forEach((g, gi) => {
    dist[g * n + gi] = 0;
    queue.push(g * n + gi);
  });
  for (let qi = 0; qi < queue.length; qi++) {
    const cur = queue[qi] as number;
    const cell = Math.floor(cur / n);
    const gi = cur % n;
    const d0 = dist[cur] as number;
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const prev = cell - d; // a box here, pushed by d, lands on cell
      const stand = prev - d; // the square the player stood on for that push
      if (prev < 0 || stand < 0 || prev >= size || stand >= size) continue;
      if (wall[prev] || wall[stand]) continue;
      const key = prev * n + gi;
      if (dist[key] !== -1) continue;
      dist[key] = d0 + 1;
      queue.push(key);
    }
  }
  return dist;
}

/** All permutations of [0..n), in lexicographic order. */
function permutations(n: number): number[][] {
  if (n <= 1) return [Array.from({ length: n }, (_, i) => i)];
  const out: number[][] = [];
  const base = Array.from({ length: n }, (_, i) => i);
  const rec = (prefix: number[], rest: number[]): void => {
    if (rest.length === 0) {
      out.push(prefix);
      return;
    }
    for (let i = 0; i < rest.length; i++) {
      rec([...prefix, rest[i] as number], rest.filter((_, j) => j !== i));
    }
  };
  rec([], base);
  return out;
}

interface RoomProblem {
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

/**
 * Push-optimal A* for one room. Returns the push sequence as
 * [box square before the push, direction index] pairs, or null when the
 * expansion budget runs out or the room cannot be solved.
 */
function solveRoom(
  w: number,
  size: number,
  prob: RoomProblem,
): [number, number][] | null {
  const { wall, goals, player } = prob;
  const n = prob.boxes.length;
  const goalSet = new Uint8Array(size);
  for (const g of goals) goalSet[g] = 1;
  const dist = pushDistances(w, wall, goals);
  const perms = permutations(n);
  const INF = Number.POSITIVE_INFINITY;

  // Admissible bound: cheapest assignment of distinct goals to boxes.
  const heuristic = (boxes: number[]): number => {
    let best = INF;
    for (const perm of perms) {
      let sum = 0;
      for (let i = 0; i < n; i++) {
        const d = dist[(boxes[i] as number) * n + (perm[i] as number)] as number;
        if (d < 0) {
          sum = INF;
          break;
        }
        sum += d;
      }
      if (sum < best) best = sum;
    }
    return best;
  };

  const occupied = new Uint8Array(size);
  // Two separate stamp fields: `seen` marks the region of the state being
  // expanded (its stand squares are tested against it for every push),
  // `mark` normalises a child's region. Sharing one field would let a
  // child's flood fill overwrite the parent's marks mid-expansion.
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  // Flood fill from `from` around walls and boxes, marking into `field`
  // with a fresh stamp; returns the region's lowest square.
  const reach = (from: number, field: Int32Array): number => {
    stamp++;
    let low = from;
    let sp = 0;
    stack[sp++] = from;
    field[from] = stamp;
    while (sp > 0) {
      const p = stack[--sp] as number;
      if (p < low) low = p;
      for (let k = 0; k < 4; k++) {
        const q = k === 0 ? p - w : k === 1 ? p + w : k === 2 ? p - 1 : p + 1;
        if (q >= 0 && q < size && wall[q] === 0 && occupied[q] === 0 && field[q] !== stamp) {
          field[q] = stamp;
          stack[sp++] = q;
        }
      }
    }
    return low;
  };

  const frozenBlock = (t: number): boolean => {
    for (const a of [t, t - 1, t - w, t - w - 1]) {
      const b = a + 1;
      const c = a + w;
      const dd = c + 1;
      if (a < 0 || dd >= size) continue;
      const solid = (x: number): boolean => wall[x] === 1 || occupied[x] === 1;
      if (!solid(a) || !solid(b) || !solid(c) || !solid(dd)) continue;
      if (
        (occupied[a] && !goalSet[a]) ||
        (occupied[b] && !goalSet[b]) ||
        (occupied[c] && !goalSet[c]) ||
        (occupied[dd] && !goalSet[dd])
      ) {
        return true;
      }
    }
    return false;
  };

  // Wall-line deadlock: a box at t (off goal) pressed against a wall. Walk
  // the maximal run of squares that share that wall on the same side; the
  // box can only ever slide along the run (a push away from the wall needs
  // the player inside the wall, a push into it is blocked). When both ends
  // of the run are closed by wall squares and the run holds no goal, the
  // box can never reach a goal: prune.
  const wallLineDead = (t: number): boolean => {
    if (goalSet[t] === 1) return false;
    for (const [perp, step] of [
      [-w, 1],
      [w, 1],
      [-1, w],
      [1, w],
    ] as [number, number][]) {
      const wp = t + perp;
      if (wp < 0 || wp >= size || wall[wp] !== 1) continue;
      let hasGoal = false;
      let closed = true;
      for (const sgn of [-1, 1]) {
        let cur = t;
        for (;;) {
          const nxt = cur + sgn * step;
          if (nxt < 0 || nxt >= size) break; // board frame: closed
          if (wall[nxt] === 1) break; // wall square: closed end
          if (wall[nxt + perp] !== 1) {
            closed = false; // open square off the wall line: the box can leave
            break;
          }
          if (goalSet[nxt] === 1) hasGoal = true;
          cur = nxt;
        }
        if (!closed) break;
      }
      if (closed && !hasGoal) return true;
    }
    return false;
  };

  const keyOf = (boxes: number[], low: number): string => low + ':' + boxes.join(',');
  const solved = (boxes: number[]): boolean => boxes.every((b) => goalSet[b] === 1);

  // State store.
  const sBoxes: number[][] = [prob.boxes.slice()];
  const sPlayer: number[] = [player];
  const sG: number[] = [0];
  const sParent: number[] = [-1];
  const sPushBox: number[] = [-1];
  const sPushDir: number[] = [-1];
  const best = new Map<string, number>();

  // Binary heap of state indices ordered by (f, h, insertion order).
  const heap: number[] = [];
  const hCache: number[] = [];
  const less = (a: number, b: number): boolean => {
    const fa = (sG[a] as number) + (hCache[a] as number);
    const fb = (sG[b] as number) + (hCache[b] as number);
    if (fa !== fb) return fa < fb;
    if (hCache[a] !== hCache[b]) return (hCache[a] as number) < (hCache[b] as number);
    return a < b;
  };
  const heapPush = (s: number): void => {
    let i = heap.length;
    heap.push(s);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i] as number, heap[p] as number)) break;
      const tmp = heap[i] as number;
      heap[i] = heap[p] as number;
      heap[p] = tmp;
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = heap[0] as number;
    const last = heap.pop() as number;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1] as number, heap[c] as number)) c++;
        if (!less(heap[c] as number, heap[i] as number)) break;
        const tmp = heap[i] as number;
        heap[i] = heap[c] as number;
        heap[c] = tmp;
        i = c;
      }
    }
    return top;
  };

  const h0 = heuristic(prob.boxes);
  if (h0 === INF) return null;
  hCache[0] = h0;
  for (const b of prob.boxes) occupied[b] = 1;
  best.set(keyOf(prob.boxes, reach(player, seen)), 0);
  for (const b of prob.boxes) occupied[b] = 0;
  heapPush(0);

  const budget = budgetFor(n);
  let expanded = 0;
  let found = -1;
  while (heap.length > 0 && expanded < budget) {
    const s = heapPop();
    const boxes = sBoxes[s] as number[];
    const g = sG[s] as number;
    for (const b of boxes) occupied[b] = 1;
    const low = reach(sPlayer[s] as number, seen);
    const regionStamp = stamp;
    if (best.get(keyOf(boxes, low)) !== g) {
      for (const b of boxes) occupied[b] = 0;
      continue;
    }
    if (solved(boxes)) {
      found = s;
      for (const b of boxes) occupied[b] = 0;
      break;
    }
    expanded++;
    for (let bi = 0; bi < n; bi++) {
      const b = boxes[bi] as number;
      for (let di = 0; di < 4; di++) {
        const d = (DIRS[di] as [number, number, string, string])[1] * w + (DIRS[di] as [number, number, string, string])[0];
        const stand = b - d;
        const target = b + d;
        if (stand < 0 || target < 0 || stand >= size || target >= size) continue;
        if (seen[stand] !== regionStamp) continue;
        if (wall[target] === 1 || occupied[target] === 1) continue;
        // Dead-square prune: the pushed box must stay live for some goal.
        let live = false;
        for (let gi = 0; gi < n; gi++) {
          if (dist[target * n + gi] !== -1) {
            live = true;
            break;
          }
        }
        if (!live) continue;
        occupied[b] = 0;
        occupied[target] = 1;
        const frozen = frozenBlock(target);
        occupied[target] = 0;
        occupied[b] = 1;
        if (frozen) continue;
        if (wallLineDead(target)) continue;
        const next = boxes.slice();
        next[bi] = target;
        next.sort((x, y) => x - y);
        const h = heuristic(next);
        if (h === INF) continue;
        const ng = g + 1;
        occupied[b] = 0;
        occupied[target] = 1;
        const nLow = reach(b, mark); // after the push the player stands on b
        occupied[target] = 0;
        occupied[b] = 1;
        const key = keyOf(next, nLow);
        const seenG = best.get(key);
        if (seenG !== undefined && seenG <= ng) continue;
        const t = sBoxes.length;
        sBoxes.push(next);
        sPlayer.push(b);
        sG.push(ng);
        sParent.push(s);
        sPushBox.push(b);
        sPushDir.push(di);
        hCache[t] = h;
        best.set(key, ng);
        heapPush(t);
      }
    }
    for (const b of boxes) occupied[b] = 0;
  }
  if (found < 0) return null;
  const pushes: [number, number][] = [];
  for (let s = found; s > 0; s = sParent[s] as number) {
    pushes.push([sPushBox[s] as number, sPushDir[s] as number]);
  }
  pushes.reverse();
  return pushes;
}

/** Shortest player walk from `from` to `to` around walls and boxes (BFS). */
function walkTo(
  w: number,
  size: number,
  wall: Uint8Array,
  occupied: Uint8Array,
  from: number,
  to: number,
): string {
  if (from === to) return '';
  const prev = new Int32Array(size).fill(-1);
  const how = new Array<string>(size);
  const queue: number[] = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi] as number;
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (q < 0 || q >= size) continue;
      if (wall[q] === 1 || occupied[q] === 1 || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) return '';
  let s = '';
  for (let p = to; p !== from; p = prev[p] as number) s = (how[p] as string) + s;
  return s;
}

interface Room {
  boxes: number[];
  goals: number[];
}

/**
 * Split a level into rooms: cut the dead squares (for every goal) of the
 * player's own row — the hallway — out of the floor and flood-fill what is
 * left. Returns null when the level is not a clean set of rooms (a box in
 * the hallway, a goal cut off, a component whose boxes and goals differ).
 */
function splitRooms(level: Level, live: Uint8Array): Room[] | null {
  const { w, wall, goals, boxes, player } = level;
  const size = wall.length;
  const cut = new Uint8Array(size);
  const row = Math.floor(player / w);
  for (let i = row * w; i < (row + 1) * w; i++) {
    if (wall[i] === 0 && live[i] === 0) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b] === 1)) return null;
  const part = new Int32Array(size).fill(-1);
  const rooms: Room[] = [];
  const firsts: number[] = [];
  for (let s = 0; s < size; s++) {
    if (wall[s] === 1 || cut[s] === 1 || part[s] >= 0) continue;
    const idx = rooms.length;
    const squares: number[] = [s];
    part[s] = idx;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = (squares[i] as number) + dy * w + dx;
        if (q < 0 || q >= size) continue;
        if (wall[q] === 0 && cut[q] === 0 && part[q] < 0) {
          part[q] = idx;
          squares.push(q);
        }
      }
    }
    rooms.push({ boxes: [], goals: [] });
    firsts.push(s);
  }
  for (const b of boxes) rooms[part[b] as number]?.boxes.push(b);
  for (const g of goals) {
    if (part[g] < 0) return null;
    rooms[part[g] as number]?.goals.push(g);
  }
  const withBoxes = rooms
    .map((r, i) => ({ r, first: firsts[i] as number }))
    .filter(({ r }) => r.boxes.length > 0);
  if (withBoxes.some(({ r }) => r.boxes.length !== r.goals.length)) return null;
  withBoxes.sort((a, b) => a.first - b.first);
  return withBoxes.map(({ r }) => ({
    boxes: r.boxes.slice().sort((a, b) => a - b),
    goals: r.goals.slice().sort((a, b) => a - b),
  }));
}

function solveLevel(text: string): string {
  const level = parseLevel(text);
  const { w, wall, goals } = level;
  const size = wall.length;
  const distAll = pushDistances(w, wall, goals);
  const live = new Uint8Array(size);
  for (let i = 0; i < size; i++) {
    for (let gi = 0; gi < goals.length; gi++) {
      if (distAll[i * goals.length + gi] !== -1) {
        live[i] = 1;
        break;
      }
    }
  }
  if (level.boxes.some((b) => live[b] === 0)) return '';

  let rooms = splitRooms(level, live);
  if (rooms === null) rooms = [{ boxes: level.boxes.slice(), goals: goals.slice() }];

  // Global stitching state.
  let boxes = level.boxes.slice();
  let player = level.player;
  const occupied = new Uint8Array(size);
  let moves = '';

  for (const room of rooms) {
    const subWall = wall.slice();
    const roomBoxSet = new Set(room.boxes);
    for (const b of boxes) if (!roomBoxSet.has(b)) subWall[b] = 1;
    const pushes = solveRoom(w, size, {
      wall: subWall,
      goals: room.goals,
      boxes: room.boxes.slice(),
      player,
    });
    if (pushes === null) return '';
    for (const [boxFrom, di] of pushes) {
      const [dx, dy, , pushLetter] = DIRS[di] as [number, number, string, string];
      const d = dy * w + dx;
      const stand = boxFrom - d;
      for (const b of boxes) occupied[b] = 1;
      const walk = walkTo(w, size, wall, occupied, player, stand);
      for (const b of boxes) occupied[b] = 0;
      if (walk === '' && player !== stand) return '';
      moves += walk + pushLetter;
      const at = boxes.indexOf(boxFrom);
      if (at < 0) return '';
      boxes[at] = boxFrom + d;
      boxes.sort((a, b) => a - b);
      player = boxFrom;
    }
  }
  return boxes.every((b) => goals.includes(b)) ? moves : '';
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
