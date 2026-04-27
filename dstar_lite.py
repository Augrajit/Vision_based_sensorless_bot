"""D* Lite pathfinding algorithm — testable standalone module."""
import heapq, math

class DStarLite:
    def __init__(self, grid_w, grid_h, cell_size):
        self.W = grid_w
        self.H = grid_h
        self.cs = cell_size
        self.cols = grid_w // cell_size
        self.rows = grid_h // cell_size
        self.g = {}
        self.rhs = {}
        self.open_list = []
        self.open_set = set()
        self.k_m = 0
        self.start = None
        self.goal = None
        self.obstacles = []
        self.path_buffer = 13
        self._counter = 0
        self._dirs = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                self._dirs.append((dx, dy))

    def _cost(self, dx, dy):
        return self.cs * (1.4142135 if abs(dx) + abs(dy) == 2 else 1.0)

    def _h(self, a, b):
        return math.hypot((a[0]-b[0])*self.cs, (a[1]-b[1])*self.cs)

    def calculate_key(self, s):
        g_val = self.g.get(s, float('inf'))
        rhs_val = self.rhs.get(s, float('inf'))
        mn = min(g_val, rhs_val)
        return (mn + self._h(s, self.start) + self.k_m, mn)

    def _in_bounds(self, c, r):
        return 0 <= c < self.cols and 0 <= r < self.rows

    def is_obstacle(self, cell):
        cx = cell[0] * self.cs + self.cs / 2
        cy = cell[1] * self.cs + self.cs / 2
        buf = self.path_buffer
        for (ox, oy, ow, oh) in self.obstacles:
            if (ox - buf) <= cx <= (ox + ow + buf) and (oy - buf) <= cy <= (oy + oh + buf):
                return True
        return False

    def _neighbors(self, s):
        result = []
        for dc, dr in self._dirs:
            nc, nr = s[0] + dc, s[1] + dr
            if self._in_bounds(nc, nr) and not self.is_obstacle((nc, nr)):
                result.append(((nc, nr), self._cost(dc, dr)))
        return result

    def initialize(self):
        self.g.clear()
        self.rhs.clear()
        self.open_list = []
        self.open_set = set()
        self.k_m = 0
        self._counter = 0
        if self.goal is None or self.start is None:
            return
        self.rhs[self.goal] = 0
        k = self.calculate_key(self.goal)
        self._counter += 1
        heapq.heappush(self.open_list, (k, self._counter, self.goal))
        self.open_set.add(self.goal)

    def update_vertex(self, u):
        if u != self.goal:
            min_rhs = float('inf')
            for nb, cost in self._neighbors(u):
                val = self.g.get(nb, float('inf')) + cost
                if val < min_rhs:
                    min_rhs = val
            self.rhs[u] = min_rhs
        if u in self.open_set:
            self.open_set.discard(u)
        g_val = self.g.get(u, float('inf'))
        rhs_val = self.rhs.get(u, float('inf'))
        if g_val != rhs_val:
            k = self.calculate_key(u)
            self._counter += 1
            heapq.heappush(self.open_list, (k, self._counter, u))
            self.open_set.add(u)

    def compute_shortest_path(self):
        if self.start is None or self.goal is None:
            return
        iterations = 0
        max_iter = self.cols * self.rows * 2
        s_key = self.calculate_key(self.start)
        while self.open_list:
            if iterations > max_iter:
                break
            top_key, _, u = self.open_list[0]
            g_s = self.g.get(self.start, float('inf'))
            rhs_s = self.rhs.get(self.start, float('inf'))
            if not (top_key < s_key or rhs_s != g_s):
                break
            heapq.heappop(self.open_list)
            self.open_set.discard(u)
            g_u = self.g.get(u, float('inf'))
            rhs_u = self.rhs.get(u, float('inf'))
            if g_u > rhs_u:
                self.g[u] = rhs_u
                for nb, cost in self._neighbors(u):
                    self.update_vertex(nb)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                for nb, cost in self._neighbors(u):
                    self.update_vertex(nb)
            s_key = self.calculate_key(self.start)
            iterations += 1

    def extract_path(self):
        if self.start is None or self.goal is None:
            return []
        path = [self.start]
        current = self.start
        visited = set()
        max_steps = self.cols * self.rows
        for _ in range(max_steps):
            if current == self.goal:
                break
            if current in visited:
                break
            visited.add(current)
            best_nb = None
            best_cost = float('inf')
            for nb, cost in self._neighbors(current):
                val = self.g.get(nb, float('inf')) + cost
                if val < best_cost:
                    best_cost = val
                    best_nb = nb
            if best_nb is None or best_cost >= float('inf'):
                return []
            path.append(best_nb)
            current = best_nb
        return path

    def cell_from_cm(self, x_cm, y_cm):
        c = max(0, min(self.cols - 1, int(x_cm / self.cs)))
        r = max(0, min(self.rows - 1, int(y_cm / self.cs)))
        return (c, r)

    def cm_from_cell(self, cell):
        return (cell[0] * self.cs + self.cs / 2, cell[1] * self.cs + self.cs / 2)

    def move_and_replan(self, robot_pos_cm):
        old_start = self.start
        self.start = self.cell_from_cm(robot_pos_cm[0], robot_pos_cm[1])
        if old_start and old_start != self.start:
            self.k_m += self._h(old_start, self.start)
        changed = set()
        for c in range(self.cols):
            for r in range(self.rows):
                cell = (c, r)
                is_obs = self.is_obstacle(cell)
                g_val = self.g.get(cell, float('inf'))
                if is_obs and g_val < float('inf'):
                    changed.add(cell)
                elif not is_obs and g_val == float('inf') and cell != self.start:
                    changed.add(cell)
        for cell in changed:
            self.update_vertex(cell)
            for nb, _ in self._neighbors(cell):
                self.update_vertex(nb)
        self.compute_shortest_path()
        return self.extract_path()
