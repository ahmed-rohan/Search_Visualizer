"""
Dynamic Pathfinding Agent - GBFS and A* Search Visualizer

Student: Ahmed Rohan
Roll No: 23F-0550
Section: 6-A
Course: Artificial Intelligence
Assignment: 2

Features: Dynamic grid, random maps, GBFS/A*, Manhattan/Euclidean heuristics,
          Dynamic obstacle spawning with re-planning

Run: python search_visualizer.py
"""

import pygame
import heapq
import math
import time
import random

# ─── Configuration ────────────────────────────────────────────────────────────
CELL_SIZE = 30
PANEL_WIDTH = 280
FPS = 60
SEARCH_DELAY = 30      # ms between search steps
AGENT_DELAY = 150      # ms between agent moves

# Colors
WHITE = (255, 255, 255)
BLACK = (30, 30, 30)
GRAY = (150, 150, 150)
GREEN = (50, 200, 80)      # Start
RED = (220, 50, 50)        # Goal
DARK = (40, 40, 40)        # Wall
YELLOW = (255, 220, 0)     # Frontier
BLUE = (100, 149, 237)     # Visited
PATH_GREEN = (50, 220, 130)  # Path
ORANGE = (255, 165, 0)     # Agent
PANEL_BG = (35, 35, 50)
BTN_COLOR = (60, 80, 140)
BTN_HOVER = (90, 120, 200)
BTN_ACTIVE = (40, 180, 100)

EMPTY, WALL, START, GOAL = 0, 1, 2, 3


# ─── Heuristic Functions ──────────────────────────────────────────────────────
def manhattan(a, b):
    """Manhattan: |x1-x2| + |y1-y2|"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean(a, b):
    """Euclidean: sqrt((x1-x2)² + (y1-y2)²)"""
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


# ─── Search Algorithms ────────────────────────────────────────────────────────
def get_neighbors(pos, grid, rows, cols):
    """Get valid 4-directional neighbors."""
    r, c = pos
    neighbors = []
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != WALL:
            neighbors.append((nr, nc))
    return neighbors


def search(grid, start, goal, rows, cols, heuristic, use_astar=True):
    """
    Unified search: A* if use_astar=True, else GBFS
    A*: f(n) = g(n) + h(n)
    GBFS: f(n) = h(n)
    Yields steps for visualization.
    """
    open_set = [(heuristic(start, goal), 0, start)]
    counter = 1
    came_from = {start: None}
    g_score = {start: 0}
    visited = set()
    start_time = time.time()

    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        
        visited.add(current)
        frontier = set(n for _, _, n in open_set)
        yield ('step', current, visited.copy(), frontier, None, len(visited), 0, 0)

        if current == goal:
            # Reconstruct path
            path = []
            node = goal
            while node:
                path.append(node)
                node = came_from[node]
            path.reverse()
            elapsed = (time.time() - start_time) * 1000
            cost = g_score[goal] if use_astar else len(path) - 1
            yield ('done', None, visited, set(), path, len(visited), cost, elapsed)
            return

        for nb in get_neighbors(current, grid, rows, cols):
            new_g = g_score[current] + 1
            if nb not in g_score or new_g < g_score[nb]:
                g_score[nb] = new_g
                came_from[nb] = current
                # A*: g + h, GBFS: h only
                f = (new_g + heuristic(nb, goal)) if use_astar else heuristic(nb, goal)
                heapq.heappush(open_set, (f, counter, nb))
                counter += 1

    elapsed = (time.time() - start_time) * 1000
    yield ('done', None, visited, set(), None, len(visited), 0, elapsed)


def find_path(grid, start, goal, rows, cols, heuristic, use_astar):
    """Run search to completion and return path."""
    for step in search(grid, start, goal, rows, cols, heuristic, use_astar):
        if step[0] == 'done':
            return step[4]  # path or None
    return None


# ─── Main Application ─────────────────────────────────────────────────────────
class PathfindingApp:
    def __init__(self):
        pygame.init()
        
        # Grid settings
        self.rows = 20
        self.cols = 25
        self.density = 0.25
        self.spawn_rate = 0.05
        
        # Algorithm settings
        self.algo = 0      # 0=GBFS, 1=A*
        self.heur = 0      # 0=Manhattan, 1=Euclidean
        self.dynamic = False
        
        self._init_grid()
        self._setup_window()
        
        # State
        self.state = 'idle'
        self.visited = set()
        self.frontier = set()
        self.path = []
        self.agent_pos = None
        self.remaining_path = []
        self.gen = None
        self.last_step = 0
        
        # Metrics
        self.nodes_visited = 0
        self.path_cost = 0
        self.elapsed = 0
        self.replans = 0

    def _init_grid(self):
        """Initialize empty grid with start and goal."""
        self.grid = [[EMPTY] * self.cols for _ in range(self.rows)]
        self.start = (1, 1)
        self.goal = (self.rows - 2, self.cols - 2)
        self.grid[self.start[0]][self.start[1]] = START
        self.grid[self.goal[0]][self.goal[1]] = GOAL

    def _setup_window(self):
        """Setup pygame window."""
        self.grid_w = self.cols * CELL_SIZE
        self.grid_h = self.rows * CELL_SIZE
        self.win_w = self.grid_w + PANEL_WIDTH
        self.win_h = max(self.grid_h, 550)
        
        self.screen = pygame.display.set_mode((self.win_w, self.win_h))
        pygame.display.set_caption("Dynamic Pathfinding - GBFS / A*")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Arial", 14)
        self.font_bold = pygame.font.SysFont("Arial", 14, bold=True)
        
        # Build buttons
        self._build_buttons()

    def _build_buttons(self):
        """Create button rectangles."""
        x = self.grid_w + 15
        w, h = 120, 28
        y = 40
        
        self.btns = {
            'gbfs': pygame.Rect(x, y, 60, h),
            'astar': pygame.Rect(x + 65, y, 60, h),
            'manhattan': pygame.Rect(x, y + 50, 60, h),
            'euclidean': pygame.Rect(x + 65, y + 50, 60, h),
            'dynamic': pygame.Rect(x, y + 100, w, h),
            'run': pygame.Rect(x, y + 150, 60, h),
            'reset': pygame.Rect(x + 65, y + 150, 60, h),
            'random': pygame.Rect(x, y + 185, w, h),
            'clear': pygame.Rect(x, y + 220, w, h),
        }
        self.density_slider = pygame.Rect(x, y + 265, w, 16)
        self.spawn_slider = pygame.Rect(x, y + 305, w, 16)

    def generate_random(self):
        """Generate random obstacles."""
        self._init_grid()
        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) not in (self.start, self.goal):
                    if random.random() < self.density:
                        self.grid[r][c] = WALL

    def reset(self):
        """Reset search state."""
        self.state = 'idle'
        self.visited = set()
        self.frontier = set()
        self.path = []
        self.agent_pos = None
        self.remaining_path = []
        self.gen = None
        self.nodes_visited = 0
        self.path_cost = 0
        self.elapsed = 0
        self.replans = 0

    def start_search(self):
        """Start pathfinding."""
        self.reset()
        h = manhattan if self.heur == 0 else euclidean
        use_astar = (self.algo == 1)
        self.gen = search(self.grid, self.start, self.goal, 
                         self.rows, self.cols, h, use_astar)
        self.state = 'searching'
        self.last_step = pygame.time.get_ticks()

    def replan(self):
        """Replan path from current agent position."""
        h = manhattan if self.heur == 0 else euclidean
        use_astar = (self.algo == 1)
        path = find_path(self.grid, self.agent_pos, self.goal,
                        self.rows, self.cols, h, use_astar)
        if path:
            self.path = path
            self.remaining_path = path[1:]
            self.replans += 1
        else:
            self.state = 'no_path'
            self.remaining_path = []

    def handle_events(self):
        """Handle pygame events."""
        mx, my = pygame.mouse.get_pos()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return False
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                # Button clicks
                if self.state == 'idle':
                    if self.btns['gbfs'].collidepoint(mx, my):
                        self.algo = 0
                    elif self.btns['astar'].collidepoint(mx, my):
                        self.algo = 1
                    elif self.btns['manhattan'].collidepoint(mx, my):
                        self.heur = 0
                    elif self.btns['euclidean'].collidepoint(mx, my):
                        self.heur = 1
                    elif self.btns['dynamic'].collidepoint(mx, my):
                        self.dynamic = not self.dynamic
                    elif self.btns['random'].collidepoint(mx, my):
                        self.generate_random()
                    elif self.btns['clear'].collidepoint(mx, my):
                        self._init_grid()
                
                if self.btns['run'].collidepoint(mx, my) and self.state in ('idle', 'done', 'no_path'):
                    self.start_search()
                elif self.btns['reset'].collidepoint(mx, my):
                    self.reset()
                
                # Grid clicks
                if mx < self.grid_w and my < self.grid_h and self.state == 'idle':
                    r, c = my // CELL_SIZE, mx // CELL_SIZE
                    if event.button == 1:  # Left - toggle wall
                        if (r, c) not in (self.start, self.goal):
                            self.grid[r][c] = EMPTY if self.grid[r][c] == WALL else WALL
                    elif event.button == 3:  # Right - set start
                        if (r, c) != self.goal:
                            self.grid[self.start[0]][self.start[1]] = EMPTY
                            self.start = (r, c)
                            self.grid[r][c] = START
                    elif event.button == 2:  # Middle - set goal
                        if (r, c) != self.start:
                            self.grid[self.goal[0]][self.goal[1]] = EMPTY
                            self.goal = (r, c)
                            self.grid[r][c] = GOAL
            
            # Slider dragging
            if event.type == pygame.MOUSEMOTION and pygame.mouse.get_pressed()[0]:
                if self.density_slider.collidepoint(mx, my):
                    self.density = max(0.1, min(0.5, (mx - self.density_slider.x) / self.density_slider.w))
                if self.spawn_slider.collidepoint(mx, my):
                    self.spawn_rate = max(0.01, min(0.15, (mx - self.spawn_slider.x) / self.spawn_slider.w))
        
        return True

    def update(self):
        """Update search/agent state."""
        now = pygame.time.get_ticks()
        
        if self.state == 'searching' and self.gen:
            if now - self.last_step >= SEARCH_DELAY:
                self.last_step = now
                try:
                    step = next(self.gen)
                    kind, _, vis, fron, path, nv, cost, el = step
                    self.visited = vis
                    self.frontier = fron
                    self.nodes_visited = nv
                    
                    if kind == 'done':
                        self.elapsed = el
                        if path:
                            self.path = path
                            self.path_cost = cost
                            if self.dynamic:
                                self.state = 'moving'
                                self.agent_pos = self.start
                                self.remaining_path = path[1:]
                            else:
                                self.state = 'done'
                        else:
                            self.state = 'no_path'
                        self.gen = None
                except StopIteration:
                    self.state = 'done'
                    self.gen = None
        
        elif self.state == 'moving':
            if now - self.last_step >= AGENT_DELAY:
                self.last_step = now
                
                # Spawn random obstacle
                if self.dynamic and random.random() < self.spawn_rate:
                    empties = [(r, c) for r in range(self.rows) for c in range(self.cols)
                              if self.grid[r][c] == EMPTY and (r, c) not in self.remaining_path
                              and (r, c) != self.agent_pos]
                    if empties:
                        nr, nc = random.choice(empties)
                        self.grid[nr][nc] = WALL
                        # Check if path is blocked
                        if (nr, nc) in self.remaining_path:
                            self.replan()
                
                # Move agent
                if self.remaining_path:
                    next_pos = self.remaining_path[0]
                    if self.grid[next_pos[0]][next_pos[1]] == WALL:
                        self.replan()
                    else:
                        self.agent_pos = next_pos
                        self.remaining_path = self.remaining_path[1:]
                        if self.agent_pos == self.goal:
                            self.state = 'done'
                elif self.agent_pos != self.goal:
                    self.replan()

    def draw(self):
        """Draw everything."""
        self.screen.fill(BLACK)
        
        # Draw grid
        path_set = set(self.path)
        for r in range(self.rows):
            for c in range(self.cols):
                x, y = c * CELL_SIZE, r * CELL_SIZE
                pos = (r, c)
                cell = self.grid[r][c]
                
                if pos == self.agent_pos:
                    color = ORANGE
                elif cell == START:
                    color = GREEN
                elif cell == GOAL:
                    color = RED
                elif cell == WALL:
                    color = DARK
                elif pos in path_set:
                    color = PATH_GREEN
                elif pos in self.visited:
                    color = BLUE
                elif pos in self.frontier:
                    color = YELLOW
                else:
                    color = WHITE
                
                pygame.draw.rect(self.screen, color, (x+1, y+1, CELL_SIZE-2, CELL_SIZE-2), border_radius=3)
        
        # Grid lines
        for r in range(self.rows + 1):
            pygame.draw.line(self.screen, GRAY, (0, r*CELL_SIZE), (self.grid_w, r*CELL_SIZE))
        for c in range(self.cols + 1):
            pygame.draw.line(self.screen, GRAY, (c*CELL_SIZE, 0), (c*CELL_SIZE, self.grid_h))
        
        # Panel
        px = self.grid_w
        pygame.draw.rect(self.screen, PANEL_BG, (px, 0, PANEL_WIDTH, self.win_h))
        
        # Title
        title = self.font_bold.render("Dynamic Pathfinding Agent", True, (180, 210, 255))
        self.screen.blit(title, (px + 10, 12))
        
        # Algorithm buttons
        self._draw_btn(self.btns['gbfs'], "GBFS", self.algo == 0)
        self._draw_btn(self.btns['astar'], "A*", self.algo == 1)
        
        # Heuristic buttons
        self._draw_btn(self.btns['manhattan'], "Manhat", self.heur == 0)
        self._draw_btn(self.btns['euclidean'], "Euclid", self.heur == 1)
        
        # Dynamic toggle
        dyn_txt = f"Dynamic: {'ON' if self.dynamic else 'OFF'}"
        self._draw_btn(self.btns['dynamic'], dyn_txt, self.dynamic)
        
        # Control buttons
        self._draw_btn(self.btns['run'], "Run")
        self._draw_btn(self.btns['reset'], "Reset")
        self._draw_btn(self.btns['random'], "Random Map")
        self._draw_btn(self.btns['clear'], "Clear")
        
        # Sliders
        self._draw_slider(self.density_slider, self.density, 0.1, 0.5, f"Density: {int(self.density*100)}%")
        if self.dynamic:
            self._draw_slider(self.spawn_slider, self.spawn_rate, 0.01, 0.15, f"Spawn: {int(self.spawn_rate*100)}%")
        
        # Metrics
        y = 340
        self.screen.blit(self.font_bold.render("Metrics", True, GRAY), (px + 10, y))
        y += 22
        metrics = [
            f"Nodes Visited: {self.nodes_visited}",
            f"Path Cost: {self.path_cost}",
            f"Time: {self.elapsed:.2f} ms",
        ]
        if self.dynamic:
            metrics.append(f"Replans: {self.replans}")
        for m in metrics:
            self.screen.blit(self.font.render(m, True, WHITE), (px + 15, y))
            y += 18
        
        # Legend
        y += 10
        self.screen.blit(self.font_bold.render("Legend", True, GRAY), (px + 10, y))
        y += 20
        legend = [(GREEN, "Start"), (RED, "Goal"), (DARK, "Wall"), 
                  (YELLOW, "Frontier"), (BLUE, "Visited"), (PATH_GREEN, "Path")]
        if self.dynamic:
            legend.append((ORANGE, "Agent"))
        for color, label in legend:
            pygame.draw.rect(self.screen, color, (px + 15, y, 14, 14), border_radius=2)
            self.screen.blit(self.font.render(label, True, WHITE), (px + 35, y))
            y += 18
        
        # Status
        y += 10
        status = {'idle': 'Ready', 'searching': 'Searching...', 'moving': 'Agent Moving...',
                  'done': 'Done!', 'no_path': 'No Path!'}
        s = status.get(self.state, self.state)
        color = RED if self.state == 'no_path' else (BTN_ACTIVE if self.state == 'done' else WHITE)
        self.screen.blit(self.font_bold.render(f"Status: {s}", True, color), (px + 10, y))
        
        # Controls help
        y += 30
        self.screen.blit(self.font.render("L-click: Wall | R-click: Start", True, GRAY), (px + 10, y))
        self.screen.blit(self.font.render("M-click: Goal | ESC: Exit", True, GRAY), (px + 10, y + 16))
        
        pygame.display.flip()

    def _draw_btn(self, rect, text, active=False):
        """Draw a button."""
        mx, my = pygame.mouse.get_pos()
        hover = rect.collidepoint(mx, my)
        color = BTN_ACTIVE if active else (BTN_HOVER if hover else BTN_COLOR)
        pygame.draw.rect(self.screen, color, rect, border_radius=5)
        lbl = self.font.render(text, True, WHITE)
        self.screen.blit(lbl, lbl.get_rect(center=rect.center))

    def _draw_slider(self, rect, value, min_v, max_v, label):
        """Draw a slider."""
        pygame.draw.rect(self.screen, (50, 50, 70), rect, border_radius=3)
        fill_w = int((value - min_v) / (max_v - min_v) * rect.w)
        pygame.draw.rect(self.screen, BTN_COLOR, (rect.x, rect.y, fill_w, rect.h), border_radius=3)
        self.screen.blit(self.font.render(label, True, WHITE), (rect.x, rect.y - 16))

    def run(self):
        """Main loop."""
        running = True
        while running:
            running = self.handle_events()
            self.update()
            self.draw()
            self.clock.tick(FPS)
        pygame.quit()


if __name__ == "__main__":
    app = PathfindingApp()
    app.run()
