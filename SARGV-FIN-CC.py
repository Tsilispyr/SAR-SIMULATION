# -*- coding: utf-8 -*-
"""
SAR - Search and Rescue Simulation System
Έκδοση 7.0

Περιλαμβάνει:
- Πάνελ στατιστικών σε πραγματικό χρόνο (με Matplotlib)
- Ξεχωριστά αρχεία JSON για κάθε σενάριο (stats_1.json .. stats_4.json)
- Λογική Co-op (οι εχθροί στοχεύουν τον πλησιέστερο ενεργό)
- Λογική Co-op (η αποστολή λήγει μόνο αν χάσουν όλοι)
- Κόστος κίνησης (score penalty) σε κάθε βήμα
- Σταθερό σενάριο ελέγχου (Test Map)
- Βελτιωμένα μενού επιλογών
- Ονόματα Agents (A, B, C, D) και καταγραφή στατιστικών ανά agent

Απαιτεί: pip install pygame matplotlib pandas
"""

import pygame
import random
import math
import heapq
import json
import collections
import os
import sys
import time
from io import BytesIO

# Matplotlib (για σχεδίαση γραφημάτων και μετατροπή σε pygame surface)
try:
    import matplotlib
    matplotlib.use("Agg")  # Χρήση "Agg" backend για σχεδίαση εκτός-οθόνης
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: 'matplotlib' library not found. Stats panel will be disabled.")
    print("Install it with: pip install matplotlib")

# Pandas (για moving average στο γράφημα ταχύτητας)
try:
    import pandas as pd
    PANDAS_AVAILABLE = True
except ImportError:
    PANDAS_AVAILABLE = False
    print("Warning: 'pandas' library not found. Speed chart will be disabled.")
    print("Install it with: pip install pandas")


# CONFIG   

# Διαστάσεις πλέγματος και παιχνιδιού
TILE_SIZE = 15
GRID_WIDTH_CELLS = 80 
GRID_HEIGHT_CELLS = 30
GAME_WIDTH = GRID_WIDTH_CELLS * TILE_SIZE
GAME_HEIGHT = GRID_HEIGHT_CELLS * TILE_SIZE

# Πάνελ Στατιστικών
PANEL_WIDTH = 700  # Σταθερό πλάτος
STATS_REDRAW_INTERVAL = 0.8 # Διάστημα ανανέωσης γραφημάτων (δευτερόλεπτα)

# Τελικό μέγεθος οθόνης
TOTAL_SCREEN_WIDTH = GAME_WIDTH + PANEL_WIDTH
# TOTAL_SCREEN_HEIGHT = GAME_HEIGHT
TOTAL_SCREEN_HEIGHT = GAME_HEIGHT * 2

# Προεπιλεγμένες παράμετροι
DEFAULT_TARGETS = 10
DEFAULT_PATROL = 3
DEFAULT_AGGRESSIVE = 2

# ΧΡΩΜΑΤΑ ΚΑΙ ΤΥΠΟΙ ΚΕΛΙΩΝ

# Βασικά χρώματα
BLACK, WHITE = (0, 0, 0), (255, 255, 255)

# Χρώματα για διάφορους τύπους εδάφους
COLOR_FREE = (200, 200, 200)
COLOR_OBSTACLE = (50, 50, 50)
COLOR_DIFFICULT = (100, 100, 160)
COLOR_HAZARD = (255, 180, 180)

# Χρώματα οχημάτων
AGENT_COLORS = [(0, 120, 255), (0, 200, 200), (120, 180, 120), (200, 120, 200)] # 4 χρώματα

# Χρώματα UI
COLOR_GOAL_OUTLINE = (255, 215, 0)
COLOR_PATH = (0, 150, 0, 150)
COLOR_COLLECTIBLE = (255, 255, 0)
COLOR_PANEL_BG = (10, 20, 35)
COLOR_PANEL_BORDER = (50, 80, 120)

# Χρώματα εχθρών
COLOR_PATROL_OBSTACLE = (180, 0, 0)
COLOR_AGGRESSIVE_OBSTACLE = (255, 0, 255)

# Χρώματα μενού
MENU_BG_COLOR = (10, 25, 47)
MENU_TEXT_COLOR = (200, 200, 255)
MENU_HIGHLIGHT_COLOR = (255, 128, 0)

# Τύποι Κελιών
TILE_TYPE_FREE = 0
TILE_TYPE_OBSTACLE = 1
TILE_TYPE_DIFFICULT = 2
TILE_TYPE_HAZARD = 3

# Σκορ & Κόστος
BASE_TARGET_SCORE = 50 # Αυξήθηκε για να αξίζει
TILE_COSTS = {
    TILE_TYPE_FREE: 0,
    TILE_TYPE_DIFFICULT: 1,   # Κόστος ανά βήμα
    TILE_TYPE_HAZARD: 3,    # Κόστος ανά βήμα
    TILE_TYPE_OBSTACLE: 0,
}

# Αρχεία Στατιστικών
STATS_FILES = {
    1: "stats_1.json",
    2: "stats_2.json",
    3: "stats_3.json",
    4: "stats_4.json",
    "custom": "stats_custom.json"
}
MASTER_STATS_FILE = "stats_all_scenarios.json"

#  UTILITIES 

def safe_json_load(path, default):
    """ Φορτώνει με ασφάλεια ένα JSON, επιστρέφει default αν αποτύχει. """
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return default
    else:
        return default

def safe_json_save(path, obj):
    """ Αποθηκεύει με ασφάλεια ένα JSON. """
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(obj, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f"Σφάλμα αποθήκευσης {path}: {e}")

def get_stats_filename(num_agents):
    """ Επιστρέφει το σωστό όνομα αρχείου .json βάσει αριθμού ρομπότ. """
    return STATS_FILES.get(num_agents, "stats_default.json")

def ensure_stats_files_exist():
    """ Δημιουργεί κενά αρχεία στατιστικών αν δεν υπάρχουν. """
    default_summary = {
        "total_missions": 0, "success": 0, "partial": 0, "failed": 0,
        "avg_time": 0.0, "avg_score": 0.0, "best_time": float('inf')
    }
    for n, fn in STATS_FILES.items():
        if not os.path.exists(fn):
            safe_json_save(fn, {"missions": [], "summary": default_summary.copy()})
    if not os.path.exists(MASTER_STATS_FILE):
        safe_json_save(MASTER_STATS_FILE, {"by_agents": {str(i): [] for i in (1,2,3,4)}})


class MapEditor:           
    def __init__(self, screen, use_stable_layout=False, previous_data=None):
        self.screen = screen
        self.font = pygame.font.SysFont("Arial", 16)
        self.font_bold = pygame.font.SysFont("Arial", 16, bold=True)
        self.running = True
        self.aborted = False 
        
        # Αρχικοποίηση λιστών
        self.custom_agents = []
        self.custom_targets = []
        self.custom_patrols = []
        self.custom_aggressives = []
        self.game_map = []

        # --- ΛΟΓΙΚΗ ΦΟΡΤΩΣΗΣ ---
        if previous_data:
            # 1. Edit Mode: Αν γυρίσαμε από το παιχνίδι, φόρτωσε ό,τι είχαμε φτιάξει
            self.game_map = [row[:] for row in previous_data['map']]
            self.custom_agents = list(previous_data['agents'])
            self.custom_targets = list(previous_data['targets'])
            self.custom_patrols = list(previous_data['patrols'])
            self.custom_aggressives = list(previous_data['aggressives'])
            
        elif use_stable_layout:
            # 2. New Template: Φτιάξε τον "Σταθερό" χάρτη
            self.generate_stable_map()
        else:
            # 3. Blank: Κενός χάρτης
            self.clear_all_map()

        self.selected_tool = 1 

    def generate_stable_map(self):
        """ (KEY 9) Επαναφέρει το Terrain/Walls (Seed 999) αλλά καθαρίζει Agents/Targets. """
        random.seed(999) 
        self.game_map = [[random.choice([TILE_TYPE_FREE, TILE_TYPE_FREE, TILE_TYPE_FREE, TILE_TYPE_FREE, 
                                         TILE_TYPE_OBSTACLE, TILE_TYPE_OBSTACLE, 
                                         TILE_TYPE_DIFFICULT, TILE_TYPE_HAZARD]) 
                          for _ in range(GRID_WIDTH_CELLS)] 
                         for _ in range(GRID_HEIGHT_CELLS)]
        random.seed() # Reset seed
        
        # Καθαρισμός περιοχής εκκίνησης (πάνω αριστερά)
        for y in range(5):
            for x in range(5):
                self.game_map[y][x] = TILE_TYPE_FREE
        
        # Καθαρισμός αντικειμένων
        self.custom_agents = []
        self.custom_targets = []
        self.custom_patrols = []
        self.custom_aggressives = []

    def clear_all_map(self):
        """ (KEY 0) Καθαρίζει τα πάντα (Ολικό Reset). """
        self.game_map = [[TILE_TYPE_FREE for _ in range(GRID_WIDTH_CELLS)] for _ in range(GRID_HEIGHT_CELLS)]
        self.custom_agents = []
        self.custom_targets = []
        self.custom_patrols = []
        self.custom_aggressives = []

    def run(self):
        clock = pygame.time.Clock()
        while self.running:
            self.handle_input()
            self.draw()
            pygame.display.flip()
            clock.tick(60)
        
        if self.aborted:
            return None # Επιστροφή στο μενού (Back)
            
        # Επιστρέφουμε τα δεδομένα για να αποθηκευτούν στη main
        return {
            "map": self.game_map,
            "agents": self.custom_agents,
            "targets": self.custom_targets,
            "patrols": self.custom_patrols,
            "aggressives": self.custom_aggressives
        }

    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); sys.exit()
            
            if event.type == pygame.KEYDOWN:
                # ESC -> Επιστροφή στο Menu (χωρίς save)
                if event.key == pygame.K_ESCAPE:
                    self.aborted = True
                    self.running = False
                
                # ENTER -> Έναρξη Simulation
                if event.key == pygame.K_RETURN:
                    if self.custom_agents and self.custom_targets:
                        self.running = False
                
                # --- RESET TOOLS ---
                if event.key == pygame.K_9: # Reset to Seed
                    self.generate_stable_map()
                
                if event.key == pygame.K_0: # Whole Reset (Empty)
                    self.clear_all_map()

                # Επιλογή Εργαλείων
                if event.key == pygame.K_1: self.selected_tool = 1
                if event.key == pygame.K_2: self.selected_tool = 2
                if event.key == pygame.K_3: self.selected_tool = 3
                if event.key == pygame.K_4: self.selected_tool = 4
                if event.key == pygame.K_5: self.selected_tool = 5
                if event.key == pygame.K_6: self.selected_tool = 6
                if event.key == pygame.K_7: self.selected_tool = 7
                if event.key == pygame.K_8: self.selected_tool = 8
                
        if pygame.mouse.get_pressed()[0]:
            mx, my = pygame.mouse.get_pos()
            if my > GAME_HEIGHT - 60: return

            gx, gy = mx // TILE_SIZE, my // TILE_SIZE
            
            if 0 <= gx < GRID_WIDTH_CELLS and 0 <= gy < GRID_HEIGHT_CELLS:
                pos = (gx, gy)
                
                # Αφαίρεση υπάρχοντος στο κελί
                if pos in self.custom_agents: self.custom_agents.remove(pos)
                if pos in self.custom_targets: self.custom_targets.remove(pos)
                if pos in self.custom_patrols: self.custom_patrols.remove(pos)
                if pos in self.custom_aggressives: self.custom_aggressives.remove(pos)
                
                # Εφαρμογή εργαλείου
                if self.selected_tool == 1: 
                    if len(self.custom_agents) < 4: self.custom_agents.append(pos); self.game_map[gy][gx] = TILE_TYPE_FREE
                elif self.selected_tool == 2: self.custom_targets.append(pos); self.game_map[gy][gx] = TILE_TYPE_FREE
                elif self.selected_tool == 3: self.custom_patrols.append(pos); self.game_map[gy][gx] = TILE_TYPE_FREE
                elif self.selected_tool == 4: self.custom_aggressives.append(pos); self.game_map[gy][gx] = TILE_TYPE_FREE
                elif self.selected_tool == 5: self.game_map[gy][gx] = TILE_TYPE_OBSTACLE
                elif self.selected_tool == 6: self.game_map[gy][gx] = TILE_TYPE_DIFFICULT
                elif self.selected_tool == 7: self.game_map[gy][gx] = TILE_TYPE_HAZARD
                elif self.selected_tool == 8: self.game_map[gy][gx] = TILE_TYPE_FREE

    def draw(self):
        self.screen.fill(BLACK)
        
        # 1. Σχεδίαση Χάρτη
        for y in range(GRID_HEIGHT_CELLS):
            for x in range(GRID_WIDTH_CELLS):
                t = self.game_map[y][x]
                c = COLOR_FREE
                if t == TILE_TYPE_OBSTACLE: c = COLOR_OBSTACLE
                elif t == TILE_TYPE_DIFFICULT: c = COLOR_DIFFICULT
                elif t == TILE_TYPE_HAZARD: c = COLOR_HAZARD
                pygame.draw.rect(self.screen, c, (x*TILE_SIZE, y*TILE_SIZE, TILE_SIZE, TILE_SIZE))
                pygame.draw.rect(self.screen, (30,30,30), (x*TILE_SIZE, y*TILE_SIZE, TILE_SIZE, TILE_SIZE), 1)
        
        # 2. Σχεδίαση Αντικειμένων
        for x,y in self.custom_agents: pygame.draw.rect(self.screen, (0,0,255), (x*TILE_SIZE,y*TILE_SIZE,TILE_SIZE,TILE_SIZE), border_radius=4)
        for x,y in self.custom_targets: pygame.draw.circle(self.screen, COLOR_COLLECTIBLE, (x*TILE_SIZE+10,y*TILE_SIZE+10), 6)
        for x,y in self.custom_patrols: pygame.draw.ellipse(self.screen, COLOR_PATROL_OBSTACLE, (x*TILE_SIZE,y*TILE_SIZE,TILE_SIZE,TILE_SIZE))
        for x,y in self.custom_aggressives: pygame.draw.ellipse(self.screen, COLOR_AGGRESSIVE_OBSTACLE, (x*TILE_SIZE,y*TILE_SIZE,TILE_SIZE,TILE_SIZE))
        
        # 3. UI Labels
        ui_y = GAME_HEIGHT - 60
        pygame.draw.rect(self.screen, (20, 20, 30), (0, ui_y, GAME_WIDTH, 60))
        pygame.draw.line(self.screen, WHITE, (0, ui_y), (GAME_WIDTH, ui_y), 2)
        
        tools_labels = [
            ("1:Agent", (0,150,255)), ("2:Target", COLOR_COLLECTIBLE), 
            ("3:Patrol", COLOR_PATROL_OBSTACLE), ("4:Aggr", COLOR_AGGRESSIVE_OBSTACLE), 
            ("5:Wall", (100,100,100)), ("6:Diff", COLOR_DIFFICULT), ("7:Haz", COLOR_HAZARD), 
            ("8:Eraser", WHITE), ("9:Seed Reset", (255,165,0)), ("0:Empty", (255,50,50))
        ]
        
        x_off = 10
        for i, (lbl, col) in enumerate(tools_labels):
            # Styling για τα Reset κουμπιά
            if i >= 8: font = self.font_bold
            elif (i+1) == self.selected_tool: font = self.font_bold; lbl = f"[{lbl}]"; col = MENU_HIGHLIGHT_COLOR
            else: font = self.font
            
            s = font.render(lbl, True, col)
            self.screen.blit(s, (x_off, ui_y + 10))
            x_off += s.get_width() + 15

        status = f"Agents: {len(self.custom_agents)}/4 | Targets: {len(self.custom_targets)} | [ENTER]: Start | [ESC]: Menu"
        self.screen.blit(self.font.render(status, True, (200,200,200)), (20, ui_y + 35))  
    

#  PATHFINDING NODE  
class Node:
    """ Κλάση κόμβου για τον A* αλγόριθμο pathfinding. """
    def __init__(self, position, parent=None):
        self.parent = parent
        self.position = position
        self.g = 0; self.h = 0; self.f = 0
    
    def __eq__(self, other): return self.position == other.position 
    def __lt__(self, other): return self.f < other.f
    def __hash__(self): return hash(self.position)


#  AGENT  
class Agent:
    """ Κλάση για το επίγειο ρομπότ """
    def __init__(self, x, y, agent_id=0):
        self.id = agent_id
        self.name = chr(ord('A') + agent_id) # A, B, C, D
        self.color = AGENT_COLORS[agent_id % len(AGENT_COLORS)]
        self.x, self.y = x, y
        self.path = []
        self.score = 0
        self.wait_timer = 0
        self.current_target = None
        self.active = True
        
        # Stats για γραφήματα
        self.distance_travelled = 0.0
        self.enemies_encountered = 0
        self.targets_collected = 0
        self.encounter_cooldown = 0
        self.last_pos = (x, y)
    
    
    def set_path(self, path):
        """ Ορίζει νέα διαδρομή για το ρομπότ """
        if path and path[0] == (self.x, self.y):
            self.path = path[1:]
        elif path:
            self.path = path
        else:
            self.path = []
    
    def move(self):
        """ Κινεί τον agent και επιστρέφει True αν έγινε κίνηση """
        if not self.active:
            return False
            
        if self.wait_timer > 0: 
            self.wait_timer -= 1
            return False
        
        if self.path and abs(self.path[0][0] - self.x) + abs(self.path[0][1] - self.y) == 1:
            self.last_pos = (self.x, self.y)
            self.x, self.y = self.path.pop(0)
            self.distance_travelled += 1.0
            return True
        else:
            self.path = []
            return False
    
    def draw(self, surface):
        """ Σχεδιάζει τον agent (αν είναι ενεργός). """
        if not self.active:
            return
            
        rect = pygame.Rect(self.x * TILE_SIZE, self.y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
        pygame.draw.rect(surface, self.color, rect, 0, 4)
        
        # Σχεδίαση ID
        font = pygame.font.SysFont("Arial", 12, bold=True)
        lab = font.render(self.name, True, WHITE)
        surface.blit(lab, lab.get_rect(center=rect.center))


# --- OBSTACLES ---
class PatrolObstacle:
    """ Εχθρός περιπολίας που κινείται τυχαία. """
    def __init__(self, x, y, game_map):
        self.x, self.y = x, y
        self.game_map = game_map
        self.harmless = False
        
    def update(self, allowed_tiles):
        moves = []
        if self.harmless: return
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_x, new_y = self.x + dx, self.y + dy
            if (0 <= new_x < GRID_WIDTH_CELLS and 
                0 <= new_y < GRID_HEIGHT_CELLS and 
                self.game_map[new_y][new_x] in allowed_tiles):
                moves.append((dx, dy))
        
        if moves:
            dx, dy = random.choice(moves)
            self.x, self.y = self.x + dx, self.y + dy
    
    def draw(self, surface):
        rect = pygame.Rect(self.x * TILE_SIZE, self.y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
        pygame.draw.ellipse(surface, COLOR_PATROL_OBSTACLE, rect)

class AggressiveObstacle(PatrolObstacle):
    """ 
    Επιθετικός εχθρός.
    Στοχεύει τον πλησιέστερο ενεργό/ζωντανό agent.
    """
    def __init__(self, x, y, game_map, agents):
        super().__init__(x, y, game_map)
        self.agents = agents
        self.detection_radius = 6
        self.mode = 'patrol'
        self.target_agent = None
        self.harmless = False
        
    def update(self, allowed_tiles):
        if self.harmless: return
        if not self.agents:
            super().update(allowed_tiles)
            return

        active_agents = [a for a in self.agents if a.active]
        
        if not active_agents:
            self.mode = 'patrol'
            super().update(allowed_tiles)
            return

        self.target_agent = min(active_agents, key=lambda a: math.hypot(self.x - a.x, self.y - a.y))
        dist = math.hypot(self.x - self.target_agent.x, self.y - self.target_agent.y)
        self.mode = 'attack' if dist <= self.detection_radius else 'patrol'
        
        if self.mode == 'attack':
            game = Game.get_instance()
            if game:
                path_res, _ = game.a_star((self.x, self.y), (self.target_agent.x, self.target_agent.y), 
                                        [TILE_TYPE_OBSTACLE])
                if path_res and len(path_res) > 1:
                    self.x, self.y = path_res[1]
                    return
        
        super().update(allowed_tiles)
    
    def draw(self, surface):
        rect = pygame.Rect(self.x * TILE_SIZE, self.y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
        color = COLOR_AGGRESSIVE_OBSTACLE if self.mode == 'attack' else COLOR_PATROL_OBSTACLE
        pygame.draw.ellipse(surface, color, rect)


def show_mission_report(screen, histories, agents, result):
        """ Εμφανίζει το τελικό γράφημα (Score/Targets vs Time) μετά το τέλος. """
        if not MATPLOTLIB_AVAILABLE: return

        # Ρυθμίσεις παραθύρου
        W, H = screen.get_size()
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("Arial", 20)
        
        # --- ΔΗΜΙΟΥΡΓΙΑ ΓΡΑΦΗΜΑΤΟΣ (ΜΙΑ ΦΟΡΑ) ---
        dpi = 100
        fig = plt.figure(figsize=(W/dpi, (H-100)/dpi), dpi=dpi, facecolor=(0.05, 0.05, 0.1))
        ax1 = fig.add_subplot(111)
        ax2 = ax1.twinx() # Δεύτερος άξονας Υ

        # Στυλ
        plt.rcParams.update({'text.color': 'white', 'axes.labelcolor': 'white', 
                            'xtick.color': 'white', 'ytick.color': 'white', 
                            'axes.edgecolor': 'white', 'axes.facecolor': (0.05, 0.05, 0.1)})
        
        ax1.set_title(f"Mission Report: {result.upper()} - Performance Analysis", color='yellow', fontsize=12)
        ax1.set_xlabel("Time (seconds)")
        ax1.set_ylabel("Targets Collected (Solid Line)", color='cyan')
        ax2.set_ylabel("Score (Dashed Line)", color='orange')
        
        has_data = False
        for i, agent in enumerate(agents):
            data = histories[i]
            times = data["times"]
            scores = data["score"]
            targets = data["targets"]
            
            if times:
                has_data = True
                # Χρώμα agent (0-1)
                c = tuple(v/255.0 for v in agent.color)
                
                # Στόχοι (Solid Line) - Αριστερός Άξονας
                ax1.plot(times, targets, color=c, linestyle='-', linewidth=2, label=f"{agent.name} Targets")
                
                # Σκορ (Dashed Line) - Δεξιός Άξονας
                # Εδώ φαίνεται το κόστος: Αν η γραμμή πέφτει απότομα, περνάει από δύσκολο έδαφος
                ax2.plot(times, scores, color=c, linestyle='--', linewidth=1.5, alpha=0.7, label=f"{agent.name} Score")

        ax1.grid(True, alpha=0.1)
        if has_data:
            # Συνδυασμός Legend
            lines1, labels1 = ax1.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=8, facecolor=(0,0,0,0.5))

        # Render σε εικόνα Pygame
        buf = BytesIO()
        fig.savefig(buf, format="png", facecolor=fig.get_facecolor())
        plt.close(fig)
        buf.seek(0)
        chart_img = pygame.image.load(buf)
        buf.close()

        # --- ΒΡΟΧΟΣ ΕΜΦΑΝΙΣΗΣ ---
        waiting = True
        while waiting:
            screen.fill(MENU_BG_COLOR)
            
            # Τίτλος
            t = font.render(f"MISSION {result.upper()} - Press ENTER/ESC to Return", True, WHITE)
            screen.blit(t, (W//2 - t.get_width()//2, 20))
            
            # Γράφημα
            screen.blit(chart_img, (0, 60))
            
            pygame.display.flip()
            
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    pygame.quit(); sys.exit()
                if e.type == pygame.KEYDOWN:
                    if e.key in [pygame.K_ESCAPE, pygame.K_RETURN]:
                        waiting = False
            clock.tick(30)


#  GAME 
class Game:
    _instance = None
    @staticmethod
    def get_instance():
        return Game._instance
    
    
    def __init__(self, custom_data=None, **kwargs):
        Game._instance = self
        self.mission_settings = kwargs
        self.custom_data = custom_data # <-- Αποθήκευση δεδομένων editor
        
        # Μεταβλητές για να βλέπουμε costmap
        self.show_heatmap = False  
        self.last_decision_log = [] # Λίστα για τα σκορ των υποψήφιων μονοπατιών
        
        # Καθορισμός Game Mode
        self.game_mode = "custom" if custom_data else "ground"
        
        # Αρχικοποίηση παραμέτρων
        if self.custom_data:
            # Αν είναι Custom, παίρνουμε τα νούμερα από τα δεδομένα του Editor
            self.num_agents = len(self.custom_data['agents'])
            self.num_targets = len(self.custom_data['targets'])
            self.num_patrol = len(self.custom_data['patrols'])
            self.num_aggressive = len(self.custom_data['aggressives'])
            self.stable_map = False
        else:
            # Αν είναι τυχαίο, παίρνουμε τα νούμερα από το μενού
            self.num_agents = self.mission_settings.get('num_agents', 1)
            self.num_targets = self.mission_settings.get('num_targets', DEFAULT_TARGETS)
            self.num_patrol = self.mission_settings.get("num_patrol", DEFAULT_PATROL)
            self.num_aggressive = self.mission_settings.get("num_aggressive", DEFAULT_AGGRESSIVE)
            self.stable_map = self.mission_settings.get('stable_map', False)
        
        # Γραφικά
        TOTAL_SCREEN_WIDTH = GAME_WIDTH + PANEL_WIDTH
        TOTAL_SCREEN_HEIGHT = GAME_HEIGHT*2

        self.screen = pygame.display.set_mode((TOTAL_SCREEN_WIDTH, TOTAL_SCREEN_HEIGHT))
        # pygame.display.set_caption(f"SAR - {self.num_agents} Agent(s) Scenario")
        pygame.display.set_caption(f"SAR - Dual View (Game & Algorithm Risk Map)")  #V6.5
        self.clock = pygame.time.Clock()

        self.font_small = pygame.font.SysFont("Arial", 14)
        self.font_medium = pygame.font.SysFont("Arial", 18)
        self.font_large = pygame.font.SysFont("Arial", 22, bold=True)
        
        # Επιφάνειες        
        self.game_surface = self.screen.subsurface((0, 0, GAME_WIDTH, GAME_HEIGHT))
        self.stats_surface = self.screen.subsurface((GAME_WIDTH, 0, PANEL_WIDTH, GAME_HEIGHT))
        self.chart_height = int(TOTAL_SCREEN_HEIGHT * 0.80)
        self.log_height = TOTAL_SCREEN_HEIGHT - self.chart_height
   
        # 1. Πάνω Αριστερά: Παιχνίδι
        self.game_surface = self.screen.subsurface((0, 0, GAME_WIDTH, GAME_HEIGHT))
        
        # 2. Κάτω Αριστερά: Heatmap
        self.heatmap_surface = self.screen.subsurface((0, GAME_HEIGHT, GAME_WIDTH, GAME_HEIGHT))

        # 3. Πάνω Δεξιά: Charts (Stats)
        self.stats_surface = self.screen.subsurface((GAME_WIDTH, 0, PANEL_WIDTH, self.chart_height))

        # 4. Κάτω Δεξιά: Agent Logs (Columns)
        self.log_surface = self.screen.subsurface((GAME_WIDTH, self.chart_height, PANEL_WIDTH, self.log_height))
        
        # Cache για τα γραφήματα με το νέο ύψος
        self.chart_surface_cache = pygame.Surface((PANEL_WIDTH, self.chart_height))
        self.chart_surface_cache.fill(COLOR_PANEL_BG)
    
        # Αρχικοποίηση μεταβλητών (χωρίς να γεμίσουμε λίστες ακόμα)
        self.game_map = []
        self.agents = []
        self.dynamic_obstacles = []
        self.collectibles = []
        self.ignored_collectibles = []
        self.total_collectibles = 0
        
        self.agent_states = {}
        self.target_cooldown = {}
        self.target_attempts = {}
        self.start_ticks = 0
        self.pause_offset = 0
        self.paused_at = None
        self.paused = False
        self.finished_flag = False

        # Στατιστικά
        self.last_stats_redraw = 0.0
        self.chart_surface_cache = pygame.Surface((PANEL_WIDTH, TOTAL_SCREEN_HEIGHT))
        self.chart_surface_cache.fill(COLOR_PANEL_BG)
        self.per_agent_histories = {} 
        self.mission_record = None
        
        # --- ΝΕΕΣ ΜΕΤΑΒΛΗΤΕΣ ---
        self.panel_mode = 0  # 0 = Charts, 1 = Tactical View
        self.tactical_surface = pygame.Surface((PANEL_WIDTH, TOTAL_SCREEN_HEIGHT))
        self.last_decision_log = []

        # *** ΤΕΛΙΚΗ ΚΛΗΣΗ *** # Αυτό φορτώνει τον χάρτη και τους agents (Custom ή Random)
        self.setup_new_mission()
    
    def is_reachable(self, start, target):
        """ Ελέγχει αν υπάρχει μονοπάτι από το start στο target (Flood Fill / BFS). """
        queue = collections.deque([start])
        visited = set([start])
        
        while queue:
            curr = queue.popleft()
            if curr == target: return True
            
            for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
                nx, ny = curr[0]+dx, curr[1]+dy
                
                if 0<=nx<GRID_WIDTH_CELLS and 0<=ny<GRID_HEIGHT_CELLS:
                    if (nx,ny) not in visited and self.game_map[ny][nx] != TILE_TYPE_OBSTACLE:
                        visited.add((nx,ny))
                        queue.append((nx,ny))
        return False

    def filter_unreachable_items(self):
        """ Αφαιρεί δυναμικά στόχους που δεν μπορεί να φτάσει κανένα ρομπότ. """
        if not self.agents: return

        # 1. Βρίσκουμε ΟΛΑ τα κελιά που είναι προσβάσιμα από ΟΠΟΙΟΔΗΠΟΤΕ ενεργό ρομπότ
        reachable_cells = set()
        
        for agent in self.agents:
            if not agent.active: continue
            
            # Flood Fill (BFS) από τη θέση του κάθε ρομπότ
            queue = collections.deque([(agent.x, agent.y)])
            visited = set([(agent.x, agent.y)])
            reachable_cells.add((agent.x, agent.y))
            
            while queue:
                cx, cy = queue.popleft()
                
                for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < GRID_WIDTH_CELLS and 0 <= ny < GRID_HEIGHT_CELLS:
                        # Αν δεν είναι εμπόδιο και δεν το έχουμε δει
                        if (nx, ny) not in visited and self.game_map[ny][nx] != TILE_TYPE_OBSTACLE:
                            visited.add((nx, ny))
                            reachable_cells.add((nx, ny))
                            queue.append((nx, ny))

        # 2. Κρατάμε μόνο τους στόχους που είναι μέσα στα reachable_cells
        valid_targets = []
        for t in self.collectibles:
            if t in reachable_cells:
                valid_targets.append(t)
            else:
                # Αν θες να βλέπεις στο log πότε διαγράφεται στόχος:
                # print(f"Removing unreachable target at: {t}")
                pass
        
        # Αν αλλάξει το πλήθος, ενημερώνουμε τη λίστα
        if len(valid_targets) < len(self.collectibles):
            self.collectibles = valid_targets
            # Ενημερώνουμε και το total για να μην χαλάσει το ποσοστό επιτυχίας
            self.total_collectibles = len(self.collectibles) + sum(a.targets_collected for a in self.agents)         
                
    def draw_algorithm_view(self):
        """ Ζωγραφίζει το Heatmap (κάτω αριστερά) και τα Agent Columns (κάτω δεξιά). """
        
        # --- Α. HEATMAP (Αριστερά - Κάτω από το Game) ---
        # Χρησιμοποιούμε το heatmap_surface
        self.heatmap_surface.fill((30, 30, 30))
        cmap = self.create_dynamic_cost_map()

        for y in range(GRID_HEIGHT_CELLS):
            for x in range(GRID_WIDTH_CELLS):
                cost = cmap[y][x]
                tile_type = self.game_map[y][x]
                rect = (x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
                
                # --- ΕΠΙΛΟΓΗ ΧΡΩΜΑΤΟΣ ---
                if tile_type == TILE_TYPE_OBSTACLE:
                    color = (0, 0, 0)
                    pygame.draw.rect(self.heatmap_surface, color, rect)
                    pygame.draw.rect(self.heatmap_surface, (40, 40, 40), rect, 1)
                    continue 

                elif (x, y) in self.collectibles:
                    color = (0, 255, 0) # Πράσινο

                elif cost > 200: 
                    ratio = min(1.0, cost / 5000.0)
                    r = 255
                    g = int(255 * (1 - ratio))
                    b = 0
                    color = (r, g, b)

                elif tile_type == TILE_TYPE_HAZARD:
                    color = (0, 0, 180)
                elif tile_type == TILE_TYPE_DIFFICULT:
                    color = (100, 100, 220)
                else:
                    color = (170, 170, 170)

                pygame.draw.rect(self.heatmap_surface, color, rect)
                pygame.draw.rect(self.heatmap_surface, (100, 100, 100), rect, 1)

        # Agents & Targets στο Heatmap
        for agent in self.agents:
            if agent.active:
                cx, cy = agent.x * TILE_SIZE + TILE_SIZE//2, agent.y * TILE_SIZE + TILE_SIZE//2
                pygame.draw.circle(self.heatmap_surface, agent.color, (cx, cy), TILE_SIZE//2 - 2)
                
                if agent.current_target:
                    tx, ty = agent.current_target
                    tx_px, ty_px = tx * TILE_SIZE + TILE_SIZE//2, ty * TILE_SIZE + TILE_SIZE//2
                    pygame.draw.line(self.heatmap_surface, agent.color, (cx, cy), (tx_px, ty_px), 2)

        # --- Β. AGENT STATUS COLUMNS (Δεξιά - Δίπλα δίπλα) ---
        # Χρησιμοποιούμε το log_surface
        self.log_surface.fill((25, 25, 35))
        
        # Borders
        pygame.draw.line(self.log_surface, WHITE, (0, 0), (0, self.log_height), 2)
        pygame.draw.line(self.log_surface, WHITE, (0, 0), (PANEL_WIDTH, 0), 2)

        num_agents = len(self.agents)
        if num_agents > 0:
            col_width = PANEL_WIDTH // num_agents
            
            for i, agent in enumerate(self.agents):
                # Υπολογισμός θέσης στήλης
                cx = i * col_width
                cy = 10 
                
                # 1. Header (Χρώμα Agent + Όνομα)
                header_rect = pygame.Rect(cx + 4, cy, col_width - 8, 28)
                pygame.draw.rect(self.log_surface, agent.color, header_rect, border_radius=5)
                
                name_txt = self.font_large.render(f"AGENT {agent.name}", True, (0,0,0))
                text_rect = name_txt.get_rect(center=header_rect.center)
                self.log_surface.blit(name_txt, text_rect)
                
                # 2. Πληροφορίες
                info = self.agent_states.get(agent.name, {})
                y_cursor = cy + 40
                
                def draw_field(label, value, y, val_color=WHITE):
                    l_s = self.font_small.render(label, True, (150, 150, 150))
                    v_s = self.font_medium.render(str(value), True, val_color)
                    self.log_surface.blit(l_s, (cx + 8, y))
                    self.log_surface.blit(v_s, (cx + 10, y + 14))

                draw_field("STATUS:", info.get("status", "Idle"), y_cursor)
                draw_field("TARGET:", str(info.get("target", "-")), y_cursor + 35)
                
                cost_val = info.get("cost", 0)
                c_color = (100, 255, 100)
                if isinstance(cost_val, int) and cost_val > 500: c_color = (255, 100, 100)
                draw_field("PATH COST:", cost_val, y_cursor + 70, c_color)

                # Διαχωριστική γραμμή
                if i < num_agents - 1:
                    line_x = cx + col_width
                    pygame.draw.line(self.log_surface, (60, 60, 80), (line_x, 10), (line_x, self.log_height - 10), 1)
    def setup_new_mission(self):
        """ Φορτώνει την αποστολή. Αν υπάρχει custom_data το χρησιμοποιεί, αλλιώς τυχαίο. """
        
        # 1. ΔΗΜΙΟΥΡΓΙΑ ΧΑΡΤΗ & ΑΝΤΙΚΕΙΜΕΝΩΝ
        if self.custom_data:
            # --- CUSTOM MODE ---
            # Αντιγράφουμε τον χάρτη για να μην χαλάσουμε το πρωτότυπο (deep copy)
            self.game_map = [row[:] for row in self.custom_data['map']]
            
            # Agents
            self.agents = []
            for i, pos in enumerate(self.custom_data['agents']):
                self.agents.append(Agent(pos[0], pos[1], agent_id=i))
            
            # Enemies
            self.dynamic_obstacles = []
            for pos in self.custom_data['patrols']:
                self.dynamic_obstacles.append(PatrolObstacle(pos[0], pos[1], self.game_map))
            for pos in self.custom_data['aggressives']:
                self.dynamic_obstacles.append(AggressiveObstacle(pos[0], pos[1], self.game_map, self.agents))
            
            # Targets (Αντιγραφή λίστας)
            self.collectibles = list(self.custom_data['targets'])
            
        else:
            # --- RANDOM MODE ---
            if self.stable_map:
                self.create_stable_map()
            else:
                self.create_random_map()

            # Agents Spawn
            self.agents = []
            spawn = [(1,1),(2,1),(1,2),(2,2)]
            for i in range(self.num_agents):
                ax, ay = spawn[i] if i < len(spawn) else (1+i, 1)
                self.agents.append(Agent(ax, ay, agent_id=i))
                self.game_map[ay][ax] = TILE_TYPE_FREE # Καθαρισμός θέσης

            # Enemies Spawn
            self.dynamic_obstacles = []
            for _ in range(self.num_patrol):
                self.add_obstacle(PatrolObstacle)
            for _ in range(self.num_aggressive):
                self.add_obstacle(AggressiveObstacle, agents_list=self.agents)

            # Targets Spawn
            self.collectibles = []
            tries = 0
            while len(self.collectibles) < self.num_targets and tries < 20000:
                tries += 1
                x = random.randint(1, GRID_WIDTH_CELLS - 2)
                y = random.randint(1, GRID_HEIGHT_CELLS - 2)
                if self.game_map[y][x] == TILE_TYPE_FREE and \
                   not any((x,y) == (o.x,o.y) for o in self.dynamic_obstacles):
                    if (x,y) not in self.collectibles:
                        self.collectibles.append((x,y))
            self.filter_unreachable_items()
            self.total_collectibles = len(self.collectibles) if self.collectibles else 1
            
        # 2. ΚΟΙΝΕΣ ΡΥΘΜΙΣΕΙΣ (ΓΙΑ ΟΛΑ ΤΑ MODES)
        self.total_collectibles = len(self.collectibles) if self.collectibles else 1
        self.target_cooldown = {}
        self.target_attempts = {}
        
        # Reset Agents State
        for a in self.agents:
            a.path = []
            a.current_target = None
            a.score = 0
            a.active = True
            a.distance_travelled = 0.0
            a.enemies_encountered = 0
            a.targets_collected = 0
            a.last_pos = (a.x, a.y)

        # Reset Time
        self.start_ticks = pygame.time.get_ticks()
        self.pause_offset = 0
        self.paused_at = None
        self.paused = False
        self.finished_flag = False
        
        # Reset Stats (Με "score" για να μην κρασάρει το γράφημα)
        # self.per_agent_histories = {i: {"times": [], "speed": [], "distance": [], "targets": [], "score": []} for i in range(len(self.agents))}
        self.per_agent_histories = {
            i: {"times": [], "speed": [], "distance": [], "targets": [], "score": [], "encounters": []} 
            for i in range(len(self.agents))
        }
        
        # Reset Stats
        self.per_agent_histories = {i: {"times": [], "speed": [], "distance": [], "targets": [], "score": [], "encounters": []} for i in range(len(self.agents))}
        
        # --- RESET STATUS ---
        self.agent_states = {} 
        for a in self.agents:
            self.agent_states[a.name] = {"status": "Idle", "target": "None", "cost": 0, "risk": "Low"}
        
        # Reset Charts
        self.last_stats_redraw = 0.0
        self.chart_surface_cache.fill(COLOR_PANEL_BG)

        # Mission Record Init
        self.mission_record = {
            "agents_count": len(self.agents), "num_targets": self.total_collectibles,
            "start_time": int(time.time()), "duration": None,
            "result": None, "per_agent": {}
        }
        
        # Αρχικό Pathfinding
        self.find_paths_for_all()   
        
        
        
    def create_random_map(self):
        self.game_map = [[random.choice([0,0,0,0,1,1,2,3]) 
                          for _ in range(GRID_WIDTH_CELLS)] 
                         for _ in range(GRID_HEIGHT_CELLS)]
        for y in range(4):
            for x in range(4):
                self.game_map[y][x] = TILE_TYPE_FREE

    def add_obstacle(self, cls, agents_list=None):
        for _ in range(2000):
            x = random.randint(1, GRID_WIDTH_CELLS - 2)
            y = random.randint(1, GRID_HEIGHT_CELLS - 2)
            if self.game_map[y][x] == TILE_TYPE_FREE and \
               not any(math.hypot(x - o.x, y - o.y) <= 2 for o in self.dynamic_obstacles) and \
               not any(math.hypot(x - a.x, y - a.y) <= 8 for a in self.agents):
                
                if cls == AggressiveObstacle:
                    o = cls(x, y, self.game_map, agents_list)
                else:
                    o = cls(x, y, self.game_map)
                self.dynamic_obstacles.append(o)
                return

    #  Χειρισμός Χρόνου 
    def get_elapsed_seconds(self):
        # Αν έχουμε τελειώσει, επιστρέφουμε τον παγωμένο χρόνο
        if self.finished_flag and self.mission_record.get("duration") is not None:
            return float(self.mission_record["duration"])
            
        now = pygame.time.get_ticks()
        total_paused = self.pause_offset
        if self.paused_at:
            total_paused = self.pause_offset + (now - self.paused_at)
        elapsed_ms = now - self.start_ticks - total_paused
        return elapsed_ms / 1000.0

    def toggle_pause(self):
        if self.finished_flag: return
        if not self.paused:
            self.paused_at = pygame.time.get_ticks()
            self.paused = True
        else:
            if self.paused_at:
                self.pause_offset += (pygame.time.get_ticks() - self.paused_at)
            self.paused_at = None
            self.paused = False


    def run(self):
        ensure_stats_files_exist()
        self.find_paths_for_all()
        
        running = True
        while running:
            # 1. Events
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    pygame.quit(); sys.exit(0)
                
                if ev.type == pygame.KEYDOWN:
                    # --- GLOBAL KEYS (Λειτουργούν ΠΑΝΤΑ: και στο παιχνίδι και στο τέλος) ---
                    
                    # R: Restart - Ξαναρχίζει την αποστολή αμέσως
                    if ev.key == pygame.K_r:
                        self.setup_new_mission()
                        # Δεν κάνουμε running=False, συνεχίζουμε το loop με τα νέα δεδομένα
                    
                    # ESC: Επιστροφή στο Menu
                    elif ev.key == pygame.K_ESCAPE:
                        running = False

                    # --- FINISHED STATE (Μόνο όταν τελειώσει) ---
                    elif self.finished_flag:
                        # ENTER: Εμφάνιση αναφοράς (Report)
                        if ev.key == pygame.K_RETURN:
                            res = self.mission_record.get("result", "Finished")
                            show_mission_report(self.screen, self.per_agent_histories, self.agents, res)
                            running = False # Μετά το report, βγες στο μενού

                    # --- PLAYING STATE (Μόνο όταν παίζει) ---
                    else:
                        # SPACE: Pause
                        if ev.key == pygame.K_SPACE:
                            self.toggle_pause()
            
            # 2. Update (Αν είναι finished, η update επιστρέφει αμέσως από μόνη της)
            if not self.paused:
                self.update()
                
            # 3. Draw (Ζωγραφίζει πάντα, ακόμα και παγωμένο)
            self.draw()
            self.clock.tick(30)
            
            # 4. Charts Update
            current_time = time.time()
            if MATPLOTLIB_AVAILABLE and PANDAS_AVAILABLE and \
               (current_time - self.last_stats_redraw > STATS_REDRAW_INTERVAL):
                if not self.paused: 
                    self.redraw_charts()
                self.last_stats_redraw = current_time
        
        return True

    def handle_events(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); sys.exit(0)
                
            if ev.type == pygame.KEYDOWN:
                # --- GLOBAL KEYS ---
                
                # ESC: Έξοδος στο μενού
                if ev.key == pygame.K_ESCAPE:
                    return False 
                
                # R: Επανεκκίνηση (Restart)
                if ev.key == pygame.K_r:
                    self.setup_new_mission()
                    return True
                
                # Η: Δείχνει το costmap
                if ev.key == pygame.K_h:
                        self.show_heatmap = not self.show_heatmap
                        print(f"Heatmap: {self.show_heatmap}")
                                        
                # --- FINISHED STATE KEYS ---
                if self.finished_flag:
                    # ENTER: Προβολή αναφοράς (Report)
                    if ev.key == pygame.K_RETURN:
                        res = self.mission_record.get("result", "Finished")
                        show_mission_report(self.screen, self.per_agent_histories, self.agents, res)
                        return False # Μετά το report, επιστροφή στο μενού
                
                # --- PLAYING STATE KEYS ---
                else:
                    # SPACE: Παύση
                    if ev.key == pygame.K_SPACE:
                        self.toggle_pause()
                        
        return True

    def update(self):
        """ Ενημερώνει τη λογική του παιχνιδιού """
        if self.finished_flag:
            return
        
        t = self.get_elapsed_seconds()
        
        # 1. Ενημέρωση εχθρών
        for obs in self.dynamic_obstacles:
            obs.update([TILE_TYPE_FREE, TILE_TYPE_DIFFICULT, TILE_TYPE_HAZARD])

        # 2. Βρόχος για κάθε ρομπότ
        for a_idx, agent in enumerate(self.agents):
            if not agent.active:
                continue
            
            pos = (agent.x, agent.y)
            
            # Έλεγχος Cooldown στόχου
            if agent.current_target and self.target_attempts.get(agent.current_target, 0) > 10:
                self.target_cooldown[agent.current_target] = 25
                agent.current_target = None
                agent.path = []
            
            # Έλεγχος σύγκρουσης
            if any((obs.x, obs.y) == pos for obs in self.dynamic_obstacles):
                agent.active = False
                agent.path = []
                agent.current_target = None
                continue
            
            # Έλεγχος Απειλών
            threats = []
            for o in self.dynamic_obstacles:
                if o.harmless: continue
                turns_to_reach = self.turns_to_reach((o.x, o.y), pos)
                if isinstance(o, AggressiveObstacle):
                    max_turns = o.detection_radius + 2
                    if o.mode == 'attack': max_turns += 1
                else: max_turns = 3
                if turns_to_reach <= max_turns:
                    threats.append(o)
                    
            if agent.encounter_cooldown > 0:
                agent.encounter_cooldown -= 1
            else:
                # Έλεγχος αν υπάρχει εχθρός σε απόσταση < 4 κελιών
                # Χρησιμοποιούμε το math.hypot για ευκλείδεια απόσταση
                for obs in self.dynamic_obstacles:
                    dist = math.hypot(obs.x - agent.x, obs.y - agent.y)
                    if dist < 4.0: # Αν είναι κοντά
                        agent.enemies_encountered += 1
                        agent.encounter_cooldown = 60 # Περίμενε 2-3 δευτερόλεπτα (60 frames) πριν ξαναμετρήσεις
                        break # Μέτρα το μία φορά ανά frame
                    
            # Λογική Αποφυγής
            if threats:
                brave_path = self.find_brave_path(agent, threats)
                if brave_path:
                    agent.current_target = brave_path[-1]
                    agent.set_path(brave_path)
                else:
                    escape_path = self.find_escape_path(agent, threats, self.dynamic_obstacles)
                    if escape_path:
                        agent.current_target = None
                        agent.set_path(escape_path)
                    else:
                        best_move, max_dist = None, -1
                        closest_threat = min(threats, key=lambda t: math.hypot(pos[0] - t.x, pos[1] - t.y))
                        for dx, dy in [(x, y) for x in [-1, 0, 1] for y in [-1, 0, 1] if not (x == 0 and y == 0)]:
                            nx, ny = pos[0] + dx, pos[1] + dy
                            if (0 <= nx < GRID_WIDTH_CELLS and 0 <= ny < GRID_HEIGHT_CELLS and 
                                self.game_map[ny][nx] not in [1, 4] and 
                                not any((nx, ny) == (o.x, o.y) for o in self.dynamic_obstacles)):
                                dist = math.hypot(nx - closest_threat.x, ny - closest_threat.y)
                                if dist > max_dist:
                                    max_dist = dist; best_move = (nx, ny)
                        if best_move: agent.set_path([best_move])
                        else: agent.path = []; agent.wait_timer = 1
            
            # Εύρεση νέου στόχου
            elif not agent.path or agent.current_target not in self.collectibles:
                assigned_targets = {a.current_target for a in self.agents if a.current_target and a.active and a != agent}
                available_collectibles = [c for c in self.collectibles if c not in assigned_targets and c not in self.target_cooldown]
                if not available_collectibles:
                    available_collectibles = [c for c in self.collectibles if c not in assigned_targets]
                
                if available_collectibles:
                    self.find_path_to_collectible(agent, available_collectibles, [TILE_TYPE_OBSTACLE])
                else:
                    # Αν δεν υπάρχουν διαθέσιμοι (π.χ. 2 agents, 1 στόχος), πάνε και οι δύο
                    self.find_path_to_collectible(agent, self.collectibles, [TILE_TYPE_OBSTACLE])
            
            # Κίνηση & Κόστος
            moved = agent.move()
            # if moved:
            #     tile_type = self.game_map[agent.y][agent.x]
            #     cost = TILE_COSTS.get(tile_type, 0)
            #     if cost > 0:
            #         agent.score = max(0, agent.score - cost)
            # moved = agent.move()
            if moved:
                # Βρες τον τύπο του κελιού που πάτησε ΤΩΡΑ (αφού κινήθηκε)
                tile_type = self.game_map[agent.y][agent.x]
                
                # Πάρε το κόστος από το λεξικό
                cost = TILE_COSTS.get(tile_type, 1) # Default 1 αν δεν βρεθεί
                
                # Αφαίρεσε το κόστος (επιτρέπουμε αρνητικά νούμερα)
                agent.score -= cost

            active_agents = [a for a in self.agents if a.active]
            if self.collectibles and all(not a.path and not a.current_target for a in active_agents):
                # Τρέχουμε ξανά το φίλτρο για να καθαρίσουμε τυχόν "φαντάσματα"
                self.filter_unreachable_items()
            
            # Έλεγχος Συλλογής
            if agent.active and (agent.x, agent.y) in self.collectibles:
                collected_pos = (agent.x, agent.y)
                self.collectibles.remove(collected_pos)
                agent.score += BASE_TARGET_SCORE
                agent.targets_collected += 1
                
                if collected_pos in self.target_attempts:
                    del self.target_attempts[collected_pos]
                
                agent.current_target = None
                agent.path = []
                
                for other_agent in self.agents:
                    if other_agent.current_target == collected_pos:
                        other_agent.current_target = None
                        other_agent.path = []

        # Ενημέρωση Στατιστικών Γραφήματος
            curr_speed = 1.0 if moved else 0.0
            hist = self.per_agent_histories[a_idx]
            hist["times"].append(t)
            hist["speed"].append(curr_speed)
            hist["distance"].append(agent.distance_travelled)
            hist["targets"].append(agent.targets_collected)
            # για τελικό διαγραμμα σκορ/χρόνου/στόχων
            hist["score"].append(agent.score)
            hist["encounters"].append(agent.enemies_encountered)
            
        # 3. Έλεγχος Τέλους
        if not self.collectibles:
            self.finish_mission("success")
            return
        if all(not a.active for a in self.agents):
            collected = sum(a.targets_collected for a in self.agents)
            frac = (collected / self.total_collectibles) if self.total_collectibles else 0
            res = "partial" if frac > 0.7 else "failed"
            self.finish_mission(res)
            return

    #  Λογική Pathfinding
    
    def is_dead_end(self, x, y):
        if (x, y) in self.collectibles: 
            return False
        free_neighbors = 0
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if (0 <= nx < GRID_WIDTH_CELLS and 
                0 <= ny < GRID_HEIGHT_CELLS and 
                self.game_map[ny][nx] != TILE_TYPE_OBSTACLE):
                free_neighbors += 1
        return free_neighbors <= 1

    def find_paths_for_all(self):
        """ Αρχική εύρεση διαδρομών """
        
        if hasattr(self, 'agents') and self.game_mode == "ground":
            shuffled_targets = self.collectibles[:]
            random.shuffle(shuffled_targets)
            assigned_targets = set()
            for agent in self.agents:
                available_targets = [t for t in shuffled_targets if t not in assigned_targets]
                if not available_targets:
                     available_targets = self.collectibles
                target_found = self.find_path_to_collectible(agent, available_targets, [TILE_TYPE_OBSTACLE])
                if target_found:
                    assigned_targets.add(agent.current_target)
    
    def find_path_to_collectible(self, agent, targets, restricted):
        if not targets: 
            agent.path = []
            agent.current_target = None
            return False
        
        pos = (agent.x, agent.y)
        
        # Δημιουργία του χάρτη κόστους (για χρήση και στο Heatmap)
        cost_map = self.create_dynamic_cost_map() 
        
        # Ταξινόμηση με βάση απόσταση (Manhattan) για γρήγορο φιλτράρισμα
        targets.sort(key=lambda t: abs(t[0]-pos[0]) + abs(t[1]-pos[1]))
        
        best_path, best_cost, best_target = None, float('inf'), None
        
        # Λίστα για να αποθηκεύσουμε τα αποτελέσματα της αξιολόγησης
        candidate_scores = [] 

        # Ελέγχουμε τους 5 κοντινότερους
        for t in [t for t in targets if t not in self.target_cooldown]:            
            # Τρέχουμε A* για να πάρουμε το πραγματικό κόστος (Path Score)
            path, cost = self.a_star(pos, t, restricted, cost_map)
            
            # Αποθήκευση για εμφάνιση (Στόχος -> Κόστος)
            candidate_scores.append(f"T({t[0]},{t[1]}): {cost}")

            if path and cost < best_cost:
                best_path, best_cost, best_target = path, cost, t
        
        # Ενημέρωση του Log για την οθόνη
        if candidate_scores:
            self.last_decision_log = [f"Agent {agent.name} Decision:"] + candidate_scores[:5]

        if best_path:
            agent.current_target = best_target
            agent.set_path(best_path)
            self.target_attempts[best_target] = self.target_attempts.get(best_target, 0) + 1
            # --- ΕΝΗΜΕΡΩΣΗ STATUS (ΓΙΑ ΤΟ ΚΑΤΩ ΠΑΝΕΛ) ---
            self.agent_states[agent.name] = {
                "status": "Moving",
                "target": str(best_target),
                "cost": int(best_cost),
                "risk": "High" if best_cost > 500 else "Low" # Απλό κριτήριο ρίσκου
            }
            return True
        else:
            # Αν δεν βρήκε
            self.agent_states[agent.name] = {
                "status": "Scanning...",
                "target": "None",
                "cost": "-",
                "risk": "-"
            }
            agent.path = []; agent.current_target = None   
            return False

    def find_escape_path(self, agent, threats, dynamic_obstacles):
        pos = (agent.x, agent.y)
        cost_map = self.create_dynamic_cost_map(escape=True)
        best_path, best_score = None, -float('inf')
        candidates = []
        for r in range(2, 8):
            for i in range(-r, r + 1):
                for j in range(-r, r + 1):
                    if (abs(i) + abs(j) == r and 
                        0 <= pos[0] + i < GRID_WIDTH_CELLS and 
                        0 <= pos[1] + j < GRID_HEIGHT_CELLS and 
                        self.game_map[pos[1] + j][pos[0] + i] not in [1, 4]):
                        candidates.append((pos[0] + i, pos[1] + j))
        for spot in candidates:
            if any(spot == (o.x, o.y) for o in dynamic_obstacles): continue
            path, cost = self.a_star(pos, spot, [TILE_TYPE_OBSTACLE], cost_map)
            if path and not any(s == (o.x, o.y) for s in path for o in dynamic_obstacles):
                score = min(self.turns_to_reach((t.x, t.y), spot) for t in threats) * 1.5 - len(path)
                if score > best_score:
                    best_path, best_score = path, score
        return best_path

    def find_brave_path(self, agent, threats):
        agent_pos = (agent.x, agent.y)
        aggressive_threats = [t for t in threats if isinstance(t, AggressiveObstacle)]
        if not aggressive_threats: return None
        sorted_collectibles = sorted(self.collectibles, 
                                   key=lambda c: abs(c[0] - agent_pos[0]) + abs(c[1] - agent_pos[1]))
        for target in sorted_collectibles:
            if self.is_dead_end(target[0], target[1]): continue
            agent_steps = self.turns_to_reach(agent_pos, target)
            if agent_steps == float('inf'): continue
            enemy_steps = min(self.turns_to_reach((t.x, t.y), target) for t in aggressive_threats)
            if agent_steps < enemy_steps - 1:
                cost_map = self.create_dynamic_cost_map(escape=True)
                path, cost = self.a_star(agent_pos, target, [TILE_TYPE_OBSTACLE], cost_map)
                if path: return path
        return None

    def create_dynamic_cost_map(self, escape=False):
        cost_map = [[1 + TILE_COSTS.get(self.game_map[y][x], 0) * 5 
                     for x in range(GRID_WIDTH_CELLS)] 
                    for y in range(GRID_HEIGHT_CELLS)]
        
        for y in range(GRID_HEIGHT_CELLS):
            for x in range(GRID_WIDTH_CELLS):
                if self.is_dead_end(x, y):
                    penalty = 4000 if escape else 200
                    cost_map[y][x] += penalty

        for obs in getattr(self, 'dynamic_obstacles', []):
            r = 5
            if isinstance(obs, AggressiveObstacle):
                r = obs.detection_radius
            
            ext_r = r + 2 if not escape else r + 4
            for dy in range(-ext_r, ext_r + 1):
                for dx in range(-ext_r, ext_r + 1):
                    nx, ny = obs.x + dx, obs.y + dy
                    if 0 <= nx < GRID_WIDTH_CELLS and 0 <= ny < GRID_HEIGHT_CELLS:
                        dist = abs(dx) + abs(dy)
                        penalty = 0
                        if dist == 0: penalty = 8000
                        elif dist <= 1: penalty = 3000
                        elif dist <= 2: penalty = 1500
                        elif dist <= r: penalty = 800
                        elif dist <= ext_r: penalty = 300
                        
                        if isinstance(obs, AggressiveObstacle) and obs.mode == 'attack':
                            penalty *= 1.5
                            
                        cost_map[ny][nx] += int(penalty)
        return cost_map

    def draw_heatmap(self, surf):
        """ Ζωγραφίζει τον χάρτη κόστους (Risk Map). """
        # Παίρνουμε έναν φρέσκο χάρτη κόστους
        cmap = self.create_dynamic_cost_map()
        
        # Βρίσκουμε το μέγιστο κόστος για normalization
        max_val = 1
        for row in cmap:
            max_val = max(max_val, max(row))
            
        s = pygame.Surface((TILE_SIZE, TILE_SIZE))
        s.set_alpha(128) # 50% διαφάνεια
        
        for y in range(GRID_HEIGHT_CELLS):
            for x in range(GRID_WIDTH_CELLS):
                val = cmap[y][x]
                if val > 1: # Αν έχει κόστος παραπάνω από το απλό έδαφος
                    # Υπολογισμός χρώματος (Κόκκινο = Υψηλό ρίσκο, Πράσινο/Κίτρινο = Μέτριο)
                    ratio = min(1.0, val / 5000.0) # Cap στο 5000 (ποινή εχθρού)
                    
                    r = int(255 * ratio)
                    g = int(255 * (1 - ratio))
                    b = 0
                    
                    s.fill((r, g, b))
                    surf.blit(s, (x*TILE_SIZE, y*TILE_SIZE))
                    
                    # Προαιρετικά: Εμφάνιση νούμερου κόστους (αν χωράει)
                    # if val > 100:
                    #     txt = self.font_small.render(str(val), True, BLACK)
                    #     surf.blit(txt, (x*TILE_SIZE, y*TILE_SIZE))

    def turns_to_reach(self, start, end):
        path, cost = self.a_star(start, end, [1, 4])
        return cost if path else float('inf')

    def a_star(self, start, end, restricted, cost_map=None):
        if not (0 <= start[0] < GRID_WIDTH_CELLS and 0 <= start[1] < GRID_HEIGHT_CELLS):
            return None, float('inf')
        if cost_map is None:
            cost_map = [[1] * GRID_WIDTH_CELLS for _ in range(GRID_HEIGHT_CELLS)]
        nodes = [(0, Node(start))]
        closed = set()
        g_costs = {start: 0}
        
        while nodes:
            _, c_node = heapq.heappop(nodes)
            if c_node.position == end:
                path = []
                p = c_node
                while p: path.append(p.position); p = p.parent
                return path[::-1], g_costs.get(end, float('inf'))
            if c_node.position in closed:
                continue
            closed.add(c_node.position)
            
            cardinal_moves = [(0, -1), (0, 1), (-1, 0), (1, 0)]
            for move in cardinal_moves:
                pos = (c_node.position[0] + move[0], c_node.position[1] + move[1])
                if (not (0 <= pos[0] < GRID_WIDTH_CELLS and 0 <= pos[1] < GRID_HEIGHT_CELLS) or
                    self.game_map[pos[1]][pos[0]] in restricted or
                    pos in closed):
                    continue
                new_g = g_costs.get(c_node.position, float('inf')) + cost_map[pos[1]][pos[0]]
                if new_g < g_costs.get(pos, float('inf')):
                    g_costs[pos] = new_g
                    h = abs(pos[0] - end[0]) + abs(pos[1] - end[1])
                    heapq.heappush(nodes, (new_g + h, Node(pos, c_node)))
        return None, float('inf')
    
    def finish_mission(self, result):
        """ Τερματίζει την αποστολή και αποθηκεύει τα στατιστικά. """
        if self.finished_flag: return
        
        elapsed = int(self.get_elapsed_seconds())
        self.mission_record["duration"] = elapsed
        self.mission_record["result"] = result
        self.mission_record["end_time"] = int(time.time())
        
        per_agent_data = {}
        for i, a in enumerate(self.agents):
            per_agent_data[a.name] = {
                "score": a.score, "targets": a.targets_collected,
                "distance": a.distance_travelled, "enemies": a.enemies_encountered,
                "active": a.active
            }
        self.mission_record["per_agent"] = per_agent_data
        
        stats_key = "custom" if self.custom_data else self.num_agents
        fn = get_stats_filename(stats_key)
    
        # Αποθήκευση στο αρχείο του σεναρίου
        # fn = get_stats_filename(self.num_agents)
        # data = safe_json_load(fn, {"missions": [], "summary": {}})
        # data["missions"].append(self.mission_record)
        
        # 1. Επιλογή του σωστού κλειδιού (custom ή αριθμός agents)
        if self.custom_data:
            stats_key = "custom"
        else:
            stats_key = self.num_agents
            
        # 2. Ανάκτηση του ονόματος αρχείου με το σωστό κλειδί
        fn = get_stats_filename(stats_key)
        
        # 3. Φόρτωση και Ενημέρωση
        data = safe_json_load(fn, {"missions": [], "summary": {}})
        data["missions"].append(self.mission_record)
        
        
        # # Ενημέρωση περίληψής
        # summary = data.get("summary", {})
        # summary.setdefault("total_missions", 0)
        # summary["total_missions"] += 1
        # summary[result] = summary.get(result, 0) + 1
        
        # valid_missions = [m for m in data["missions"] if m.get("result") in ["success", "partial"]]
        # if valid_missions:
        #     summary["avg_time"] = sum(m.get("duration", 0) for m in valid_missions) / len(valid_missions)
        #     summary["avg_score"] = sum(sum(p.get("score", 0) for p in m.get("per_agent", {}).values()) for m in valid_missions) / len(valid_missions)
        #     if result == "success":
        #         success_times = [m.get("duration", float('inf')) for m in valid_missions if m.get("result") == "success"]
        #         if success_times:
        #             current_best = summary.get("best_time", float('inf'))
        #             summary["best_time"] = min(current_best, min(success_times))
        
        # data["summary"] = summary
        # safe_json_save(fn, data)
        
        # # Ενημέρωση master αρχείου
        # master = safe_json_load(MASTER_STATS_FILE, {"by_agents": {str(i): [] for i in (1,2,3,4)}})
        # master["by_agents"].setdefault(str(self.num_agents), [])
        # master["by_agents"][str(self.num_agents)].append({
        #     "duration": elapsed,
        #     "result": result,
        #     "total_score": sum(a.score for a in self.agents),
        #     "total_targets": sum(a.targets_collected for a in self.agents)
        # })
        # safe_json_save(MASTER_STATS_FILE, master)

        # self.finished_flag = True
        
        # --- ΥΠΟΛΟΓΙΣΜΟΣ ΣΤΑΤΙΣΤΙΚΩΝ ---
        s = data.get("summary", {})
        s["total_missions"] = s.get("total_missions",0) + 1
        s[result] = s.get(result,0) + 1
        
        valid = [m for m in data["missions"] if m.get("result") in ["success", "partial"]]
        if valid:
            # 1. Time Stats
            s["avg_time"] = sum(m.get("duration", 0) for m in valid) / len(valid)
            if result == "success":
                curr_time = min(m.get("duration", float('inf')) for m in valid if m.get("result") == "success")
                s["best_time"] = min(s.get("best_time", float('inf')), curr_time)

            # 2. Team Score Stats (Συνολικό Σκορ Αποστολής)
            team_scores = [sum(p.get("score", 0) for p in m.get("per_agent", {}).values()) for m in valid]
            s["avg_score"] = sum(team_scores) / len(valid) # Avg Team Score
            s["best_team_score"] = max(team_scores)        # Best Team Score

            # 3. Agent Score Stats (Ατομικό Σκορ Πράκτορα)
            # Μαζεύουμε όλα τα σκορ όλων των πρακτόρων από όλες τις valid αποστολές
            all_agent_scores = [p.get("score", 0) for m in valid for p in m.get("per_agent", {}).values()]
            if all_agent_scores:
                s["avg_agent_score"] = sum(all_agent_scores) / len(all_agent_scores) # Avg Agent Score
                s["best_agent_score"] = max(all_agent_scores)                        # Best Agent Score
        
        data["summary"] = s
        safe_json_save(fn, data)
        
        # Master update
        master = safe_json_load(MASTER_STATS_FILE, {"by_agents": {str(i): [] for i in (1,2,3,4)}})
        master["by_agents"].setdefault(str(self.num_agents), [])
        master["by_agents"][str(self.num_agents)].append({
            "duration": elapsed, "result": result,
            "total_score": sum(a.score for a in self.agents)
        })
        safe_json_save(MASTER_STATS_FILE, master)

        self.finished_flag = True
        
        
        
        
    # Σχεδίαση
    def draw(self):
        # 1. Πάνω Μέρος (Παιχνίδι)
        self.draw_playfield(self.game_surface)
        self.draw_overlays()
        
        # 2. Δεξιά Στήλη (Γραφήματα)
        self.stats_surface.blit(self.chart_surface_cache, (0, 0))
        
        # 3. Κάτω Μέρος (Αλγόριθμος)
        self.draw_algorithm_view()
        
        # --- 4. ΧΑΡΑΚΕΣ (RULERS) ---
        # Καλείται τελευταίο για να φαίνεται πάνω από χάρτες
        self.draw_rulers()
        
        pygame.display.flip()

    def draw_playfield(self, surf):
        """ Σχεδιάζει το πλέγμα, τους στόχους, τους εχθρούς και τα ρομπότ. """
        surf.fill(COLOR_FREE)
        # Κελιά
        for y in range(GRID_HEIGHT_CELLS):
            for x in range(GRID_WIDTH_CELLS):
                t = self.game_map[y][x]
                c = None
                if t == TILE_TYPE_OBSTACLE: c = COLOR_OBSTACLE
                elif t == TILE_TYPE_DIFFICULT: c = COLOR_DIFFICULT
                elif t == TILE_TYPE_HAZARD: c = COLOR_HAZARD
                
                if c:
                    pygame.draw.rect(surf, c, (x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE))
                
                pygame.draw.rect(surf, BLACK, (x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE), 1)

        if hasattr(self, 'ignored_collectibles'):
            for ipos in self.ignored_collectibles:
                # Γκρι κύκλος (δηλώνει inactive)
                pygame.draw.circle(surf, (100, 100, 100), 
                                  (ipos[0]*TILE_SIZE + TILE_SIZE//2, ipos[1]*TILE_SIZE + TILE_SIZE//2), 
                                  TILE_SIZE//3, 1) # Το 1 στο τέλος το κάνει hollow (κούφιο)
        
        # Στόχοι
        for cpos in self.collectibles:
            pygame.draw.circle(surf, COLOR_COLLECTIBLE, (cpos[0]*TILE_SIZE + TILE_SIZE//2, cpos[1]*TILE_SIZE + TILE_SIZE//2), TILE_SIZE//3)
        
        # Εχθροί
        for obs in self.dynamic_obstacles:
            obs.draw(surf)
            
        # Διαδρομές
        for a in self.agents:
            if a.path and a.active:
                tmp = pygame.Surface((GAME_WIDTH, GAME_HEIGHT), pygame.SRCALPHA)
                for i, step in enumerate(a.path[:20]):
                    alpha = max(20, 150 - i*7)
                    rect = (step[0]*TILE_SIZE + 5, step[1]*TILE_SIZE + 5, TILE_SIZE-10, TILE_SIZE-10)
                    pygame.draw.rect(tmp, (a.color[0], a.color[1], a.color[2], alpha), rect, border_radius=3)
                surf.blit(tmp, (0,0))
                
        # Agents
        for a in self.agents:
            a.draw(surf)

    def draw_overlays(self):
        """ Σχεδιάζει μηνύματα PAUSED / FINISHED πάνω από το game_surface. """
        
        tsec = int(self.get_elapsed_seconds())
        active_count = sum(1 for a in self.agents if a.active)
        collected_count = self.total_collectibles - len(self.collectibles)
        
        txt = f"Time: {tsec}s | Agents: {active_count}/{self.num_agents} | Targets: {collected_count}/{self.total_collectibles}"
        surf_txt = self.font_medium.render(txt, True, WHITE)
        
        text_rect = surf_txt.get_rect(topleft=(5, 5))
        bg_rect = text_rect.inflate(10, 5) 
        bg_surf = pygame.Surface(bg_rect.size, pygame.SRCALPHA)
        bg_surf.fill((0, 0, 0, 150))
        self.game_surface.blit(bg_surf, bg_rect)
        self.game_surface.blit(surf_txt, text_rect)

        if self.paused:
            s = self.font_large.render("PAUSED - Press SPACE to resume", True, MENU_HIGHLIGHT_COLOR)
            rect = s.get_rect(center=(GAME_WIDTH//2, GAME_HEIGHT//2 - 30))
            pygame.draw.rect(self.game_surface, (0,0,0, 200), rect.inflate(20,10))
            self.game_surface.blit(s, rect)
            
        if self.finished_flag:
            res = self.mission_record.get("result", "finished")
            msg = f"MISSION {res.upper()} (R: Restart / ESC: Menu)"
            color = (0,255,0) if res == "success" else ((255,255,0) if res == "partial" else (255,0,0))
            s = self.font_large.render(msg, True, color)
            rect = s.get_rect(center=(GAME_WIDTH//2, GAME_HEIGHT//2))
            pygame.draw.rect(self.game_surface, (0,0,0, 200), rect.inflate(20,10))
            self.game_surface.blit(s, rect)

    def draw_rulers(self):
        """ 
        Ζωγραφίζει τους χάρακες (Axes) για ευκολότερη ανάλυση.
        - Κεντρικός Άξονας Χ (Κόκκινο): Κοινός και για τα δύο.
        - Άξονας Υ Πάνω (Μπλε): Για το Game (0-30).
        - Άξονας Υ Κάτω (Πράσινο): Για το Heatmap (0-30).
        """
        font = pygame.font.SysFont("Arial", 10, bold=True)
        
        # Χρώματα
        # col_x = (255, 50, 50)     # Κόκκινο (Κεντρικός Χ)
        # col_y_up = (50, 150, 255) # Μπλε (Πάνω Υ)
        # col_y_down = (50, 255, 50)# Πράσινο (Κάτω Υ)
        
        col_x = (0, 0, 0)     
        col_y_up = (0, 0, 0) 
        col_y_down = (0, 0, 0)
        col_x_num = WHITE
        col_y_up_num = WHITE
        col_y_down_num = WHITE
        
        
        mid_y = GAME_HEIGHT # Η διαχωριστική γραμμή (Y=600)

        # --- 1. ΚΕΝΤΡΙΚΟΣ ΑΞΟΝΑΣ Χ ---
        # Γραμμή στη μέση
        pygame.draw.line(self.screen, col_x, (0, mid_y), (GAME_WIDTH, mid_y), 3)
        
        for x in range(0, GRID_WIDTH_CELLS, 5): # Κάθε 5 κελιά
            pos_x = x * TILE_SIZE
            # Κάθετη γραμμούλα (tick)
            pygame.draw.line(self.screen, col_x, (pos_x, mid_y - 5), (pos_x, mid_y + 5), 2)
            
            # Αριθμός
            txt = font.render(str(x), True, col_x)
            # Τον βάζουμε λίγο πιο πάνω και λίγο πιο κάτω για να φαίνεται και στα δύο
            # self.screen.blit(txt, (pos_x + 2, mid_y - 15)) 
            # self.screen.blit(txt, (pos_x + 2, mid_y + 5))
            self.screen.blit(txt, (pos_x, mid_y )) 
            
        # --- 2. ΑΞΟΝΑΣ Υ - ΠΑΝΩ ---
        # Γραμμή αριστερά (από 0 έως μέση)
        pygame.draw.line(self.screen, col_y_up, (0, 0), (0, mid_y), 3)
        
        for y in range(0, GRID_HEIGHT_CELLS, 5):
            pos_y = y * TILE_SIZE
            # Οριζόντια γραμμούλα (tick)
            pygame.draw.line(self.screen, col_y_up, (0, pos_y), (8, pos_y), 2)
            
            # Αριθμός
            txt = font.render(str(y), True, col_y_up)
            # self.screen.blit(txt, (10, pos_y - 5))
            self.screen.blit(txt, (10, pos_y ))

        # --- 3. ΑΞΟΝΑΣ Υ - ΚΑΤΩ ---
        # Γραμμή αριστερά (από μέση έως κάτω)
        pygame.draw.line(self.screen, col_y_down, (0, mid_y), (0, TOTAL_SCREEN_HEIGHT), 3)
        
        for y in range(0, GRID_HEIGHT_CELLS, 5):
            # Υπολογισμός θέσης (προσθέτουμε το mid_y για να πάει κάτω)
            pos_y = mid_y + (y * TILE_SIZE) 
            
            # Οριζόντια γραμμούλα (tick)
            pygame.draw.line(self.screen, col_y_down, (0, pos_y), (8, pos_y), 2)
            
            # Αριθμός (τυπώνουμε το y, όχι το pos_y, για να δείχνει 0-30)
            txt = font.render(str(y), True, col_y_down)
            # self.screen.blit(txt, (10, pos_y - 5))
            self.screen.blit(txt, (10, pos_y ))

    def redraw_charts(self):
        """ 
        Layout 5 Γραφημάτων:
        Roq 1: Targets & Score (Full Width)
        Row 2: Left Speed (Left) | Enemies (Right)
        Row 3: Bar Chart (Left) | Pie Chart (Right)
        """
        if not MATPLOTLIB_AVAILABLE: return
        try:
            dpi = 100.0
            fig_w = PANEL_WIDTH / dpi
            fig_h = self.chart_height / dpi # 600px ύψος
            mpl_bg_color = (COLOR_PANEL_BG[0]/255.0, COLOR_PANEL_BG[1]/255.0, COLOR_PANEL_BG[2]/255.0)
            
            fig = plt.figure(figsize=(fig_w, fig_h), dpi=dpi, facecolor=mpl_bg_color)
            
            gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1]) 
            fig.subplots_adjust(left=0.10, right=0.90, top=0.94, bottom=0.06, hspace=0.5, wspace=0.3)

            plt.rcParams.update({'text.color': 'white', 'axes.labelcolor': 'white', 'xtick.color': 'white', 
                                 'ytick.color': 'white', 'axes.edgecolor': 'white', 'axes.facecolor': mpl_bg_color, 
                                 'font.size': 6, 'axes.titlesize': 8})

            max_time = self.get_elapsed_seconds()
            
            # Grid: 3 Γραμμές με άνετο spacing
            gs = fig.add_gridspec(3, 2, height_ratios=[1.2, 1, 1]) 
            fig.subplots_adjust(left=0.12, right=0.88, top=0.94, bottom=0.08, hspace=0.4, wspace=0.3)

            plt.rcParams.update({'text.color': 'white', 'axes.labelcolor': 'white', 'xtick.color': 'white', 
                                 'ytick.color': 'white', 'axes.edgecolor': 'white', 'axes.facecolor': mpl_bg_color, 
                                 'font.size': 8, 'axes.titlesize': 10})

            max_time = self.get_elapsed_seconds()
            
            
            # --- 1. TOP: DUAL AXIS ---
            ax1 = fig.add_subplot(gs[0, :])
            ax2 = ax1.twinx()
            ax1.set_title("Targets (Solid) & Score (Dashed)")
            ax1.set_ylabel("Targets", color='cyan'); ax2.set_ylabel("Score", color='orange')
            ax1.tick_params(axis='y', labelcolor='cyan'); ax2.tick_params(axis='y', labelcolor='orange')
            
            for i, a in enumerate(self.agents):
                d = self.per_agent_histories[i]
                if d["times"]:
                    c = tuple(v/255.0 for v in a.color)
                    ax1.plot(d["times"], d["targets"], color=c, ls='-', lw=1.0, alpha=0.9)
                    ax2.plot(d["times"], d["score"], color=c, ls='--', lw=1.0, alpha=0.6)
            ax1.set_xlim(0, max(10, max_time)); ax1.grid(True, alpha=0.15)
            # Αναγκάζουμε να δείχνει μόνο ακέραιους
            ax1.locator_params(axis='y', integer=True)

            # --- 2. MID-LEFT: SPEED ---
            ax_speed = fig.add_subplot(gs[1, 0])
            for i in range(self.num_agents):
                d = self.per_agent_histories[i]
                s = d["speed"]
                if len(d["times"]) > 10 and PANDAS_AVAILABLE:
                    s = pd.Series(s).rolling(10, min_periods=1).mean().tolist()
                if d["times"]:
                    c = tuple(v/255.0 for v in self.agents[i].color)
                    ax_speed.plot(d["times"], s, label=self.agents[i].name, color=c, alpha=0.9)
            ax_speed.set_title("Avg Speed")
            ax_speed.set_ylim(0, 1.1); ax_speed.set_xlim(0, max(10, max_time))
            ax_speed.grid(True, alpha=0.15)

            # --- 3. MID-RIGHT: ENEMIES ---
            ax_enc = fig.add_subplot(gs[1, 1])
            max_enc_val = 0
            for i in range(self.num_agents):
                d = self.per_agent_histories[i]
                if d["times"]:
                    c = tuple(v/255.0 for v in self.agents[i].color)
                    ax_enc.plot(d["times"], d["encounters"], color=c, lw=1.5, alpha=0.9)
                    if d["encounters"]: max_enc_val = max(max_enc_val, d["encounters"][-1])
            ax_enc.set_title("Enemies Encountered")
            ax_enc.set_xlim(0, max(10, max_time))
            ax_enc.set_ylim(0, max(5, max_enc_val + 2))
            ax_enc.grid(True, alpha=0.15)

            # --- 4. BOT-LEFT: BAR ---
            ax_bar = fig.add_subplot(gs[2, 0])
            names = [a.name for a in self.agents]
            scores = [a.score for a in self.agents]
            targs = [a.targets_collected for a in self.agents]
            x = range(len(names)); w = 0.35
            ax_bar.bar([i-w/2 for i in x], scores, w, color='orange')
            ax_bar.set_ylabel("Score", color='orange')
            ax_bar.tick_params(axis='y', labelcolor='orange')
            ax_bar2 = ax_bar.twinx()
            ax_bar2.bar([i+w/2 for i in x], targs, w, color='cyan')
            ax_bar2.set_ylabel("Trgts", color='cyan')
            ax_bar2.tick_params(axis='y', labelcolor='cyan')
            ax_bar.set_xticks(x); ax_bar.set_xticklabels(names); ax_bar.set_title("Current Status")
            # Αναγκάζουμε να δείχνει μόνο ακέραιους
            ax_bar.locator_params(axis='y', integer=True)
            ax_bar2.locator_params(axis='y', integer=True)
            
            # --- 5. BOT-RIGHT: PIE (CLEANED) ---
            ax_pie = fig.add_subplot(gs[2, 1])
            fn = get_stats_filename(self.num_agents)
            data = safe_json_load(fn, {"summary": {}})
            s = data.get("summary",{}).get("success",0)
            p = data.get("summary",{}).get("partial",0)
            f = data.get("summary",{}).get("failed",0)
                      
            total_runs = s + p + f  
            # if total_runs > 0:
            #     # Συνάρτηση για να δείχνει το απόλυτο νούμερο (π.χ. "5") αντί για %
            #     def make_autopct(values):
            #         def my_autopct(pct):
            #             total = sum(values)
            #             val = int(round(pct*total/100.0))
            #             return '{v:d}'.format(v=val) if val > 0 else ''
            #         return my_autopct

            #     ax_pie.pie([s, p, f], labels=["Win", "Part", "Fail"], 
            #                colors=['#00cc00', '#cccc00', '#cc0000'], 
            #                autopct=make_autopct([s, p, f]), # <-- Εμφάνιση αριθμού
            #                pctdistance=0.6, 
            #                textprops={'color':'white', 'fontsize': 8, 'weight':'bold'}, 
            #                wedgeprops=dict(width=1.0)) # Γεμάτη πίτα (όχι donut)
                
            #     # Εμφάνιση Συνολικού Αριθμού (Κάτω δεξιά, εκτός πίτας)
            #     ax_pie.text(2, -1.2, f"Total Runs: {total_runs}", ha='right', va='bottom', 
            #                 color='white', fontsize=9, fontweight='bold',
            #                 bbox=dict(facecolor='black', alpha=0.5, edgecolor='none'))
            #     ax_pie.text(2, -1.6, f"Current Run: {total_runs + 1}", ha='right', va='bottom', 
            #                 color='white', fontsize=9, fontweight='bold',
            #                 bbox=dict(facecolor='black', alpha=0.5, edgecolor='none'))
            # else: 
            #     ax_pie.text(0.5, 0.5, "No Data", ha='center', color='gray')
            
            # ax_pie.set_title("History (Counts)")
            
            # Φιλτράρισμα: Κράτα μόνο όσα είναι > 0
            sizes = []
            labels = []
            colors = []
            
            if s > 0:
                sizes.append(s); labels.append("Win"); colors.append('green')
            if p > 0:
                sizes.append(p); labels.append("Part"); colors.append('yellow')
            if f > 0:
                sizes.append(f); labels.append("Fail"); colors.append('red')
            
            if sizes:
                ax_pie.pie(sizes, labels=labels, colors=colors, autopct='%1.0f%%', 
                           textprops={'color':'white', 'weight':'bold', 'fontsize': 6}, startangle=90)
                # Εμφάνιση Συνολικού Αριθμού (Κάτω δεξιά, εκτός πίτας)
                ax_pie.text(2, -1.2, f"Total Runs: {total_runs}", ha='right', va='bottom', 
                            color='white', fontsize=9, fontweight='bold',
                            bbox=dict(facecolor='black', alpha=0.5, edgecolor='none'))
                ax_pie.text(2, -1.6, f"Current Run: {total_runs + 1}", ha='right', va='bottom', 
                            color='white', fontsize=9, fontweight='bold',
                            bbox=dict(facecolor='black', alpha=0.5, edgecolor='none'))
            else: 
                ax_pie.text(0.5, 0.5, "No Data", ha='center', color='gray')
            ax_pie.set_title("History")

            # Render
            buf = BytesIO()
            fig.savefig(buf, format="png", facecolor=mpl_bg_color)
            plt.close(fig); buf.seek(0)
            img = pygame.image.load(buf)
            self.chart_surface_cache.fill(COLOR_PANEL_BG); self.chart_surface_cache.blit(img, (0,0)); buf.close()
        except BaseException as e: print(f"Chart Error: {e}")

#  MENUS
def main_menu(screen):
    """ Κύριο μενού """
    pygame.display.set_caption("SAR - Κεντρικό Μενού")
    font = pygame.font.SysFont("Arial", 24)
    selected = 0
    
    options = [
        "1. Επίγειο Ρομπότ (Τυχαίο)",
        "2. Σενάριο Ελέγχου (Σταθερό)",
        "3. Στατιστικά Επίγειου Ρομπότ" 
    ]
    keys = ["ground_random", "ground_test", "stats"]

    while True:
        screen.fill(MENU_BG_COLOR)
        title = font.render("SAR - Επιλογή Σεναρίου", True, (255, 0, 127))
        screen.blit(title, title.get_rect(center=(275, 40)))
        
        for i, opt in enumerate(options):
            color = MENU_HIGHLIGHT_COLOR if i == selected else MENU_TEXT_COLOR
            screen.blit(font.render(opt, True, color), 
                       font.render(opt, True, color).get_rect(center=(275, 100 + i * 45)))
        
        pygame.display.flip()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_DOWN:
                    selected = (selected + 1) % len(options)
                if event.key == pygame.K_UP:
                    selected = (selected - 1) % len(options)
                if event.key == pygame.K_RETURN:
                    return keys[selected]
                if event.key == pygame.K_ESCAPE:
                    return None
        pygame.time.Clock().tick(30)

def settings_selection_menu(screen):
    """ Μενού επιλογής Ρομπότ & Στόχων """
    pygame.display.set_caption("SAR - Ρυθμίσεις Αποστολής")
    font = pygame.font.SysFont("Arial", 24)
    
    num_robots = 1
    num_targets = DEFAULT_TARGETS
    min_robots, max_robots = 1, 4
    min_targets, max_targets = 6, 20
    
    selected_option = 0 # 0 = Robots, 1 = Targets
    
    while True:
        screen.fill(MENU_BG_COLOR)
        title = font.render("Ρυθμίσεις Αποστολής", True, MENU_HIGHLIGHT_COLOR)
        screen.blit(title, title.get_rect(center=(275, 40)))
        
        color_r = MENU_HIGHLIGHT_COLOR if selected_option == 0 else MENU_TEXT_COLOR
        robot_text = f"Αριθμός Ρομπότ: < {num_robots} > " + ("(Σόλο)" if num_robots == 1 else "(Co-op)")
        robot_surf = font.render(robot_text, True, color_r)
        screen.blit(robot_surf, robot_surf.get_rect(center=(275, 120)))
        
        color_t = MENU_HIGHLIGHT_COLOR if selected_option == 1 else MENU_TEXT_COLOR
        target_text = f"Αριθμός Στόχων: < {num_targets} >"
        target_surf = font.render(target_text, True, color_t)
        screen.blit(target_surf, target_surf.get_rect(center=(275, 170)))
        
        instr = font.render("Enter: Επόμενο, Esc: Πίσω", True, (200, 200, 200))
        screen.blit(instr, instr.get_rect(center=(275, 280)))
        
        pygame.display.flip()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: return None
                if event.key == pygame.K_RETURN:
                    return num_robots, num_targets
                if event.key in [pygame.K_UP, pygame.K_DOWN]:
                    selected_option = 1 - selected_option
                if event.key == pygame.K_LEFT:
                    if selected_option == 0:
                        num_robots = max(min_robots, num_robots - 1)
                    else:
                        num_targets = max(min_targets, num_targets - 5)
                if event.key == pygame.K_RIGHT:
                    if selected_option == 0:
                        num_robots = min(max_robots, num_robots + 1)
                    else:
                        num_targets = min(max_targets, num_targets + 5)
        pygame.time.Clock().tick(30)
                
def enemy_selection_menu(screen):
    """ Μενού επιλογής εχθρών """
    pygame.display.set_caption("SAR - Ρύθμιση Εχθρών")
    font = pygame.font.SysFont("Arial", 24)
    
    num_patrol, num_aggressive = DEFAULT_PATROL, DEFAULT_AGGRESSIVE
    min_val, max_patrol, max_aggr = 0, 8, 6
    selected_option = 0
    
    while True:
        screen.fill(MENU_BG_COLOR)
        title = font.render("Ρύθμιση Εχθρών", True, MENU_HIGHLIGHT_COLOR)
        screen.blit(title, title.get_rect(center=(275, 40)))
        
        color_p = MENU_HIGHLIGHT_COLOR if selected_option == 0 else MENU_TEXT_COLOR
        patrol_txt = font.render(f"Εχθροί Περιπολίας: < {num_patrol} >", True, color_p)
        screen.blit(patrol_txt, patrol_txt.get_rect(center=(275, 120)))
        
        color_a = MENU_HIGHLIGHT_COLOR if selected_option == 1 else MENU_TEXT_COLOR
        aggr_txt = font.render(f"Επιθετικοί Εχθροί: < {num_aggressive} >", True, color_a)
        screen.blit(aggr_txt, aggr_txt.get_rect(center=(275, 170)))
        
        instr = font.render("Enter: Έναρξη, Esc: Πίσω", True, (200, 200, 200))
        screen.blit(instr, instr.get_rect(center=(275, 250)))
        
        pygame.display.flip()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: return None
                if event.key == pygame.K_RETURN:
                    return num_patrol, num_aggressive
                if event.key in [pygame.K_UP, pygame.K_DOWN]:
                    selected_option = 1 - selected_option
                if event.key == pygame.K_LEFT:
                    if selected_option == 0: num_patrol = max(min_val, num_patrol - 1)
                    else: num_aggressive = max(min_val, num_aggressive - 1)
                if event.key == pygame.K_RIGHT:
                    if selected_option == 0: num_patrol = min(max_patrol, num_patrol + 1)
                    else: num_aggressive = min(max_aggr, num_aggressive + 1)
        pygame.time.Clock().tick(30)
        
def show_stats_screen(screen):
    font = pygame.font.SysFont("Arial", 18); small = pygame.font.SysFont("Arial", 14)
    ensure_stats_files_exist()
    
    # Λίστα κλειδιών: 1-4 Agents ΚΑΙ το "custom"
    keys_to_show = [1, 2, 3, 4, "custom"]
    
    # Φόρτωση όλων των δεδομένων
    all_data = {k: safe_json_load(get_stats_filename(k), {}).get("summary", {}) for k in keys_to_show}
    
    pies = {}
    if MATPLOTLIB_AVAILABLE:
        for k, d in all_data.items():
            s, p, f = d.get("success",0), d.get("partial",0), d.get("failed",0)
            if s+p+f > 0:
                fig = plt.figure(figsize=(1.1, 1.1), dpi=100); fig.patch.set_alpha(0)
                ax = fig.add_subplot(111)
                l = [f"{s/(s+p+f):.0%}\nWin" if s else "", f"{p/(s+p+f):.0%}\nPart" if p else "", f"{f/(s+p+f):.0%}\nFail" if f else ""]
                ax.pie([s,p,f], labels=l, colors=['green','yellow','red'], labeldistance=0.3, textprops={'fontsize':7, 'weight':'bold'})
                buf=BytesIO(); fig.savefig(buf, format="png", transparent=True); plt.close(fig); buf.seek(0)
                pies[k] = pygame.image.load(buf); buf.close()

    while True:
        screen.fill(MENU_BG_COLOR)
        screen.blit(font.render("Statistics Overview", True, MENU_HIGHLIGHT_COLOR), (300, 15))
        
        y = 50 # Ξεκινάμε λίγο πιο ψηλά
        
        for k in keys_to_show:
            d = all_data[k]
            
            # Διαφορετικός τίτλος για το Custom
            if k == "custom":
                title_str = "Custom Sandbox Mode"
                box_color = (100, 100, 150) # Ελαφρώς διαφορετικό περίγραμμα για να ξεχωρίζει
            else:
                title_str = f"{k} Agent(s) Scenario"
                box_color = COLOR_PANEL_BORDER

            # Σχεδίαση Πλαισίου (Ύψος 95 pixels για να χωρέσουν 5)
            pygame.draw.rect(screen, box_color, (50, y, 700, 95), 1)
            
        #     # Κείμενα
        #     screen.blit(font.render(title_str, True, MENU_HIGHLIGHT_COLOR), (70, y+10))
        #     screen.blit(small.render(f"Missions: {d.get('total_missions',0)}", True, WHITE), (70, y+35))
            
        #     # Αποτελέσματα
        #     res_txt = f"Success: {d.get('success',0)} | Partial: {d.get('partial',0)} | Failed: {d.get('failed',0)}"
        #     screen.blit(small.render(res_txt, True, (200,200,200)), (70, y+55))
            
        #     # # Χρόνος (Αν υπάρχει Custom, μπορεί να μην έχει νόημα το Best Time αν αλλάζει ο χάρτης, αλλά το δείχνουμε)
        #     # if d.get('best_time', float('inf')) != float('inf'):
        #     #     best_time_str = f"{d.get('best_time'):.1f}s"
        #     #     avg_time = d.get('avg_time', 0.0)
        #     # else:
        #     #     best_time_str = "N/A"
        #     # avg_time = f"{avg_time:.1f}s"
        #     # # screen.blit(small.render(f"Best Time: {best_time_str}", True, (255,215,0)), (450, y+35))
        #     # time_txt = f"Best: {best_time_str} | Avg: {avg_time}"
        #     # screen.blit(small.render(time_txt, True, (255, 215, 0)), (450, y+35))           
        
        #     best_val = d.get('best_time', float('inf'))
        #     avg_val = d.get('avg_time', 0.0)
            
        #     best_str = f"{best_val:.1f}s" if best_val != float('inf') else "N/A"
        #     avg_str = f"{avg_val:.1f}s"
            
        #     time_txt = f"Best: {best_str} | Avg: {avg_str}"
        #     screen.blit(small.render(time_txt, True, (255, 215, 0)), (450, y+35))
         
        #     # Πιτα
        #     if k in pies: 
        #         screen.blit(pies[k], (630, y-5))
            
        #     y += 105 # Βήμα για το επόμενο κουτί

        # screen.blit(small.render("ESC: Return", True, (200,200,200)), (350, 570))
        # pygame.display.flip()
        
        # for e in pygame.event.get():
        #     if e.type == pygame.QUIT: sys.exit()
        #     if e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE: return        


        # --- ΑΡΙΣΤΕΡΗ ΣΤΗΛΗ (Γενικά) ---
            screen.blit(font.render(title_str, True, MENU_HIGHLIGHT_COLOR), (70, y+10))
            screen.blit(small.render(f"Missions: {d.get('total_missions',0)}", True, WHITE), (70, y+35))
            
            res_txt = f"Success: {d.get('success',0)} | Partial: {d.get('partial',0)} | Failed: {d.get('failed',0)}"
            screen.blit(small.render(res_txt, True, (200,200,200)), (70, y+55))
            
            # --- ΔΕΞΙΑ ΣΤΗΛΗ (Αναλυτικά Σκορ & Χρόνοι) ---
            # Time
            best_t = d.get('best_time', float('inf'))
            avg_t = d.get('avg_time', 0.0)
            time_str = f"Time: Avg {avg_t:.1f}s | Best {best_t:.1f}s" if best_t != float('inf') else "Time: N/A"
            screen.blit(small.render(time_str, True, (255, 215, 0)), (300, y+15))
            
            # Team Score
            avg_team = d.get('avg_score', 0)
            best_team = d.get('best_team_score', 0)
            team_str = f"Team Score: Avg {int(avg_team)} | Best {int(best_team)}"
            screen.blit(small.render(team_str, True, (100, 255, 100)), (300, y+35))
            
            # Agent Score
            avg_ag = d.get('avg_agent_score', 0)
            best_ag = d.get('best_agent_score', 0)
            agent_str = f"Agent Score: Avg {int(avg_ag)} | Best {int(best_ag)}"
            screen.blit(small.render(agent_str, True, (100, 200, 255)), (300, y+55))

            # Πίτα (Τέρμα Δεξιά)
            if k in pies: screen.blit(pies[k], (630, y-5))
            
            y += 105 

        screen.blit(small.render("ESC: Return", True, (200,200,200)), (350, 570))
        pygame.display.flip()
        
        for e in pygame.event.get():
            if e.type == pygame.QUIT: sys.exit()
            if e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE: return


def main():
    pygame.init()
    pygame.font.init()
    if not MATPLOTLIB_AVAILABLE or not PANDAS_AVAILABLE: print("Warning: Libs missing")
    ensure_stats_files_exist()
    
    menu_screen = pygame.display.set_mode((550, 350))

    while True:
        choice = main_menu(menu_screen)
        if choice is None: break
        
        if choice == "stats":
            stats_scr = pygame.display.set_mode((800, 600))
            show_stats_screen(stats_scr)
            menu_screen = pygame.display.set_mode((550, 350))
            continue
            
        # Setup Params
        params = {}
        custom_map_data = None 
        
        if choice == "ground_test": 
            
            current_map_data = None
            # Μπαίνουμε σε βρόχο: Editor -> Game -> Editor -> ...
            while True:
                # 1. Ανοίγουμε Editor (με τα προηγούμενα δεδομένα αν υπάρχουν)
                editor = MapEditor(pygame.display.set_mode((GAME_WIDTH, GAME_HEIGHT)), 
                                   use_stable_layout=True, 
                                   previous_data=current_map_data)
                
                # 2. Περιμένουμε τον χρήστη να σχεδιάσει και να πατήσει ENTER
                new_data = editor.run()
                
                # Αν ο χρήστης πάτησε ESC στον Editor, επιστρέφει None -> Βγαίνουμε στο Main Menu
                if new_data is None:
                    break 
                
                # Αλλιώς, κρατάμε τα νέα δεδομένα
                current_map_data = new_data
                
                # 3. Τρέχουμε το Παιχνίδι
                game = Game(custom_data=current_map_data, **params)
                game.run()
                # Όταν τελειώσει το παιχνίδι (ή πατηθεί ESC), ο βρόχος while ξαναρχίζει
                # και ξανανοίγει τον Editor με το current_map_data έτοιμο για αλλαγές!
            
            # Όταν σπάσει το while (ESC στον Editor), επαναφέρουμε το παράθυρο μενού
            menu_screen = pygame.display.set_mode((550, 350))
            continue
        
        elif choice == "ground_random":
            # RANDOM MODE
            settings_screen = pygame.display.set_mode((550, 350))
            settings = settings_selection_menu(settings_screen)
            if settings is None: continue 
            params["num_agents"], params["num_targets"] = settings
            
            enemy_screen = pygame.display.set_mode((550, 300))
            enemy_settings = enemy_selection_menu(enemy_screen)
            if enemy_settings is None: continue
            params["num_patrol"], params["num_aggressive"] = enemy_settings

        # ΕΚΚΙΝΗΣΗ GAME
        game = Game(custom_data=custom_map_data, **params)
        
        # Αν πατηθεί ESC μέσα στο παιχνίδι, η game.run() επιστρέφει True/False
        # και ο βρόχος while συνεχίζει, ξαναδείχνοντας το μενού.
        if not game.run():
            menu_screen = pygame.display.set_mode((550, 350))
            continue
            
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()