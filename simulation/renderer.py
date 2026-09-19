"""
simulation/renderer.py
=========================
Pygame-based 2D visualization of the warehouse simulation, per project
spec section 30.

DEVELOPMENT NOTE: pygame is not installable in the sandbox this project
was built in (no network egress, headless environment), so this module
is written carefully against the pygame API but has NOT been visually
run-tested here. The live Flask dashboard (dashboard/app.py) and the
offline HTML trace viewer (tools/build_trace_viewer.py) were used as
verified visual alternatives during development - see README.md section
6. If pygame fails to render correctly on your machine, please open an
issue; the rendering logic here mirrors the already-verified dashboard
JS (dashboard/static/dashboard.js) cell-for-cell, which reduces (but
does not eliminate) the risk of a translation error.

Design: this module ONLY draws. It reads Simulator/Warehouse/Robot state
every frame and never mutates it - the same "pure observer" rule as
dashboard/app.py and simulation/trace_recorder.py.
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

import config
from simulation.simulator import Simulator
from simulation.warehouse import CellType

Coordinate = Tuple[int, int]

CELL_COLORS = {
    CellType.EMPTY: (244, 244, 246),
    CellType.WALL: (43, 47, 56),
    CellType.SHELF: (138, 90, 43),
    CellType.PICKUP: (46, 158, 91),
    CellType.DROPOFF: (43, 111, 206),
    CellType.CHARGING_STATION: (224, 180, 0),
    CellType.INTERSECTION: (239, 227, 255),
    CellType.ROBOT: (0, 0, 0),
}

STATUS_COLORS = {
    "IDLE": (154, 160, 166),
    "MOVING": (46, 158, 91),
    "WAITING": (224, 163, 0),
    "NEGOTIATING": (199, 125, 255),
    "REROUTING": (255, 140, 66),
    "BLOCKED": (209, 72, 60),
    "CHARGING": (224, 180, 0),
    "COMPLETED": (91, 100, 114),
    "FAILED": (0, 0, 0),
}

ROBOT_PALETTE = [
    (230, 57, 70), (29, 53, 87), (42, 157, 143), (244, 162, 97),
    (106, 76, 147), (69, 123, 157), (255, 0, 110), (67, 170, 139),
]


class Renderer:
    """Pygame renderer. Import pygame lazily inside __init__ so the rest
    of the project (headless CLI, tests, dashboard) never needs pygame
    installed at all - only users who actually request the GUI do.
    """

    def __init__(self, simulator: Simulator, cell_size: int = config.CELL_SIZE_PX) -> None:
        import pygame  # local import: only required when the GUI is used
        self.pygame = pygame
        self.simulator = simulator
        self.cell_size = cell_size

        pygame.init()
        pygame.display.set_caption("Decentralized Multi-Robot Warehouse Simulation")
        width = simulator.warehouse.width * cell_size
        height = simulator.warehouse.height * cell_size + 60  # status bar
        self.screen = pygame.display.set_mode((width, height))
        self.font = pygame.font.SysFont("monospace", 14)
        self.small_font = pygame.font.SysFont("monospace", 11)
        self.clock = pygame.time.Clock()

        self._robot_colors: Dict[str, Tuple[int, int, int]] = {}
        for i, rid in enumerate(simulator.robots.keys()):
            self._robot_colors[rid] = ROBOT_PALETTE[i % len(ROBOT_PALETTE)]

        self.paused = False
        self.speed = config.SIMULATION_SPEED

    def handle_events(self) -> bool:
        """Process pygame events (START/PAUSE/RESET/SPEED controls).
        Returns False if the window was closed (caller should stop)."""
        pygame = self.pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                    if self.paused:
                        self.simulator.pause()
                    else:
                        self.simulator.resume()
                elif event.key == pygame.K_r:
                    self.simulator.reset()
                elif event.key == pygame.K_EQUALS or event.key == pygame.K_PLUS:
                    self.speed = min(8.0, self.speed * 2)
                elif event.key == pygame.K_MINUS:
                    self.speed = max(0.25, self.speed / 2)
        return True

    def draw_warehouse(self) -> None:
        pygame = self.pygame
        wh = self.simulator.warehouse
        for (x, y), cell_type in wh.grid.items():
            color = CELL_COLORS.get(cell_type, CELL_COLORS[CellType.EMPTY])
            rect = pygame.Rect(x * self.cell_size, y * self.cell_size, self.cell_size, self.cell_size)
            pygame.draw.rect(self.screen, color, rect)
            pygame.draw.rect(self.screen, (0, 0, 0, 20), rect, width=1)

        for (x, y) in wh.blocked_aisles:
            rect = pygame.Rect(x * self.cell_size, y * self.cell_size, self.cell_size, self.cell_size)
            overlay = pygame.Surface((self.cell_size, self.cell_size), pygame.SRCALPHA)
            overlay.fill((220, 60, 50, 140))
            self.screen.blit(overlay, rect)

    def draw_robots(self) -> None:
        pygame = self.pygame
        for rid, robot in self.simulator.robots.items():
            x, y = robot.position
            cx = x * self.cell_size + self.cell_size // 2
            cy = y * self.cell_size + self.cell_size // 2
            radius = int(self.cell_size * 0.38)

            color = self._robot_colors.get(rid, (200, 200, 200))
            pygame.draw.circle(self.screen, color, (cx, cy), radius)
            outline = STATUS_COLORS.get(robot.status.value, (255, 255, 255))
            pygame.draw.circle(self.screen, outline, (cx, cy), radius, width=2)

            label = self.small_font.render(rid, True, (255, 255, 255))
            label_rect = label.get_rect(center=(cx, cy))
            self.screen.blit(label, label_rect)

    def draw_status_bar(self) -> None:
        pygame = self.pygame
        wh = self.simulator.warehouse
        bar_y = wh.height * self.cell_size
        bar_rect = pygame.Rect(0, bar_y, wh.width * self.cell_size, 60)
        pygame.draw.rect(self.screen, (20, 22, 26), bar_rect)

        text = (f"t={self.simulator.sim_time:.1f}s  tick={self.simulator.tick_count}  "
                f"speed={self.speed:.2f}x  {'PAUSED' if self.paused else 'RUNNING'}  "
                f"[SPACE]=pause/resume [R]=reset [+/-]=speed")
        surface = self.font.render(text, True, (230, 230, 230))
        self.screen.blit(surface, (8, bar_y + 8))

        robot_text = "  ".join(
            f"{rid}:{r.status.value}" for rid, r in self.simulator.robots.items()
        )
        surface2 = self.small_font.render(robot_text, True, (180, 180, 180))
        self.screen.blit(surface2, (8, bar_y + 32))

    def render_frame(self) -> None:
        self.screen.fill((20, 22, 26))
        self.draw_warehouse()
        self.draw_robots()
        self.draw_status_bar()
        self.pygame.display.flip()

    def run(self, target_fps: int = 30) -> None:
        """Blocking render loop: advances the simulator and redraws each
        frame until the window is closed. The simulator's own step() is
        called directly here for simplicity in single-process GUI mode;
        for the --dashboard (headless + Flask) mode, main.py instead runs
        the simulator on a background thread - see main.py's
        run_with_dashboard() for that alternative wiring.
        """
        running = True
        while running:
            running = self.handle_events()
            if not running:
                break
            if not self.paused:
                self.simulator.step()
            self.render_frame()
            self.clock.tick(target_fps * self.speed)

        self.pygame.quit()
