"""
simulation/renderer_3d.py
============================
Real 3D visualization of the warehouse simulation using PyOpenGL, with
pygame supplying the window/OpenGL context (project spec section 30,
upgraded from 2D to true 3D per user request).

DEVELOPMENT NOTE (read before debugging): neither pygame nor PyOpenGL are
installable in the sandbox this project was built in (no network
access, headless environment). This module is written carefully against
the documented PyOpenGL/pygame APIs and syntax-checked
(`python -m py_compile`), but has NOT been visually run-tested. If
something renders incorrectly, that is expected as a real risk of
building 3D graphics code blind - please report exactly what you see
(a traceback, a black screen, wrong colors/shapes, camera issues) and
it can be fixed against real feedback rather than guessed at again.

ARCHITECTURE: this is a SEPARATE renderer from simulation/renderer.py
(the working, simpler 2D top-down pygame view). Both read the same
Simulator/Warehouse/Robot state and never mutate it - the same
"pure observer" rule used throughout this project (see
dashboard/app.py, simulation/trace_recorder.py). Use --gui for the 2D
view, --gui3d for this one.

CONTROLS:
    Left/Right arrow  - orbit camera around the warehouse (azimuth)
    Up/Down arrow      - raise/lower camera pitch
    +/-                 - zoom in/out
    SPACE               - pause/resume the simulation
    R                   - reset the simulation
    C                   - reset camera to default view
    Close window         - quit

DEPENDENCIES: requires `pip install PyOpenGL PyOpenGL_accelerate` in
addition to pygame (see requirements.txt).
"""

from __future__ import annotations

import math
from typing import Dict, Tuple

import config
from simulation.simulator import Simulator
from simulation.warehouse import CellType

Coordinate = Tuple[int, int]

# -- colors (RGB 0..1 floats, since OpenGL wants normalized color) ----------
FLOOR_COLORS = {
    CellType.EMPTY: (0.85, 0.85, 0.87),
    CellType.WALL: (0.17, 0.18, 0.22),
    CellType.SHELF: (0.54, 0.35, 0.17),
    CellType.PICKUP: (0.18, 0.62, 0.36),
    CellType.DROPOFF: (0.17, 0.44, 0.81),
    CellType.CHARGING_STATION: (0.88, 0.71, 0.0),
    CellType.INTERSECTION: (0.94, 0.89, 1.0),
    CellType.ROBOT: (0.0, 0.0, 0.0),
}

STATUS_COLORS = {
    "IDLE": (0.60, 0.63, 0.65),
    "MOVING": (0.18, 0.62, 0.36),
    "WAITING": (0.88, 0.64, 0.0),
    "NEGOTIATING": (0.78, 0.49, 1.0),
    "REROUTING": (1.0, 0.55, 0.26),
    "BLOCKED": (0.82, 0.28, 0.24),
    "CHARGING": (0.88, 0.71, 0.0),
    "COMPLETED": (0.36, 0.39, 0.45),
    "FAILED": (0.0, 0.0, 0.0),
}

ROBOT_PALETTE = [
    (0.90, 0.22, 0.28), (0.11, 0.21, 0.34), (0.16, 0.62, 0.56),
    (0.96, 0.64, 0.38), (0.42, 0.30, 0.58), (0.27, 0.48, 0.62),
    (1.0, 0.0, 0.43), (0.26, 0.67, 0.55),
]

WALL_HEIGHT = 1.2
SHELF_HEIGHT = 0.9
ROBOT_RADIUS = 0.32
ROBOT_HEIGHT = 0.5


class Renderer3D:
    """True 3D renderer: pygame owns the window/OpenGL context, PyOpenGL
    draws the scene every frame. Camera is an orbit camera looking down
    at the warehouse grid from a configurable azimuth/pitch/distance.
    """

    def __init__(self, simulator: Simulator) -> None:
        # Lazy imports: only users who actually request --gui3d need
        # pygame+PyOpenGL installed at all - mirrors simulation/renderer.py's
        # pattern for the 2D view.
        import pygame
        from OpenGL.GL import (
            glClearColor, glEnable, GL_DEPTH_TEST, GL_LIGHTING, GL_LIGHT0,
            GL_COLOR_MATERIAL, glColorMaterial, GL_FRONT_AND_BACK,
            GL_AMBIENT_AND_DIFFUSE, glLightfv, GL_POSITION, GL_AMBIENT,
            GL_DIFFUSE, glShadeModel, GL_SMOOTH, GL_BLEND, glBlendFunc,
            GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_CULL_FACE,
        )
        from OpenGL.GLU import gluPerspective
        self.pygame = pygame
        self.gl = __import__("OpenGL.GL", fromlist=["*"])
        self.glu = __import__("OpenGL.GLU", fromlist=["*"])

        self.simulator = simulator

        pygame.init()
        pygame.display.set_caption("Decentralized Multi-Robot Warehouse - 3D View")
        self.width, self.height = 1024, 720
        pygame.display.set_mode(
            (self.width, self.height),
            pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE,
        )
        self.font = pygame.font.SysFont("consolas", 16)
        self.clock = pygame.time.Clock()

        glClearColor(0.08, 0.09, 0.11, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, (0.0, 20.0, 0.0, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.35, 0.35, 0.38, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.9, 0.9, 0.9, 1.0))
        glShadeModel(GL_SMOOTH)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        self._setup_projection()

        wh = simulator.warehouse
        self.center = (wh.width / 2.0, 0.0, wh.height / 2.0)
        self.azimuth = 45.0     # degrees, orbiting around Y axis
        self.pitch = 35.0       # degrees above the horizon
        self.distance = max(wh.width, wh.height) * 1.3

        self._robot_colors: Dict[str, Tuple[float, float, float]] = {}
        for i, rid in enumerate(simulator.robots.keys()):
            self._robot_colors[rid] = ROBOT_PALETTE[i % len(ROBOT_PALETTE)]

        self._quadric = self.glu.gluNewQuadric()

        self.paused = False
        self.speed = config.SIMULATION_SPEED

    # -- setup -----------------------------------------------------------------
    def _setup_projection(self) -> None:
        GL = self.gl
        GLU = self.glu
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GLU.gluPerspective(50.0, self.width / self.height, 0.1, 200.0)
        GL.glMatrixMode(GL.GL_MODELVIEW)

    def _camera_position(self) -> Tuple[float, float, float]:
        az = math.radians(self.azimuth)
        pt = math.radians(self.pitch)
        cx, cy, cz = self.center
        x = cx + self.distance * math.cos(pt) * math.sin(az)
        y = cy + self.distance * math.sin(pt)
        z = cz + self.distance * math.cos(pt) * math.cos(az)
        return (x, y, z)

    def _apply_camera(self) -> None:
        GL = self.gl
        GLU = self.glu
        GL.glLoadIdentity()
        eye = self._camera_position()
        GLU.gluLookAt(eye[0], eye[1], eye[2],
                       self.center[0], self.center[1], self.center[2],
                       0.0, 1.0, 0.0)

    # -- input -------------------------------------------------------------
    def handle_events(self) -> bool:
        pygame = self.pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.VIDEORESIZE:
                self.width, self.height = event.w, event.h
                pygame.display.set_mode((self.width, self.height),
                                         pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE)
                self._setup_projection()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                    if self.paused:
                        self.simulator.pause()
                    else:
                        self.simulator.resume()
                elif event.key == pygame.K_r:
                    self.simulator.reset()
                elif event.key == pygame.K_c:
                    self.azimuth, self.pitch = 45.0, 35.0
                    wh = self.simulator.warehouse
                    self.distance = max(wh.width, wh.height) * 1.3

        keys = pygame.key.get_pressed()
        rotate_speed = 60.0 / 60.0   # degrees per frame at 60fps
        if keys[pygame.K_LEFT]:
            self.azimuth -= rotate_speed
        if keys[pygame.K_RIGHT]:
            self.azimuth += rotate_speed
        if keys[pygame.K_UP]:
            self.pitch = min(85.0, self.pitch + rotate_speed)
        if keys[pygame.K_DOWN]:
            self.pitch = max(5.0, self.pitch - rotate_speed)
        if keys[pygame.K_EQUALS] or keys[pygame.K_PLUS] or keys[pygame.K_KP_PLUS]:
            self.distance = max(3.0, self.distance - 0.3)
        if keys[pygame.K_MINUS] or keys[pygame.K_KP_MINUS]:
            self.distance = min(80.0, self.distance + 0.3)

        return True

    # -- drawing helpers ---------------------------------------------------
    def _draw_box(self, x: float, z: float, dx: float, dz: float,
                  height: float, color: Tuple[float, float, float], y0: float = 0.0) -> None:
        """Draw an axis-aligned box (a cell's wall/shelf/floor volume)
        spanning [x, x+dx] x [y0, y0+height] x [z, z+dz]."""
        GL = self.gl
        x1, x2 = x, x + dx
        y1, y2 = y0, y0 + height
        z1, z2 = z, z + dz
        GL.glColor3f(*color)

        faces = [
            # top
            [(x1, y2, z1), (x2, y2, z1), (x2, y2, z2), (x1, y2, z2)],
            # bottom
            [(x1, y1, z2), (x2, y1, z2), (x2, y1, z1), (x1, y1, z1)],
            # front (+z)
            [(x1, y1, z2), (x2, y1, z2), (x2, y2, z2), (x1, y2, z2)],
            # back (-z)
            [(x2, y1, z1), (x1, y1, z1), (x1, y2, z1), (x2, y2, z1)],
            # left (-x)
            [(x1, y1, z1), (x1, y1, z2), (x1, y2, z2), (x1, y2, z1)],
            # right (+x)
            [(x2, y1, z2), (x2, y1, z1), (x2, y2, z1), (x2, y2, z2)],
        ]
        GL.glBegin(GL.GL_QUADS)
        for face in faces:
            for vx, vy, vz in face:
                GL.glVertex3f(vx, vy, vz)
        GL.glEnd()

    def _draw_floor(self) -> None:
        wh = self.simulator.warehouse
        for (x, y), cell_type in wh.grid.items():
            color = FLOOR_COLORS.get(cell_type, FLOOR_COLORS[CellType.EMPTY])
            if cell_type == CellType.WALL:
                self._draw_box(x, y, 1.0, 1.0, WALL_HEIGHT, color)
            elif cell_type == CellType.SHELF:
                self._draw_box(x, y, 1.0, 1.0, SHELF_HEIGHT, color)
            else:
                self._draw_box(x, y, 1.0, 1.0, 0.05, color)

        # Blocked (dynamic) aisles: translucent red slab hovering over the
        # affected cells so it reads clearly against whatever floor color
        # is underneath.
        GL = self.gl
        GL.glColor4f(0.86, 0.24, 0.20, 0.55)
        for (x, y) in wh.blocked_aisles:
            self._draw_box(x, y, 1.0, 1.0, 0.08, (0.86, 0.24, 0.20), y0=0.06)

    def _draw_robot(self, position: Coordinate, color: Tuple[float, float, float],
                    status_color: Tuple[float, float, float]) -> None:
        GL = self.gl
        GLU = self.glu
        x, z = position
        cx, cz = x + 0.5, z + 0.5

        GL.glPushMatrix()
        GL.glTranslatef(cx, ROBOT_HEIGHT, cz)
        GL.glColor3f(*color)
        GLU.gluSphere(self._quadric, ROBOT_RADIUS, 16, 16)
        GL.glPopMatrix()

        # Status-colored ring at the robot's base (drawn as a flattened
        # disc just above the floor) - the 3D equivalent of the colored
        # outline ring used in the 2D renderer/dashboard.
        GL.glPushMatrix()
        GL.glTranslatef(cx, 0.06, cz)
        GL.glRotatef(90, 1, 0, 0)
        GL.glColor3f(*status_color)
        GLU.gluDisk(self._quadric, ROBOT_RADIUS * 0.9, ROBOT_RADIUS * 1.15, 16, 1)
        GL.glPopMatrix()

    def _draw_scene(self) -> None:
        self._draw_floor()
        for rid, robot in self.simulator.robots.items():
            color = self._robot_colors.get(rid, (0.8, 0.8, 0.8))
            status_color = STATUS_COLORS.get(robot.status.value, (1.0, 1.0, 1.0))
            self._draw_robot(robot.position, color, status_color)

    # -- 2D text overlay (HUD) ----------------------------------------------
    def _draw_hud(self) -> None:
        """Render status text as a 2D orthographic overlay on top of the
        3D scene - the standard technique for HUD text in an OpenGL app:
        render text to a pygame Surface, upload it as a texture, draw a
        textured quad in an orthographic (non-3D) projection pass with
        depth testing and lighting temporarily disabled.
        """
        GL = self.gl
        pygame = self.pygame

        lines = [
            f"t={self.simulator.sim_time:.1f}s  tick={self.simulator.tick_count}  "
            f"speed={self.speed:.2f}x  {'PAUSED' if self.paused else 'RUNNING'}",
            "  ".join(f"{rid}:{r.status.value}" for rid, r in self.simulator.robots.items()),
            "[SPACE]=pause/resume [R]=reset [C]=reset camera [arrows]=orbit [+/-]=zoom",
        ]

        surface = pygame.Surface((self.width, 70), pygame.SRCALPHA)
        surface.fill((20, 22, 26, 200))
        for i, line in enumerate(lines):
            text_surface = self.font.render(line, True, (230, 230, 230))
            surface.blit(text_surface, (8, 6 + i * 20))

        texture_data = pygame.image.tostring(surface, "RGBA", True)
        tex_w, tex_h = surface.get_size()

        tex_id = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, tex_id)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGBA, tex_w, tex_h, 0,
                         GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, texture_data)

        GL.glDisable(GL.GL_LIGHTING)
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_TEXTURE_2D)

        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glPushMatrix()
        GL.glLoadIdentity()
        GL.glOrtho(0, self.width, self.height, 0, -1, 1)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glPushMatrix()
        GL.glLoadIdentity()

        y0 = self.height - tex_h
        GL.glColor4f(1.0, 1.0, 1.0, 1.0)
        GL.glBegin(GL.GL_QUADS)
        GL.glTexCoord2f(0, 1); GL.glVertex2f(0, y0)
        GL.glTexCoord2f(1, 1); GL.glVertex2f(tex_w, y0)
        GL.glTexCoord2f(1, 0); GL.glVertex2f(tex_w, y0 + tex_h)
        GL.glTexCoord2f(0, 0); GL.glVertex2f(0, y0 + tex_h)
        GL.glEnd()

        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glPopMatrix()
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glPopMatrix()

        GL.glDisable(GL.GL_TEXTURE_2D)
        GL.glDeleteTextures([tex_id])
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_LIGHTING)

    # -- main loop ------------------------------------------------------------
    def render_frame(self) -> None:
        GL = self.gl
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        self._apply_camera()
        self._draw_scene()
        self._draw_hud()
        self.pygame.display.flip()

    def run(self, target_fps: int = 60) -> None:
        running = True
        while running:
            running = self.handle_events()
            if not running:
                break
            if not self.paused:
                self.simulator.step()
            self.render_frame()
            self.clock.tick(target_fps)

        self.pygame.quit()
