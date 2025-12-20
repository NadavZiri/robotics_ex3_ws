#!/usr/bin/env python3
"""
ARGoS Arena Designer - Visual tool for creating ARGoS arena configurations
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import math

class ArenaDesigner:
    def __init__(self, root):
        self.root = root
        self.root.title("ARGoS Arena Designer")
        
        # Arena parameters
        self.arena_size = 8.0  # meters
        self.canvas_size = 600
        self.scale = self.canvas_size / self.arena_size
        self.grid_size = 0.1  # Grid snap size in meters
        self.grid_snap = self.grid_size * self.scale  # Grid snap size in pixels
        
        # Objects in arena
        self.obstacles = []
        self.robot_pos = None
        self.target_pos = None
        
        # Drawing state
        self.drawing_obstacle = False
        self.start_x = None
        self.start_y = None
        self.current_rect = None
        self.snap_to_grid = tk.BooleanVar(value=True)
        self.controller_type = tk.StringVar(value="controller_bug1")
        
        self.setup_ui()
        
    def setup_ui(self):
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Left panel - Canvas
        canvas_frame = ttk.LabelFrame(main_frame, text="Arena (click and drag to create obstacles)", padding="5")
        canvas_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        
        self.canvas = tk.Canvas(canvas_frame, width=self.canvas_size, height=self.canvas_size, bg='white')
        self.canvas.pack()
        
        # Draw grid
        self.draw_grid()
        
        # Right panel - Controls
        control_frame = ttk.Frame(main_frame, padding="5")
        control_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        
        # Mode selection
        mode_frame = ttk.LabelFrame(control_frame, text="Mode", padding="5")
        mode_frame.pack(fill=tk.X, pady=5)
        
        self.mode = tk.StringVar(value="obstacle")
        ttk.Radiobutton(mode_frame, text="Add Obstacle", variable=self.mode, value="obstacle").pack(anchor=tk.W)
        ttk.Radiobutton(mode_frame, text="Place Robot", variable=self.mode, value="robot").pack(anchor=tk.W)
        ttk.Radiobutton(mode_frame, text="Place Target", variable=self.mode, value="target").pack(anchor=tk.W)
        
        # Grid snap control
        ttk.Separator(mode_frame, orient='horizontal').pack(fill=tk.X, pady=5)
        ttk.Checkbutton(mode_frame, text="Snap to Grid (0.1m)", variable=self.snap_to_grid).pack(anchor=tk.W)
        
        # Controller selection
        controller_frame = ttk.LabelFrame(control_frame, text="Controller", padding="5")
        controller_frame.pack(fill=tk.X, pady=5)
        ttk.Radiobutton(controller_frame, text="Bug 1", variable=self.controller_type, value="controller_bug1").pack(anchor=tk.W)
        ttk.Radiobutton(controller_frame, text="Bug 2", variable=self.controller_type, value="controller_bug2").pack(anchor=tk.W)
        
        # Object list
        list_frame = ttk.LabelFrame(control_frame, text="Objects", padding="5")
        list_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.object_listbox = tk.Listbox(list_frame, height=15)
        self.object_listbox.pack(fill=tk.BOTH, expand=True)
        
        ttk.Button(list_frame, text="Delete Selected", command=self.delete_selected).pack(fill=tk.X, pady=2)
        ttk.Button(list_frame, text="Clear All", command=self.clear_all).pack(fill=tk.X, pady=2)
        
        # Export controls
        export_frame = ttk.LabelFrame(control_frame, text="Export", padding="5")
        export_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(export_frame, text="Generate ARGoS Config", command=self.generate_argos).pack(fill=tk.X, pady=2)
        
        # Canvas bindings
        self.canvas.bind('<Button-1>', self.on_canvas_click)
        self.canvas.bind('<B1-Motion>', self.on_canvas_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_canvas_release)
        
        self.update_object_list()
        
    def draw_grid(self):
        # Draw grid lines every 0.5 meters
        grid_spacing = 0.5 * self.scale
        
        for i in range(int(self.arena_size / 0.5) + 1):
            x = i * grid_spacing
            self.canvas.create_line(x, 0, x, self.canvas_size, fill='lightgray', tags='grid')
            self.canvas.create_line(0, x, self.canvas_size, x, fill='lightgray', tags='grid')
        
        # Draw center lines
        center = self.canvas_size / 2
        self.canvas.create_line(center, 0, center, self.canvas_size, fill='gray', tags='grid')
        self.canvas.create_line(0, center, self.canvas_size, center, fill='gray', tags='grid')
    def snap_to_grid_coord(self, coord):
        """Snap a coordinate to the grid"""
        if self.snap_to_grid.get():
            return round(coord / self.grid_snap) * self.grid_snap
        return coord
    
    def canvas_to_arena(self, canvas_x, canvas_y):
        """Convert canvas coordinates to arena coordinates"""
        arena_x = (canvas_x - self.canvas_size/2) / self.scale
        arena_y = -(canvas_y - self.canvas_size/2) / self.scale  # Y is inverted
        return arena_x, arena_y
    
    def arena_to_canvas(self, arena_x, arena_y):
        """Convert arena coordinates to canvas coordinates"""
        canvas_x = arena_x * self.scale + self.canvas_size / 2
        canvas_y = -arena_y * self.scale + self.canvas_size / 2
        return canvas_x, canvas_y

    def on_canvas_click(self, event):
        mode = self.mode.get()
        if mode == "robot":
            x = self.snap_to_grid_coord(event.x)
            y = self.snap_to_grid_coord(event.y)
            self.place_robot(x, y)
        elif mode == "target":
            x = self.snap_to_grid_coord(event.x)
            y = self.snap_to_grid_coord(event.y)
            self.place_target(x, y)
        elif mode == "obstacle":
            self.drawing_obstacle = True
            self.start_x = self.snap_to_grid_coord(event.x)
            self.start_y = self.snap_to_grid_coord(event.y)
    def on_canvas_drag(self, event):
        if self.drawing_obstacle and self.mode.get() == "obstacle":
            if self.current_rect:
                self.canvas.delete(self.current_rect)
            end_x = self.snap_to_grid_coord(event.x)
            end_y = self.snap_to_grid_coord(event.y)
            x1, y1 = min(self.start_x, end_x), min(self.start_y, end_y)
            x2, y2 = max(self.start_x, end_x), max(self.start_y, end_y)
            self.current_rect = self.canvas.create_rectangle(x1, y1, x2, y2,
                                                            fill='gray', outline='black',
                                                            tags='temp')
    def on_canvas_release(self, event):
        if self.drawing_obstacle and self.mode.get() == "obstacle":
            self.drawing_obstacle = False
            if self.current_rect:
                self.canvas.delete(self.current_rect)
                self.current_rect = None
            end_x = self.snap_to_grid_coord(event.x)
            end_y = self.snap_to_grid_coord(event.y)
            # Minimum size check (5 pixels)
            if abs(end_x - self.start_x) > 5 and abs(end_y - self.start_y) > 5:
                self.add_obstacle(self.start_x, self.start_y, end_x, end_y)
            
    def add_obstacle(self, x1, y1, x2, y2):
        # Calculate center and size in arena coordinates
        cx_canvas = (x1 + x2) / 2
        cy_canvas = (y1 + y2) / 2
        width_canvas = abs(x2 - x1)
        height_canvas = abs(y2 - y1)
        
        cx, cy = self.canvas_to_arena(cx_canvas, cy_canvas)
        width = width_canvas / self.scale
        height = height_canvas / self.scale
        
        # Create rectangle on canvas
        rect_id = self.canvas.create_rectangle(x1, y1, x2, y2, 
                                               fill='gray', outline='black', 
                                               tags='obstacle')
        
        obstacle = {
            'type': 'obstacle',
            'canvas_id': rect_id,
            'position': (cx, cy),
            'size': (width, height)
        }
        
        self.obstacles.append(obstacle)
        self.update_object_list()
        
    def place_robot(self, canvas_x, canvas_y):
        # Remove old robot
        if self.robot_pos:
            self.canvas.delete(self.robot_pos['canvas_id'])
        
        arena_x, arena_y = self.canvas_to_arena(canvas_x, canvas_y)
        
        # Draw robot (circle)
        r = 10
        circle_id = self.canvas.create_oval(canvas_x-r, canvas_y-r, canvas_x+r, canvas_y+r,
                                           fill='blue', outline='darkblue', tags='robot')
        
        self.robot_pos = {
            'type': 'robot',
            'canvas_id': circle_id,
            'position': (arena_x, arena_y)
        }
        
        self.update_object_list()
        
    def place_target(self, canvas_x, canvas_y):
        # Remove old target
        if self.target_pos:
            self.canvas.delete(self.target_pos['canvas_id'])
        
        arena_x, arena_y = self.canvas_to_arena(canvas_x, canvas_y)
        
        # Draw target (red circle)
        r = 8
        circle_id = self.canvas.create_oval(canvas_x-r, canvas_y-r, canvas_x+r, canvas_y+r,
                                           fill='red', outline='darkred', tags='target')
        
        self.target_pos = {
            'type': 'target',
            'canvas_id': circle_id,
            'position': (arena_x, arena_y)
        }
        
        self.update_object_list()
        
    def update_object_list(self):
        self.object_listbox.delete(0, tk.END)
        
        if self.robot_pos:
            x, y = self.robot_pos['position']
            self.object_listbox.insert(tk.END, f"Robot at ({x:.2f}, {y:.2f})")
            
        if self.target_pos:
            x, y = self.target_pos['position']
            self.object_listbox.insert(tk.END, f"Target at ({x:.2f}, {y:.2f})")
            
        for i, obs in enumerate(self.obstacles):
            x, y = obs['position']
            w, h = obs['size']
            self.object_listbox.insert(tk.END, f"Obstacle{i+1}: {w:.2f}x{h:.2f} at ({x:.2f}, {y:.2f})")
            
    def delete_selected(self):
        selection = self.object_listbox.curselection()
        if not selection:
            return
            
        idx = selection[0]
        
        # Determine what to delete
        offset = 0
        if self.robot_pos:
            if idx == offset:
                self.canvas.delete(self.robot_pos['canvas_id'])
                self.robot_pos = None
                self.update_object_list()
                return
            offset += 1
            
        if self.target_pos:
            if idx == offset:
                self.canvas.delete(self.target_pos['canvas_id'])
                self.target_pos = None
                self.update_object_list()
                return
            offset += 1
            
        # Must be an obstacle
        obs_idx = idx - offset
        if 0 <= obs_idx < len(self.obstacles):
            obs = self.obstacles[obs_idx]
            self.canvas.delete(obs['canvas_id'])
            self.obstacles.pop(obs_idx)
            self.update_object_list()
            
    def clear_all(self):
        if messagebox.askyesno("Clear All", "Are you sure you want to clear all objects?"):
            self.canvas.delete('obstacle', 'robot', 'target')
            self.obstacles = []
            self.robot_pos = None
            self.target_pos = None
            self.update_object_list()
            
    def generate_argos(self):
        if not self.robot_pos:
            messagebox.showwarning("Missing Robot", "Please place a robot in the arena.")
            return
            
        if not self.target_pos:
            messagebox.showwarning("Missing Target", "Please place a target in the arena.")
            return
        
        # Ask for filename
        filename = filedialog.asksaveasfilename(
            defaultextension=".argos",
            filetypes=[("ARGoS files", "*.argos"), ("All files", "*.*")],
            initialdir="./experiments"
        )
        
        if not filename:
            return
            
        # Generate XML
        xml = self.create_argos_xml()
        
        with open(filename, 'w') as f:
            f.write(xml)
            
        messagebox.showinfo("Success", f"Arena configuration saved to:\n{filename}")
        
    def create_argos_xml(self):
        robot_x, robot_y = self.robot_pos['position']
        target_x, target_y = self.target_pos['position']
        
        # Calculate wall positions based on arena size
        half_size = self.arena_size / 2
        
        xml = f'''<?xml version="1.0" ?>
<argos-configuration>

    <!-- ************************* -->
    <!-- * General configuration * -->
    <!-- ************************* -->
    <framework>
        <system threads="0" />
        <experiment length="0" ticks_per_second="10" random_seed="0" />
    </framework>
  
    <!-- *************** -->
    <!-- * Controllers * -->
    <!-- *************** -->
    <controllers>
        <!-- bug1 controller -->
        <controller_bug1 library="build/lib/libpipuck_controllers"
                                         id="controller_bug1">
            <actuators>
                <pipuck_differential_drive implementation="default" />
                <pipuck_leds implementation="color" medium="leds" />
            </actuators>
            <sensors>
                <colored_blob_omnidirectional_camera implementation="rot_z_only" medium="leds" show_rays="true" />
                <pipuck_rangefinders implementation="default" />
                <pipuck_system implementation="default" />
                <positioning implementation="default" />
            </sensors>
            <params>
                <target_position x="{target_x:.2f}" y="{target_y:.2f}" />
            </params>
        </controller_bug1>
        <!-- bug2 controller -->
        <controller_bug2 library="build/lib/libpipuck_controllers"
                                         id="controller_bug2">
            <actuators>
                <pipuck_differential_drive implementation="default" />
                <pipuck_leds implementation="color" medium="leds" />
            </actuators>
            <sensors>
                <colored_blob_omnidirectional_camera implementation="rot_z_only" medium="leds" show_rays="true" />
                <pipuck_rangefinders implementation="default" />
                <pipuck_system implementation="default" />
                <positioning implementation="default" />
            </sensors>
            <params>
                <target_position x="{target_x:.2f}" y="{target_y:.2f}" />
            </params>
        </controller_bug2>
    </controllers>

    <!-- ****************** -->
    <!-- * Loop functions * -->
    <!-- ****************** -->
    <loop_functions library="build/lib/libloop_functions"
                                    label="trajectory_loop_functions" />

    <!-- *********************** -->
    <!-- * Arena configuration * -->
    <!-- *********************** -->
    <arena size="{self.arena_size}, {self.arena_size}, 1" positional_index="grid">
        <!-- walls -->
        <box id="wall_north" size="{self.arena_size},0.1,0.5" movable="false"><body position="0,{half_size},0" orientation="0,0,0"/></box>
        <box id="wall_south" size="{self.arena_size},0.1,0.5" movable="false"><body position="0,{-half_size},0" orientation="0,0,0"/></box>
        <box id="wall_east" size="0.1,{self.arena_size},0.5" movable="false"><body position="{half_size},0,0" orientation="0,0,0"/></box>
        <box id="wall_west" size="0.1,{self.arena_size},0.5" movable="false"><body position="{-half_size},0,0" orientation="0,0,0"/></box>

        <!-- obstacles -->
'''
        
        # Add obstacles
        for i, obs in enumerate(self.obstacles):
            x, y = obs['position']
            w, h = obs['size']
            xml += f'    <box id="obstacle{i+1}" size="{w:.2f},{h:.2f},0.5" movable="false"> <body position="{x:.2f},{y:.2f},0" orientation="0,0,0"/> </box>\n'
        
        xml += f'''    <!-- target -->
        <light id="light_target"
                             position="{target_x:.2f},{target_y:.2f}, 0"
                             orientation="0,0,0"
                             color="cyan"
                             intensity="5.0"
                             medium="leds" />

        <!-- robot -->
        <pipuck id="pipuck0" led_medium="leds">
            <body position="{robot_x:.2f},{robot_y:.2f},0"/>
            <controller config="{self.controller_type.get()}"/>
        </pipuck>

    </arena>

    <!-- ******************* -->
    <!-- * Physics engines * -->
    <!-- ******************* -->
    <physics_engines>
        <dynamics2d id="dyn2d" />
    </physics_engines>

    <!-- ********* -->
    <!-- * Media * -->
    <!-- ********* -->
    <media>
        <led id="leds"/>
    </media>

    <!-- ****************** -->
    <!-- * Visualization * -->
    <!-- ****************** -->
    <visualization>
        <qt-opengl show_boundary="false">
            <user_functions library="build/lib/libloop_functions"
                                            label="trajectory_qtuser_functions"/>
            <camera>
                <placements>  
                    <placement index="0" position="0,0,30" look_at="0,0,0" up="0,1,0" lens_focal_length="100" />
                </placements>
            </camera>
        </qt-opengl>
    </visualization>

</argos-configuration>
'''
        return xml

def main():
    root = tk.Tk()
    app = ArenaDesigner(root)
    root.mainloop()

if __name__ == "__main__":
    main()
