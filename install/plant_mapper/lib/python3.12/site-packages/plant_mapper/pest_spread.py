# pest_spread.py
# This script manages the 2D grid representation of pest levels and runs the simulation ticks.
# You can modify the simulation and spread logic inside this file!

# Global 2D list to store pest levels for each cell.
# 0.0 means healthy/no pest, higher values mean higher pest percentage.
pest_grid = []

def initialize_grid(rows, cols):
    """
    Resets and initializes the 2D pest grid with all zeros.
    """
    global pest_grid
    pest_grid = [[0.0 for _ in range(cols)] for _ in range(rows)]

def get_pest_grid():
    """
    Returns the current 2D pest grid as a list of lists of floats.
    """
    global pest_grid
    return pest_grid

def on_plant_scanned(row, col, pest_value):
    """
    Called when a plant is scanned (either automatically or manually).
    Updates the grid cell with the scanned plant's pest level.
    """
    global pest_grid
    if not pest_grid:
        return
        
    rows = len(pest_grid)
    cols = len(pest_grid[0]) if rows > 0 else 0
    
    if 0 <= row < rows and 0 <= col < cols:
        pest_grid[row][col] = float(pest_value)

def tick_simulation(steps, variance, spread_coef):
    """
    Performs the specified number of simulation ticks (steps).
    Write your own spread and decay math here!
    
    Parameters:
    - steps (int): number of simulation steps to run
    - variance (float): the variance parameter (sigma^2) from the slider
    - spread_coef (float): the spread coefficient (%) from the slider
    """
    global pest_grid
    if not pest_grid:
        return
        
    rows = len(pest_grid)
    cols = len(pest_grid[0]) if rows > 0 else 0
    
    import math
    import random

    for step in range(steps):
        # 1. Calculate the background spread (e.g. Gaussian spread) for all cells.
        # This acts as the probability distribution for new infections.
        gaussian_spread = [[0.0 for _ in range(cols)] for _ in range(rows)]
        
        # Find all cells that currently have pest (acting as sources)
        sources = []
        for r in range(rows):
            for c in range(cols):
                if pest_grid[r][c] > 0.01:
                    sources.append((r, c, pest_grid[r][c]))
                    
        # Calculate contribution of each source to other cells
        for r_src, c_src, pest in sources:
            # We cut off the Gaussian contribution at exp(-d^2 / (2 * variance)) >= 0.05
            # d_cutoff = sqrt(-2 * variance * log(0.05))
            r_cutoff = math.sqrt(-2.0 * variance * math.log(0.05))
            
            # Convert cutoff in meters to grid cells (resolution is 0.05 meters)
            resolution = 0.05
            cell_cutoff = int(math.ceil(r_cutoff / resolution))
            
            min_r = max(0, r_src - cell_cutoff)
            max_r = min(rows - 1, r_src + cell_cutoff)
            min_c = max(0, c_src - cell_cutoff)
            max_c = min(cols - 1, c_src + cell_cutoff)
            
            for r_dest in range(min_r, max_r + 1):
                for c_dest in range(min_c, max_c + 1):
                    # Compute distance in physical meters
                    dy = (r_dest - r_src) * resolution
                    dx = (c_dest - c_src) * resolution
                    d2 = dx * dx + dy * dy
                    
                    val = math.exp(-d2 / (2.0 * variance))
                    if val >= 0.05:
                        gaussian_spread[r_dest][c_dest] += pest * val
                        
        # 2. Update each cell based on cellular automata rules.
        next_grid = [[pest_grid[r][c] for c in range(cols)] for r in range(rows)]
        
        for r in range(rows):
            for c in range(cols):
                current_val = pest_grid[r][c]
                
                if current_val > 0.0:
                    # Active pest pixel:
                    # Rule 1: 60% chance to increase by 1%, 40% chance to decrease by 1%
                    if random.random() < 0.60:
                        new_val = current_val + 1.0
                    else:
                        new_val = current_val - 1.0
                    new_val = max(0.0, min(100.0, new_val))
                    
                    # Rule 3: if below 10% pest, 50% chance to disappear
                    if new_val < 10.0 and random.random() < 0.50:
                        next_grid[r][c] = 0.0
                    else:
                        next_grid[r][c] = new_val
                else:
                    # Non-pest pixel:
                    # Rule 2: chance to get infected by the spread
                    g_val = gaussian_spread[r][c]
                    if g_val > 0.01:
                        # Probability = (g_val / 100) * (spread_coef / 100)
                        prob = (g_val / 100.0) * (spread_coef / 100.0)
                        if random.random() < prob:
                            next_grid[r][c] = 100.0 # Set as a new active pest source
                            
        pest_grid = next_grid
