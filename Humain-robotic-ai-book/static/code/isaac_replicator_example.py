"""
Isaac Sim Replicator API Example: Synthetic Data Generation for Cup Detection

This script demonstrates the complete workflow for generating synthetic training data
using NVIDIA Isaac Sim's Replicator API. It creates a scene with a table and cups,
applies domain randomization (lighting, placement, camera pose), and captures 100
labeled images suitable for training object detection models.

Prerequisites:
- NVIDIA Isaac Sim 2023.1.1 or later
- NVIDIA GPU with RTX capabilities (RTX 2060 or higher recommended)
- Python 3.10+ (Isaac Sim Python environment)

Usage:
    1. Launch Isaac Sim
    2. Open the Script Editor: Window → Script Editor
    3. Paste this script and click "Run"
    4. Data will be saved to: <IsaacSimUserData>/synthetic_data/cup_detection/

Output:
    ./synthetic_data/cup_detection/
    ├── rgb_0000.png, rgb_0001.png, ..., rgb_0099.png (RGB images)
    ├── semantic_segmentation_0000.png, ... (pixel-perfect labels)
    ├── depth_0000.npy, ... (depth maps in meters)
    ├── instance_segmentation_0000.png, ... (unique object IDs)
    └── metadata.json (camera intrinsics, object poses)

Author: Module 3 - Isaac Perception & Navigation
License: Educational use, based on NVIDIA Isaac Sim examples
"""

import omni.replicator.core as rep
import numpy as np
from pxr import Gf

# =============================================================================
# STEP 1: Scene Setup
# =============================================================================

def create_scene():
    """
    Create a simple scene with a table, ground plane, and lighting.

    Returns:
        tuple: (table_prim, ground_prim) USD prims for reference
    """
    # Create ground plane (10m x 10m gray floor)
    ground = rep.create.plane(
        position=(0, 0, 0),
        scale=(10, 10, 1),
        semantics=[('class', 'floor')]
    )

    # Create table (1m x 1m x 0.8m high)
    table = rep.create.cube(
        position=(0, 0, 0.4),  # 0.8m high table (cube center at 0.4m)
        scale=(1.0, 1.0, 0.8),
        semantics=[('class', 'table')]
    )

    # Apply wood material to table
    with table:
        rep.randomizer.materials(
            materials=rep.get.materials(semantics=[('material', 'wood')]),
            input_prims=table
        )

    # Create directional light (simulates sun)
    distant_light = rep.create.light(
        light_type="distant",
        rotation=(45, 0, 0),  # 45-degree angle
        intensity=3000.0,
        color=(1.0, 1.0, 0.95),  # Slightly warm white
        temperature=5500  # Daylight color temperature (Kelvin)
    )

    print("✓ Scene created: ground plane, table, lighting")
    return table, ground

# =============================================================================
# STEP 2: Object Creation (Cups)
# =============================================================================

def create_cups(num_cups=3):
    """
    Create multiple cup models with different shapes and materials.

    Args:
        num_cups (int): Number of cup variations to create

    Returns:
        list: List of cup USD prims
    """
    cups = []

    for i in range(num_cups):
        # Create cylinder (cup shape): radius=0.04m, height=0.12m
        cup = rep.create.cylinder(
            position=(0, 0, 1.0),  # Initial position above table
            scale=(0.04, 0.04, 0.12),  # Realistic cup dimensions
            semantics=[('class', 'cup'), ('instance', f'cup_{i}')]
        )
        cups.append(cup)

        # Randomize cup material (ceramic, plastic, metal)
        with cup:
            rep.randomizer.color(
                colors=rep.distribution.choice([
                    (1.0, 0.0, 0.0),   # Red (ceramic mug)
                    (0.0, 0.5, 1.0),   # Blue (plastic cup)
                    (0.8, 0.8, 0.8),   # White (porcelain mug)
                    (0.3, 0.3, 0.3)    # Gray (metal tumbler)
                ])
            )

    print(f"✓ Created {num_cups} cup models with varied colors")
    return cups

# =============================================================================
# STEP 3: Camera Configuration
# =============================================================================

def setup_camera():
    """
    Create an RGB camera with semantic segmentation output.

    Returns:
        tuple: (camera, render_product) for use in data capture
    """
    # Create camera at initial position (2m away, 1.5m high)
    camera = rep.create.camera(
        position=(2.0, 2.0, 1.5),
        look_at=(0, 0, 0.8),  # Look at table surface
        focus_distance=2.5,
        f_stop=1.8,  # Shallow depth of field
        focal_length=24.0  # Wide-angle lens (24mm equivalent)
    )

    # Create render product (enables sensor outputs)
    render_product = rep.create.render_product(camera, resolution=(1024, 1024))

    print("✓ Camera configured: 1024x1024 RGB + depth + segmentation")
    return camera, render_product

# =============================================================================
# STEP 4: Data Writer Configuration
# =============================================================================

def setup_writer(render_product, output_dir="./synthetic_data/cup_detection"):
    """
    Configure data writer to save RGB, depth, and segmentation outputs.

    Args:
        render_product: Render product from camera
        output_dir (str): Directory to save generated data

    Returns:
        BasicWriter: Configured writer object
    """
    writer = rep.WriterRegistry.get("BasicWriter")

    writer.initialize(
        output_dir=output_dir,
        rgb=True,                       # Save RGB images as PNG
        depth=True,                     # Save depth maps as NumPy arrays (.npy)
        semantic_segmentation=True,     # Save per-pixel class labels
        instance_segmentation=True,     # Save per-pixel object IDs
        bounding_box_2d_tight=True,     # Save 2D bounding boxes (COCO format)
        camera_params=True              # Save camera intrinsics (focal length, etc.)
    )

    writer.attach([render_product])

    print(f"✓ Data writer configured: output → {output_dir}")
    return writer

# =============================================================================
# STEP 5: Domain Randomization
# =============================================================================

def randomize_scene(cups, camera, table):
    """
    Apply domain randomization to increase dataset diversity.

    Randomizes:
    - Lighting (intensity, color temperature)
    - Cup placement (position, rotation on table)
    - Camera viewpoint (orbit around table)

    Args:
        cups (list): List of cup prims
        camera: Camera prim
        table: Table prim (for reference bounds)
    """
    # Randomize lighting intensity (0.5x to 3.0x)
    rep.randomizer.lighting(
        intensity=rep.distribution.uniform(1500.0, 9000.0),
        color_temperature=rep.distribution.uniform(2500, 7000)  # Warm to cool white
    )

    # Randomly select one cup to be visible (others hidden)
    active_cup_idx = rep.distribution.choice(range(len(cups)))

    for i, cup in enumerate(cups):
        if i == active_cup_idx:
            # Place active cup on table surface (randomize XY position, rotation)
            with cup:
                rep.modify.pose(
                    position=rep.distribution.uniform(
                        (-0.3, -0.3, 1.0),  # Table bounds (1m x 1m, Z=1.0 is surface)
                        (0.3, 0.3, 1.0)
                    ),
                    rotation=rep.distribution.uniform(
                        (0, 0, 0),
                        (0, 0, 360)  # Random yaw rotation
                    )
                )
                rep.modify.visibility(visible=True)
        else:
            # Hide inactive cups
            with cup:
                rep.modify.visibility(visible=False)

    # Randomize camera position (orbit around table)
    with camera:
        # Spherical coordinates: radius=2-3m, azimuth=0-360°, elevation=30-70°
        azimuth = rep.distribution.uniform(0, 360)
        elevation = rep.distribution.uniform(30, 70)
        radius = rep.distribution.uniform(2.0, 3.0)

        # Convert spherical to Cartesian coordinates
        x = radius * np.cos(np.radians(azimuth)) * np.cos(np.radians(elevation))
        y = radius * np.sin(np.radians(azimuth)) * np.cos(np.radians(elevation))
        z = radius * np.sin(np.radians(elevation))

        rep.modify.pose(
            position=(x, y, z),
            look_at=(0, 0, 0.8)  # Always point at table surface
        )

    # Small random rotation to table orientation (±5 degrees) for variety
    with table:
        rep.modify.pose(
            rotation=rep.distribution.uniform(
                (-5, -5, 0),
                (5, 5, 0)
            )
        )

# =============================================================================
# MAIN EXECUTION
# =============================================================================

def main():
    """
    Main execution function: orchestrates scene setup, randomization, and data capture.
    """
    print("=" * 70)
    print("Isaac Sim Replicator: Synthetic Data Generation Pipeline")
    print("=" * 70)

    # Step 1: Create scene
    print("\n[1/6] Creating scene...")
    table, ground = create_scene()

    # Step 2: Create objects (cups)
    print("\n[2/6] Creating objects...")
    cups = create_cups(num_cups=5)  # Create 5 cup variations

    # Step 3: Setup camera
    print("\n[3/6] Configuring camera...")
    camera, render_product = setup_camera()

    # Step 4: Setup data writer
    print("\n[4/6] Configuring data writer...")
    writer = setup_writer(render_product)

    # Step 5: Register randomization function
    print("\n[5/6] Registering randomizers...")
    rep.randomizer.register(lambda: randomize_scene(cups, camera, table))

    # Step 6: Trigger data capture (100 frames with randomization)
    print("\n[6/6] Starting data capture (100 frames)...")
    with rep.trigger.on_frame(num_frames=100):
        rep.randomizer.randomize_scene()

    # Run the orchestrator (executes capture loop)
    print("\nRunning simulation...")
    rep.orchestrator.run()

    print("\n" + "=" * 70)
    print("✓ Data generation complete!")
    print("  - 100 RGB images")
    print("  - 100 depth maps")
    print("  - 100 semantic segmentation masks")
    print("  - 100 instance segmentation masks")
    print("  - Metadata with camera parameters and object poses")
    print("=" * 70)
    print("\nNext steps:")
    print("  1. Review generated data in: ./synthetic_data/cup_detection/")
    print("  2. Train object detection model (YOLO, Faster R-CNN)")
    print("  3. Evaluate on real-world test images")
    print("  4. Fine-tune with small amount of real data if needed")
    print("=" * 70)

if __name__ == "__main__":
    main()


# =============================================================================
# EDUCATIONAL NOTES
# =============================================================================
"""
Key Concepts Demonstrated:

1. **Scene Setup (USD Prims)**:
   - Ground plane, table, and objects created using rep.create API
   - Semantic labels assigned via 'semantics' parameter (class, instance)

2. **Camera Configuration**:
   - Position, look_at, focal length control viewpoint and FOV
   - Render product enables multiple output types (RGB, depth, segmentation)

3. **Domain Randomization**:
   - Lighting: Randomize intensity and color temperature to simulate day/night, indoor/outdoor
   - Placement: Randomize object XY position and rotation on table surface
   - Camera Pose: Randomize viewpoint using spherical coordinates (azimuth, elevation, radius)

4. **Data Capture**:
   - rep.trigger.on_frame() executes randomization for N frames
   - rep.orchestrator.run() starts the simulation loop
   - BasicWriter automatically saves outputs to disk

5. **Scalability**:
   - Change num_frames=100 to num_frames=10000 for larger datasets
   - Run on multiple GPUs or cloud instances for parallel generation
   - Modify randomization ranges to increase/decrease diversity

Limitations (Intentional for Educational Clarity):
- Simple cup shapes (cylinders) instead of complex 3D models
- Single object per frame (simplifies labeling for beginners)
- No distractor objects (can be added with rep.randomizer.scatter)
- No texture randomization (materials are solid colors)

Extensions for Advanced Users:
- Load custom USD assets: rep.create.from_usd("/Path/To/Cup.usd")
- Add distractor objects: rep.randomizer.scatter() for random background clutter
- Randomize textures: rep.randomizer.texture() with material libraries
- Multi-object scenes: Place multiple cups simultaneously
- Export to ROS 2 bags: Use ROSWriter instead of BasicWriter

References:
- NVIDIA Isaac Sim Replicator Docs: https://docs.nvidia.com/isaac/doc/isaac_sim/replicator.html
- [Tobin2017] Domain Randomization for Sim-to-Real Transfer
- [Tremblay2018] Training Deep Networks with Synthetic Data
"""
