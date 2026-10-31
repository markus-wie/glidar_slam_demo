"""Stanford Bunny point cloud with axis-varying density, then voxelized.

Samples the Stanford Bunny mesh, keeps points with a probability that depends
on one coordinate axis, and visualizes that uneven cloud. A second view shows
the same cloud after voxel downsampling, which caps the local density.

Run from the repository root:

    .venv/bin/python visualize_pcl.py

Close each Open3D window to continue. Edit the globals below to retune the demo.
"""

from __future__ import annotations

import sys

import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit(
        "open3d is not installed in this interpreter.\n"
        "From the repository root, run:\n"
        "  .venv/bin/pip install -r requirements.txt"
    )


# --- sampling -----------------------------------------------------------------
# Axis whose coordinate controls keep-probability. "x", "y", or "z".
AXIS = "y"
# Flip the gradient so density increases toward the negative end of AXIS.
INVERT_AXIS = False
# Uniform surface samples drawn before rejection. Higher packs the dense end.
INITIAL_SAMPLE_COUNT = 200_000
# Keep-probability at the low and high ends of the (optionally inverted) axis.
# The low end must stay small or the whole bunny looks uniformly filled.
DENSITY_AT_MIN = 0.05
DENSITY_AT_MAX = 0.8
# "exponential" multiplies density along the axis, so spacing changes a lot.
# "power" blends linearly after raising the axis coordinate to DENSITY_GAMMA.
DENSITY_CURVE = "exponential"
# 1.0 follows the axis directly. >1 keeps the lower body sparse and packs the top.
DENSITY_GAMMA = 3.0
RNG_SEED = 0
# Empty string downloads Open3D's copy of the Stanford Bunny mesh.
BUNNY_MESH_PATH = ""

# --- voxelization -------------------------------------------------------------
# Absolute voxel edge length. None selects the size from VOXEL_SIZE_FRACTION.
VOXEL_SIZE = None
# Used when VOXEL_SIZE is None. Fraction of the cloud's largest bounding-box edge.
# Large enough to swallow the sparse-end gaps created by DENSITY_GAMMA.
VOXEL_SIZE_FRACTION = 0.022

# --- visualization ------------------------------------------------------------
VISUALIZE_UNEVEN = True
VISUALIZE_VOXEL = True
# Optional third window with the voxel cloud shifted beside the uneven cloud.
VISUALIZE_COMPARISON = False
POINT_SIZE = 3
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 800
BACKGROUND_COLOR = (1.0, 1.0, 1.0)
# "axis" colors by COLOR_AXIS. "spacing" colors by neighbor distance.
COLOR_MODE = "axis"
# Coordinate axis used for the color gradient; independent of the density axis.
COLOR_AXIS = "y"
# Shift, as a multiple of the bounding-box width, used in the comparison view.
COMPARISON_GAP = 1.25
SHOW_COORDINATE_FRAME = False

# --- camera -------------------------------------------------------------------
# Open3D uses a target point plus a viewing direction rather than an eye point.
# These values are printed after each window closes, so they can be copied back.
# Set CAMERA_POSITION to a 3-tuple to control the camera eye directly. Leave it
# as None to use CAMERA_LOOKAT, CAMERA_FRONT, CAMERA_UP, and CAMERA_ZOOM.
# CAMERA_POSITION = None
CAMERA_LOOKAT = (0.0, 0.0, 0.0)
# CAMERA_FRONT = (0.0, 0.0, -1.0)
# CAMERA_UP = (0.0, 1.0, 0.0)
CAMERA_ZOOM = 0.7

CAMERA_POSITION = (-0.056255, 0.260176, 0.426071)
CAMERA_FRONT = (-0.099705, 0.304615, 0.947243)
CAMERA_UP = (0.049552, -0.949285, 0.310488)


def _axis_index(axis: str) -> int:
    try:
        return {"x": 0, "y": 1, "z": 2}[axis.lower()]
    except KeyError as exc:
        raise ValueError('AXIS must be "x", "y", or "z"') from exc


def load_bunny_mesh() -> o3d.geometry.TriangleMesh:
    """Load the Stanford Bunny triangle mesh shipped with Open3D, or a local file."""
    if BUNNY_MESH_PATH:
        path = BUNNY_MESH_PATH
    else:
        path = o3d.data.BunnyMesh().path
    mesh = o3d.io.read_triangle_mesh(path)
    if mesh.is_empty() or len(mesh.triangles) == 0:
        raise RuntimeError(f"Could not read a triangle mesh from {path}")
    mesh.compute_vertex_normals()
    return mesh


def density_coordinate(
    points: np.ndarray,
    axis: str = AXIS,
    invert: bool = INVERT_AXIS,
) -> np.ndarray:
    """Map an axis to [0, 1]."""
    coord = points[:, _axis_index(axis)]
    span = float(coord.max() - coord.min())
    if span <= 0.0:
        return np.zeros(len(points), dtype=np.float64)
    normalized = (coord - float(coord.min())) / span
    if invert:
        normalized = 1.0 - normalized
    return normalized


def keep_probability(normalized_axis: np.ndarray) -> np.ndarray:
    if DENSITY_AT_MIN < 0.0 or DENSITY_AT_MAX < 0.0:
        raise ValueError("DENSITY_AT_MIN and DENSITY_AT_MAX must be >= 0")
    if DENSITY_GAMMA <= 0.0:
        raise ValueError("DENSITY_GAMMA must be > 0")
    shaped = np.power(np.clip(normalized_axis, 0.0, 1.0), DENSITY_GAMMA)
    curve = DENSITY_CURVE.lower()
    if curve == "power":
        return DENSITY_AT_MIN + (DENSITY_AT_MAX - DENSITY_AT_MIN) * shaped
    if curve == "exponential":
        if DENSITY_AT_MIN <= 0.0 or DENSITY_AT_MAX <= 0.0:
            raise ValueError("DENSITY_CURVE='exponential' needs densities > 0")
        log_probability = np.log(DENSITY_AT_MIN) + shaped * (
            np.log(DENSITY_AT_MAX) - np.log(DENSITY_AT_MIN)
        )
        return np.exp(log_probability)
    raise ValueError('DENSITY_CURVE must be "exponential" or "power"')


def sample_varying_density(
    mesh: o3d.geometry.TriangleMesh,
) -> o3d.geometry.PointCloud:
    """Sample the surface uniformly, then reject points using the axis density."""
    uniform = mesh.sample_points_uniformly(number_of_points=INITIAL_SAMPLE_COUNT)
    points = np.asarray(uniform.points)
    probability = keep_probability(density_coordinate(points))
    rng = np.random.default_rng(RNG_SEED)
    keep = rng.random(len(points)) < probability
    if not np.any(keep):
        raise RuntimeError(
            "Density rejection kept zero points. Increase DENSITY_AT_MIN or "
            "INITIAL_SAMPLE_COUNT."
        )
    uneven = uniform.select_by_index(np.flatnonzero(keep).tolist())
    paint_cloud(uneven)
    return uneven


def resolve_voxel_size(cloud: o3d.geometry.PointCloud) -> float:
    if VOXEL_SIZE is not None:
        if VOXEL_SIZE <= 0.0:
            raise ValueError("VOXEL_SIZE must be > 0, or None to use the fraction")
        return float(VOXEL_SIZE)
    if VOXEL_SIZE_FRACTION <= 0.0:
        raise ValueError("VOXEL_SIZE_FRACTION must be > 0")
    extent = float(np.max(cloud.get_axis_aligned_bounding_box().get_extent()))
    return VOXEL_SIZE_FRACTION * extent


def voxelize(cloud: o3d.geometry.PointCloud) -> tuple[o3d.geometry.PointCloud, float]:
    """Downsample to at most one point per voxel, which caps local density."""
    voxel_size = resolve_voxel_size(cloud)
    even = cloud.voxel_down_sample(voxel_size)
    if even.is_empty():
        raise RuntimeError(
            "Voxel downsampling removed every point. Decrease VOXEL_SIZE."
        )
    paint_cloud(even)
    return even, voxel_size


def _viridis(t: np.ndarray) -> np.ndarray:
    anchors = np.array(
        [
            [0.267004, 0.004874, 0.329415],
            [0.282327, 0.140926, 0.457517],
            [0.253935, 0.265254, 0.529983],
            [0.206756, 0.371758, 0.553117],
            [0.163625, 0.471133, 0.558148],
            [0.127568, 0.566949, 0.550556],
            [0.134692, 0.658636, 0.517649],
            [0.266941, 0.748751, 0.440573],
            [0.477504, 0.821444, 0.318195],
            [0.741388, 0.873449, 0.149561],
            [0.993248, 0.906157, 0.143936],
        ],
        dtype=np.float64,
    )
    positions = np.linspace(0.0, 1.0, len(anchors))
    colors = np.empty((t.shape[0], 3), dtype=np.float64)
    for channel in range(3):
        colors[:, channel] = np.interp(t, positions, anchors[:, channel])
    return colors


def paint_cloud(cloud: o3d.geometry.PointCloud) -> None:
    points = np.asarray(cloud.points)
    mode = COLOR_MODE.lower()
    if mode == "axis":
        color_t = density_coordinate(points, axis=COLOR_AXIS, invert=False)
    elif mode == "spacing":
        if len(points) < 2:
            color_t = np.zeros(len(points), dtype=np.float64)
        else:
            distances = np.asarray(cloud.compute_nearest_neighbor_distance())
            low, high = np.percentile(distances, [5.0, 95.0])
            if high <= low:
                color_t = np.zeros(len(points), dtype=np.float64)
            else:
                # Small spacing (dense) maps to the bright end of the colormap.
                color_t = 1.0 - np.clip((distances - low) / (high - low), 0.0, 1.0)
    else:
        raise ValueError('COLOR_MODE must be "axis" or "spacing"')
    cloud.colors = o3d.utility.Vector3dVector(_viridis(color_t))


def spacing_stats(cloud: o3d.geometry.PointCloud) -> tuple[float, float]:
    if len(cloud.points) < 2:
        return float("nan"), float("nan")
    distances = np.asarray(cloud.compute_nearest_neighbor_distance())
    return float(np.median(distances)), float(np.std(distances))


def _frame_for(cloud: o3d.geometry.PointCloud) -> o3d.geometry.TriangleMesh:
    extent = float(np.max(cloud.get_axis_aligned_bounding_box().get_extent()))
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25 * extent)
    center = cloud.get_axis_aligned_bounding_box().get_center()
    frame.translate(center - np.array([0.55 * extent, 0.55 * extent, 0.0]))
    return frame


def _camera_parameters(
    visualizer: o3d.visualization.Visualizer,
) -> o3d.camera.PinholeCameraParameters:
    view = visualizer.get_view_control()
    return view.convert_to_pinhole_camera_parameters()


def _apply_camera_parameters(
    visualizer: o3d.visualization.Visualizer,
    parameters: o3d.camera.PinholeCameraParameters | None,
) -> None:
    view = visualizer.get_view_control()
    if parameters is None:
        view.set_lookat(CAMERA_LOOKAT)
        view.set_front(CAMERA_FRONT)
        view.set_up(CAMERA_UP)
        view.set_zoom(CAMERA_ZOOM)
        if CAMERA_POSITION is not None:
            initial = view.convert_to_pinhole_camera_parameters()
            front = np.asarray(CAMERA_FRONT, dtype=np.float64)
            front /= np.linalg.norm(front)
            up = np.asarray(CAMERA_UP, dtype=np.float64)
            up -= front * np.dot(up, front)
            up /= np.linalg.norm(up)
            right = np.cross(front, up)
            rotation = np.vstack((right, up, -front))
            extrinsic = np.array(initial.extrinsic, copy=True)
            extrinsic[:3, :3] = rotation
            extrinsic[:3, 3] = -rotation @ np.asarray(CAMERA_POSITION, dtype=np.float64)
            initial.extrinsic = extrinsic
            view.convert_from_pinhole_camera_parameters(initial, allow_arbitrary=True)
    else:
        view.convert_from_pinhole_camera_parameters(parameters, allow_arbitrary=True)


def _print_camera_parameters(
    parameters: o3d.camera.PinholeCameraParameters,
) -> None:
    rotation = np.asarray(parameters.extrinsic[:3, :3])
    translation = np.asarray(parameters.extrinsic[:3, 3])
    position = -rotation.T @ translation
    front = -rotation.T[:, 2]
    up = rotation.T[:, 1]
    as_tuple = lambda values: tuple(float(value) for value in np.round(values, 6))
    print("Current camera pose:")
    print(f"CAMERA_POSITION = {as_tuple(position)}")
    print(f"CAMERA_FRONT = {as_tuple(front)}")
    print(f"CAMERA_UP = {as_tuple(up)}")
    print("The exact pose is reused for the next visualization window.")


def show_cloud(
    cloud: o3d.geometry.PointCloud,
    window_name: str,
    camera_parameters: o3d.camera.PinholeCameraParameters | None = None,
) -> o3d.camera.PinholeCameraParameters:
    geometries: list[o3d.geometry.Geometry] = [cloud]
    if SHOW_COORDINATE_FRAME:
        geometries.append(_frame_for(cloud))
    visualizer = o3d.visualization.Visualizer()
    if not visualizer.create_window(
        window_name=window_name,
        width=WINDOW_WIDTH,
        height=WINDOW_HEIGHT,
    ):
        raise RuntimeError(
            "Open3D could not open a window. Check that DISPLAY is available."
        )
    for geometry in geometries:
        visualizer.add_geometry(geometry)
    _apply_camera_parameters(visualizer, camera_parameters)
    options = visualizer.get_render_option()
    options.point_size = POINT_SIZE
    options.background_color = np.asarray(BACKGROUND_COLOR, dtype=np.float64)
    print(f"Showing '{window_name}'. Close the window to continue.")
    visualizer.run()
    camera_parameters = _camera_parameters(visualizer)
    _print_camera_parameters(camera_parameters)
    visualizer.destroy_window()
    return camera_parameters


def show_comparison(
    uneven: o3d.geometry.PointCloud,
    even: o3d.geometry.PointCloud,
) -> None:
    shift = float(np.max(uneven.get_axis_aligned_bounding_box().get_extent()))
    even_shifted = o3d.geometry.PointCloud(even)
    even_shifted.translate((COMPARISON_GAP * shift, 0.0, 0.0))
    combined = uneven + even_shifted
    show_cloud(combined, "Comparison: uneven (left) vs voxelized (right)")


def print_summary(
    uneven: o3d.geometry.PointCloud,
    even: o3d.geometry.PointCloud,
    voxel_size: float,
) -> None:
    uneven_median, uneven_std = spacing_stats(uneven)
    even_median, even_std = spacing_stats(even)
    print(
        f"Axis density: {AXIS}  invert={INVERT_AXIS}  "
        f"curve={DENSITY_CURVE}  gamma={DENSITY_GAMMA}"
    )
    print(
        f"Keep probability: {DENSITY_AT_MIN:.3f} at min -> {DENSITY_AT_MAX:.3f} at max"
    )
    print(f"Uneven cloud:   {len(uneven.points):7d} points")
    print(f"Voxel size:     {voxel_size:.6f}")
    print(f"Voxel cloud:    {len(even.points):7d} points")
    print(
        "Nearest-neighbor spacing (median / std): "
        f"uneven {uneven_median:.6f} / {uneven_std:.6f}, "
        f"voxel {even_median:.6f} / {even_std:.6f}"
    )
    print(
        "Voxel downsampling removes extra points in dense cells. "
        "It does not invent points in cells that were already empty."
    )


def main() -> None:
    mesh = load_bunny_mesh()
    uneven = sample_varying_density(mesh)
    even, voxel_size = voxelize(uneven)
    print_summary(uneven, even, voxel_size)

    camera_parameters = None
    if VISUALIZE_UNEVEN:
        camera_parameters = show_cloud(
            uneven, f"Stanford Bunny uneven density along {AXIS}"
        )
    if VISUALIZE_VOXEL:
        show_cloud(
            even,
            f"Voxelized Bunny (voxel size {voxel_size:.4f})",
            camera_parameters,
        )
    if VISUALIZE_COMPARISON:
        show_comparison(uneven, even)


if __name__ == "__main__":
    main()
