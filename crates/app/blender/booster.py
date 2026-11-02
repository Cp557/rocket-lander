"""Builds the booster model and exports it as binary glTF for the viewer.

    blender -b --factory-startup --python crates/app/blender/booster.py

Frame: Blender Z is the long axis (the exporter turns it into glTF +Y); the origin is the
center of mass, 15 m above the engine plane, matching `sim::Vehicle::nominal()`.

The viewer animates these named empties, so their names and pivots are an interface:
  engines   gimbal pivot at the base center; children are the octaweb and nine bells
  fin_0..3  grid fin hinges on the body wall, local +X pointing out (deployed at rest)
  leg_0..3  leg hinges on the body wall, local +X out, legs stowed along local +Z at rest
"""

import math
from pathlib import Path

import bmesh
import bpy
from mathutils import Matrix, Vector

OUT = Path(__file__).resolve().parent.parent / "src" / "models" / "booster.glb"

# Keep in step with sim::Vehicle::nominal() and the viewer's booster.rs.
RADIUS = 1.85
BASE = -15.0
TOP = 25.0
FIN_Z = 23.0
LEG_HINGE_Z = BASE + 2.0
LEG_REACH = 9.0 - RADIUS  # foot radius minus hinge radius
LEG_DROP = 2.0 + 1.5  # hinge height above base plus foot drop below it
LEG_LENGTH = math.hypot(LEG_REACH, LEG_DROP)
LEG_DEPLOYED = math.pi / 2 + math.atan2(LEG_DROP, LEG_REACH)
ENGINE_RING = 1.2
OCTAWEB_DEPTH = 0.4
BELL_LENGTH = 1.6  # nozzle exit 2.0 m below the base
SEGMENTS = 96


def smoothstep(a, b, x):
    t = min(max((x - a) / (b - a), 0.0), 1.0)
    return t * t * (3 - 2 * t)


def material(name, color, metallic=0.0, roughness=0.5, double_sided=False, color_attribute=None):
    """Principled material; with `color_attribute`, base color comes from that vertex color
    layer, which the glTF exporter then writes as COLOR_0."""
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    if color_attribute:
        node = mat.node_tree.nodes.new("ShaderNodeVertexColor")
        node.layer_name = color_attribute
        mat.node_tree.links.new(node.outputs["Color"], bsdf.inputs["Base Color"])
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    mat.use_backface_culling = not double_sided
    return mat


def link(name, bm, mat, parent=None):
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    mesh.materials.append(mat)
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    obj.parent = parent
    return obj


def empty(name, location, angle=0.0):
    obj = bpy.data.objects.new(name, None)
    obj.location = location
    obj.rotation_euler = (0.0, 0.0, angle)
    bpy.context.collection.objects.link(obj)
    return obj


def lathe(bm, profile, segments=SEGMENTS, inward=False):
    """Surface of revolution about Z through (radius, z) points listed bottom to top.
    Faces point outward (or inward) and shade smooth. Returns the rings of vertices."""
    rings = []
    for r, z in profile:
        rings.append([
            bm.verts.new((r * math.cos(a), r * math.sin(a), z))
            for a in (2 * math.pi * j / segments for j in range(segments))
        ])
    for lower, upper in zip(rings, rings[1:]):
        for j in range(segments):
            k = (j + 1) % segments
            quad = (lower[j], lower[k], upper[k], upper[j])
            face = bm.faces.new(tuple(reversed(quad)) if inward else quad)
            face.smooth = True
    return rings


def cap(bm, ring, up):
    """Flat cap over a ring, with a sharp edge so it shades flat."""
    face = bm.faces.new(ring if up else tuple(reversed(ring)))
    face.smooth = False
    for edge in face.edges:
        edge.smooth = False


def box(bm, center, size, matrix=Matrix.Identity(4)):
    geom = bmesh.ops.create_cube(bm, size=1.0)
    verts = geom["verts"]
    bmesh.ops.scale(bm, vec=Vector(size), verts=verts)
    bmesh.ops.translate(bm, vec=Vector(center), verts=verts)
    bmesh.ops.transform(bm, matrix=matrix, verts=verts)


def body(paint):
    """Main tank: dense rings near the base for the soot gradient; sooty streaks rise
    unevenly around the circumference, as on a recovered booster."""
    bm = bmesh.new()
    zs = [BASE + 0.5 * i for i in range(31)] + [0.0 + 2.0 * i for i in range(1, 11)]
    rings = lathe(bm, [(RADIUS, z) for z in zs])
    cap(bm, rings[0], up=False)
    layer = bm.loops.layers.float_color.new("Col")
    for face in bm.faces:
        for loop in face.loops:
            x, y, z = loop.vert.co
            a = math.atan2(y, x)
            streak = 0.5 + 0.5 * math.sin(7 * a + 1.3) * math.sin(3 * a + 0.4)
            edge = -7.0 + 9.0 * streak
            soot = max(0.85 * smoothstep(edge + 2.5, edge - 3.0, z), smoothstep(-8.0, BASE, z))
            white_ = (0.92, 0.92, 0.90)
            black = (0.12, 0.11, 0.10)
            loop[layer] = (*[w + (b - w) * soot for w, b in zip(white_, black)], 1.0)
    obj = link("body", bm, paint)
    obj.data.color_attributes.active_color_name = "Col"
    return obj


def seams(gray):
    bm = bmesh.new()
    for z in (-1.5, 7.0, 15.5):
        lathe(bm, [(RADIUS + 0.008, z - 0.04), (RADIUS + 0.008, z + 0.04)])
    return link("seams", bm, gray)


def interstage(black, dark):
    """Open-topped black interstage: outer wall, lip, inner wall and the tank dome below."""
    bm = bmesh.new()
    lathe(bm, [(RADIUS + 0.01, 20.0), (RADIUS + 0.01, TOP)])
    lathe(bm, [(RADIUS + 0.01, TOP), (RADIUS - 0.12, TOP)])
    lathe(bm, [(RADIUS - 0.12, 21.0), (RADIUS - 0.12, TOP)], inward=True)
    link("interstage", bm, black)
    bm = bmesh.new()
    inner = RADIUS - 0.12
    dome = [(inner * math.cos(t * math.pi / 2), 21.0 + 0.9 * math.sin(t * math.pi / 2)) for t in (0.0, 0.3, 0.6, 0.85)]
    rings = lathe(bm, dome)
    cap(bm, rings[-1], up=True)
    link("tank_dome", bm, dark)


def raceway_and_thrusters(white, black):
    bm = bmesh.new()
    turn = Matrix.Rotation(math.pi / 4, 4, "Z")
    box(bm, (RADIUS + 0.12, 0.0, 3.0), (0.3, 0.4, 32.0), turn)
    link("raceway", bm, white)
    bm = bmesh.new()
    for side in (1.0, -1.0):
        box(bm, (side * (RADIUS + 0.2), 0.0, 18.0), (0.45, 0.8, 0.9))
    link("thruster_pods", bm, black)


def grid_fin(index, titanium, dark):
    """A lattice of thin walls, horizontal and sticking straight out when deployed."""
    angle = index * math.pi / 2
    hinge = empty(f"fin_{index}", (RADIUS * math.cos(angle), RADIUS * math.sin(angle), FIN_Z), angle)
    bm = bmesh.new()
    box(bm, (0.12, 0.0, 0.0), (0.3, 0.55, 0.6))
    link(f"fin_{index}_actuator", bm, dark, hinge)
    x0, x1, half, depth = 0.2, 1.8, 0.65, 0.32
    bm = bmesh.new()
    for i in range(6):
        y = -half + 2 * half * i / 5
        thickness = 0.07 if i in (0, 5) else 0.025
        box(bm, ((x0 + x1) / 2, y, 0.0), (x1 - x0, thickness, depth))
    for i in range(6):
        x = x0 + (x1 - x0) * i / 5
        thickness = 0.07 if i in (0, 5) else 0.025
        box(bm, (x, 0.0, 0.0), (thickness, 2 * half, depth))
    link(f"fin_{index}_lattice", bm, titanium, hinge)


def leg(index, carbon, metal):
    """Tapered carbon leg lying along the body (stowed) with a foot pad at the tip that is
    level once the leg swings out to its deployed angle."""
    angle = index * math.pi / 2  # legs share the fins' azimuths
    hinge = empty(f"leg_{index}", (RADIUS * math.cos(angle), RADIUS * math.sin(angle), LEG_HINGE_Z), angle)
    bm = bmesh.new()
    bottom = [(0.0, -0.36), (0.45, -0.36), (0.45, 0.36), (0.0, 0.36)]
    top = [(0.02, -0.2), (0.3, -0.2), (0.3, 0.2), (0.02, 0.2)]
    lower = [bm.verts.new((x, y, 0.0)) for x, y in bottom]
    upper = [bm.verts.new((x, y, LEG_LENGTH)) for x, y in top]
    bm.faces.new(tuple(reversed(lower)))
    bm.faces.new(upper)
    for j in range(4):
        k = (j + 1) % 4
        bm.faces.new((lower[j], lower[k], upper[k], upper[j]))
    link(f"leg_{index}_beam", bm, carbon, hinge)
    # Pad axis that becomes vertical after the deploy swing (a rotation about +Y here).
    axis = Vector((-math.sin(LEG_DEPLOYED), 0.0, math.cos(LEG_DEPLOYED)))
    bm = bmesh.new()
    pad = bmesh.ops.create_cone(bm, cap_ends=True, segments=24, radius1=0.6, radius2=0.5, depth=0.18)
    for face in bm.faces:
        face.smooth = len(face.verts) == 4
    align = axis.to_track_quat("Z", "Y").to_matrix().to_4x4()
    bmesh.ops.transform(bm, matrix=Matrix.Translation((0.16, 0.0, LEG_LENGTH)) @ align, verts=pad["verts"])
    link(f"leg_{index}_foot", bm, metal, hinge)


def engines(soot, nozzle):
    """Octaweb thrust structure and nine bells on a shared gimbal pivot at the base."""
    pivot = empty("engines", (0.0, 0.0, BASE))
    bm = bmesh.new()
    rings = lathe(bm, [(RADIUS * 0.97, -OCTAWEB_DEPTH), (RADIUS * 0.97, 0.0)])
    cap(bm, rings[0], up=False)
    for i in range(8):
        turn = Matrix.Rotation(i * math.pi / 4, 4, "Z")
        box(bm, (0.9, 0.0, -OCTAWEB_DEPTH - 0.06), (1.7, 0.12, 0.12), turn)
    link("octaweb", bm, soot, pivot)
    # Throat to exit with a bell-shaped flare; double-sided so the inside shows.
    profile = []
    for i in range(13):
        s = i / 12
        profile.append((0.17 + 0.33 * math.sqrt(s), -OCTAWEB_DEPTH - BELL_LENGTH * s))
    profile.reverse()  # lathe wants bottom to top
    bm = bmesh.new()
    for engine in range(9):
        if engine == 0:
            x = y = 0.0
        else:
            # Same placement as the viewer: 45 degree steps offset by 22.5 degrees.
            a = (engine - 1) * math.pi / 4 + math.pi / 8
            x, y = ENGINE_RING * math.cos(a), -ENGINE_RING * math.sin(a)
        geom = bmesh.new()
        lathe(geom, profile, segments=32)
        bmesh.ops.translate(geom, vec=Vector((x, y, 0.0)), verts=geom.verts)
        mesh = bpy.data.meshes.new("tmp")
        geom.to_mesh(mesh)
        geom.free()
        bm.from_mesh(mesh)
        bpy.data.meshes.remove(mesh)
    link("bells", bm, nozzle, pivot)


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    white = material("paint", (0.92, 0.92, 0.90), roughness=0.45)
    weathered = material("weathered_paint", (1.0, 1.0, 1.0), roughness=0.45, color_attribute="Col")
    gray = material("seam", (0.55, 0.55, 0.55), roughness=0.6)
    black = material("interstage", (0.03, 0.03, 0.035), roughness=0.55)
    dark = material("dark_metal", (0.12, 0.12, 0.13), metallic=0.6, roughness=0.5)
    soot = material("soot", (0.08, 0.075, 0.07), metallic=0.3, roughness=0.7)
    titanium = material("titanium", (0.52, 0.5, 0.48), metallic=0.9, roughness=0.35)
    carbon = material("carbon", (0.035, 0.035, 0.04), metallic=0.1, roughness=0.3)
    metal = material("aluminium", (0.6, 0.6, 0.62), metallic=0.9, roughness=0.4)
    nozzle = material("nozzle", (0.2, 0.16, 0.14), metallic=0.85, roughness=0.3, double_sided=True)

    body(weathered)
    seams(gray)
    interstage(black, dark)
    raceway_and_thrusters(white, black)
    for i in range(4):
        grid_fin(i, titanium, dark)
        leg(i, carbon, metal)
    engines(soot, nozzle)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(OUT),
        export_format="GLB",
        export_yup=True,
        export_apply=True,
        export_vertex_color="MATERIAL",
        export_cameras=False,
        export_lights=False,
    )
    print(f"exported {OUT}")


main()
