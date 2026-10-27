#!/usr/bin/env python3
"""Generate vo_town model SDFs and the vo_town.sdf world."""
import os

REPO = "/mnt/disk2/sprind/baza/src/simulation_tools"
MODELS = f"{REPO}/models/vo_town"
WORLD = f"{REPO}/worlds/vo_town.sdf"


def material(albedo, normal=None, rough=0.85, metal=0.0, ambient=None, diffuse=None,
             color=None, specular="0.2 0.2 0.2 1"):
    if color:  # plain colored material, no texture
        return f"""
          <material>
            <ambient>{color} 1</ambient>
            <diffuse>{color} 1</diffuse>
            <specular>{specular}</specular>
            <pbr><metal><roughness>{rough}</roughness><metalness>{metal}</metalness></metal></pbr>
          </material>"""
    nrm = f"\n              <normal_map>{normal}</normal_map>" if normal else ""
    return f"""
          <material>
            <ambient>{ambient or '1 1 1 1'}</ambient>
            <diffuse>{diffuse or '1 1 1 1'}</diffuse>
            <specular>{specular}</specular>
            <pbr><metal>
              <albedo_map>{albedo}</albedo_map>{nrm}
              <roughness>{rough}</roughness><metalness>{metal}</metalness>
            </metal></pbr>
          </material>"""


def visual(name, geom, mat, pose="0 0 0 0 0 0", shadows=False):
    return f"""
      <visual name="{name}">
        <pose>{pose}</pose>
        <cast_shadows>{'true' if shadows else 'false'}</cast_shadows>
        <geometry>{geom}</geometry>{mat}
      </visual>"""


def box(x, y, z):
    return f"<box><size>{x} {y} {z}</size></box>"


def cyl(r, l):
    return f"<cylinder><radius>{r}</radius><length>{l}</length></cylinder>"


def sphere(r):
    return f"<sphere><radius>{r}</radius></sphere>"


def collision_box(x, y, z, cz):
    return f"""
      <collision name="collision">
        <pose>0 0 {cz} 0 0 0</pose>
        <geometry>{box(x, y, z)}</geometry>
      </collision>"""


def write_model(name, body, description):
    d = f"{MODELS}/{name}"
    os.makedirs(d, exist_ok=True)
    with open(f"{d}/model.config", "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>vo_town generator</name><email>zhuk.pn@ucu.edu.ua</email></author>
  <description>{description}</description>
</model>
""")
    with open(f"{d}/model.sdf", "w") as f:
        f.write(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">{body}
    </link>
  </model>
</sdf>
""")
    print(f"model {name}")


# ---------------------------------------------------------------- ground quadrants
for v in "abcd":
    body = visual("grass", "<plane><normal>0 0 1</normal><size>100 100</size></plane>",
                  material(f"textures/ground_{v}_ground_color.jpg", f"textures/ground_{v}_ground_normal.jpg", rough=0.95))
    write_model(f"ground_{v}", body, "100x100 m grass quadrant with baked dirt/path detail (CC0 ambientCG Grass004/Ground037)")

# ---------------------------------------------------------------- roads
for v in "ab":
    road = visual("asphalt", box(8, 22, 0.06),
                  material(f"textures/road_straight_{v}_road_color.jpg", f"textures/road_straight_{v}_road_normal.jpg", rough=0.9),
                  pose="0 0 0.03 0 0 0")
    walks = ""
    for side, sx in (("left", -5.2), ("right", 5.2)):
        walks += visual(f"sidewalk_{side}", box(2.4, 22, 0.18),
                        material(f"textures/road_straight_{v}_sidewalk_color.jpg", f"textures/road_straight_{v}_sidewalk_normal.jpg", rough=0.9),
                        pose=f"{sx} 0 0.09 0 0 0")
    write_model(f"road_straight_{v}", road + walks,
                "22 m two-lane road segment with lane markings and sidewalks (CC0 ambientCG Asphalt026C/PavingStones128)")

inter = visual("asphalt", box(8, 8, 0.06),
               material("textures/road_intersection_intersection_color.jpg", "textures/road_intersection_intersection_normal.jpg", rough=0.9),
               pose="0 0 0.03 0 0 0")
for i, (px, py) in enumerate([(-5.2, -5.2), (-5.2, 5.2), (5.2, -5.2), (5.2, 5.2)]):
    inter += visual(f"pad_{i}", box(2.4, 2.4, 0.18),
                    material("textures/road_intersection_pad_color.jpg", rough=0.9),
                    pose=f"{px} {py} 0.09 0 0 0")
write_model("road_intersection", inter,
            "8x8 m intersection with zebra crosswalks and corner sidewalk pads (CC0 ambientCG)")

write_model("parking_lot",
            visual("lot", box(12, 18, 0.05),
                   material("textures/parking_lot_parking_color.jpg", "textures/parking_lot_parking_normal.jpg", rough=0.9),
                   pose="0 0 0.025 0 0 0"),
            "12x18 m parking lot with stall markings (CC0 ambientCG Asphalt026C)")

# ---------------------------------------------------------------- buildings
def building(name, w, l, h, roof_kind, desc, roof_over=0.4, roof_th=0.25,
             facade_rough=0.7, facade_metal=0.0):
    roof_mat = {"tile": material(f"textures/{name}_roof_color.jpg", f"textures/{name}_roof_normal.jpg", rough=0.9),
                "flat": material(f"textures/{name}_roof_color.jpg", rough=0.95),
                "metal": material(f"textures/{name}_roof_color.jpg", rough=0.6, metal=0.4)}[roof_kind]
    body = visual("walls", box(w, l, h),
                  material(f"textures/{name}_facade_color.jpg", f"textures/{name}_facade_normal.jpg",
                           rough=facade_rough, metal=facade_metal),
                  pose=f"0 0 {h / 2} 0 0 0")
    body += visual("roof", box(w + roof_over, l + roof_over, roof_th), roof_mat,
                   pose=f"0 0 {h + roof_th / 2} 0 0 0")
    body += collision_box(w, l, h + roof_th, (h + roof_th) / 2)
    write_model(name, body, desc)


building("house_brick_a", 8, 10, 5.5, "tile",
         "Single-story brick house, tiled roof (CC0 ambientCG Bricks059/RoofingTiles003)", roof_over=0.7)
building("house_brick_b", 9, 9, 6.5, "tile",
         "Brick house with two window rows, dark tiled roof (CC0 ambientCG Bricks090/RoofingTiles003)", roof_over=0.7)
building("house_plaster", 8, 10, 5.0, "tile",
         "Plastered cottage, light tiled roof (CC0 ambientCG Plaster003/RoofingTiles003)", roof_over=0.7)
building("house_two_story", 9, 11, 7.5, "tile",
         "Two-story rendered house (CC0 ambientCG Plaster003/RoofingTiles003)", roof_over=0.7)
building("shop_row", 20, 8, 6.5, "flat",
         "Row of shops: striped awnings, storefront glass, sign bands (CC0 ambientCG Bricks090/Concrete034)",
         roof_over=0.2, roof_th=0.35)
building("midrise_office", 16, 20, 18, "flat",
         "Mid-rise office block, white-mullion glass grid facade (CC0 ambientCG Facade006/Concrete034)",
         roof_over=0.2, roof_th=0.4, facade_rough=0.45, facade_metal=0.15)
building("midrise_dark", 14, 14, 15, "flat",
         "Dark mid-rise with lit-window facade (CC0 ambientCG Facade012/Concrete034)",
         roof_over=0.2, roof_th=0.4, facade_rough=0.55)
building("tower_glass", 18, 18, 30, "flat",
         "Glass curtain-wall tower with rooftop helipad (CC0 ambientCG Facade001/Concrete034)",
         roof_over=0.2, roof_th=0.5, facade_rough=0.3, facade_metal=0.3)
building("warehouse", 24, 18, 9, "metal",
         "Corrugated-steel warehouse with roller doors, seamed metal roof (CC0 ambientCG CorrugatedSteel005/MetalPlates006)",
         roof_over=0.6, roof_th=0.3)
building("civic_hall", 16, 12, 8, "flat",
         "Civic hall: concrete facade, tall windows, banner (CC0 ambientCG Concrete034)",
         roof_over=0.5, roof_th=0.35)

# ---------------------------------------------------------------- props
pole_mat = material(None, color="0.25 0.26 0.28", rough=0.5, metal=0.7)
light = (visual("pole", cyl(0.08, 5.0), pole_mat, pose="0 0 2.5 0 0 0")
         + visual("arm", box(1.5, 0.1, 0.1), pole_mat, pose="0.7 0 4.95 0 0 0")
         + visual("head", box(0.6, 0.24, 0.14),
                  material(None, color="0.9 0.9 0.82", rough=0.4), pose="1.3 0 4.9 0 0 0"))
write_model("streetlight", light, "5 m streetlight, visual only (no lamp light source)")

tree = (visual("trunk", cyl(0.18, 2.6),
               material("textures/tree_bark_color.jpg", rough=0.95), pose="0 0 1.3 0 0 0")
        + visual("canopy", sphere(2.1),
                 material("textures/tree_canopy_color.jpg", rough=0.95), pose="0 0 4.1 0 0 0")
        + visual("canopy2", sphere(1.4),
                 material("textures/tree_canopy_color.jpg", rough=0.95), pose="0.9 0.5 3.2 0 0 0.9"))
write_model("tree", tree, "Low-poly street tree: bark cylinder + textured canopy spheres (CC0 ambientCG Bark006)")

for cname, ccol in [("car_red", "0.55 0.08 0.07"), ("car_blue", "0.10 0.16 0.35"),
                    ("car_white", "0.82 0.82 0.80")]:
    car = (visual("body", box(4.2, 1.8, 0.6),
                  material(None, color=ccol, rough=0.35, metal=0.5,
                           specular="0.5 0.5 0.5 1"), pose="0 0 0.55 0 0 0")
           + visual("cabin", box(2.2, 1.6, 0.55),
                    material(None, color="0.06 0.07 0.09", rough=0.15, metal=0.3,
                             specular="0.6 0.6 0.6 1"), pose="-0.2 0 1.1 0 0 0")
           + visual("under", box(4.0, 1.7, 0.28),
                    material(None, color="0.05 0.05 0.05", rough=0.9), pose="0 0 0.14 0 0 0"))
    write_model(cname, car, "Parked car (static, decorative, no collision)")

for sname in ("street_sign", "street_sign_yellow"):
    sign = (visual("pole", cyl(0.04, 2.6), pole_mat, pose="0 0 1.3 0 0 0")
            + visual("board", box(0.06, 0.95, 0.7),
                     material(f"textures/{sname}_sign_color.jpg", rough=0.4, metal=0.2),
                     pose="0 0 2.15 0 0 0"))
    write_model(sname, sign, "Street sign board, visual only")

# ================================================================ WORLD
inc = []


def include(model, name, x, y, yaw=0.0, z=0.0):
    inc.append(f"""    <include>
      <uri>model://vo_town/{model}</uri>
      <name>{name}</name>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
    </include>""")


import math
D90 = math.pi / 2

# ground quadrants
for (v, x, y) in [("a", 50, 50), ("b", -50, 50), ("c", -50, -50), ("d", 50, -50)]:
    include(f"ground_{v}", f"ground_{v}", x, y, z=0.001)

# road grid: roads at x/y in {-52, 0, 52}, intersections at the 9 crossings
GRID = (-52, 0, 52)
seg_centers = (-89, -67, -37, -15, 15, 37, 67, 89)
n = 0
for gx in GRID:
    for sy in seg_centers:
        n += 1
        include(f"road_straight_{'ab'[n % 2]}", f"road_ns_{n}", gx, sy)
for gy in GRID:
    for sx in seg_centers:
        n += 1
        include(f"road_straight_{'ab'[n % 2]}", f"road_ew_{n}", sx, gy, yaw=D90)
for i, gx in enumerate(GRID):
    for j, gy in enumerate(GRID):
        include("road_intersection", f"intersection_{i}{j}", gx, gy)

# ---- buildings (16 city blocks) ----
B = [
    # central business blocks
    ("tower_glass",     "tower",          22,  22,  0),
    ("parking_lot",     "parking_ne",     38,  14,  0),
    ("midrise_office",  "office_nw",     -26,  30,  0),
    ("shop_row",        "shops_nw",      -26,  13,  0),
    ("midrise_dark",    "midrise_dark",  -29, -29,  0),
    ("civic_hall",      "civic_hall",    -26, -13,  0),
    ("shop_row",        "shops_se",       26, -11,  math.pi),
    ("parking_lot",     "parking_se",     38, -30,  0),
    ("house_two_story", "h2_se",          14, -38,  0.2),
    # north edge blocks
    ("midrise_office",  "office_n",       26,  80,  D90),
    ("house_two_story", "h2_n1",         -18,  72,  0),
    ("house_brick_a",   "hba_n1",        -35,  75,  D90),
    ("house_plaster",   "hpl_n1",        -24,  90,  0.1),
    # east edge blocks
    ("house_brick_a",   "hba_e1",         70,  15,  D90),
    ("house_plaster",   "hpl_e1",         88,  20,  0),
    ("house_brick_b",   "hbb_e1",         75,  38,  0.26),
    ("warehouse",       "warehouse_e",    80, -26,  0),
    # corners
    ("house_two_story", "h2_ne",          70,  70,  0),
    ("house_brick_a",   "hba_ne",         88,  75,  D90),
    ("house_plaster",   "hpl_ne",         75,  90,  0),
    ("house_brick_b",   "hbb_nw",        -70,  70,  0.7),
    ("house_plaster",   "hpl_nw",        -88,  78,  0),
    ("house_brick_a",   "hba_nw",        -74,  90,  D90),
    ("house_brick_b",   "hbb_se",         70, -70,  0),
    ("house_plaster",   "hpl_se2",        88, -72,  D90),
    ("house_brick_a",   "hba_se",         72, -90,  0),
    ("house_two_story", "h2_sw",         -70, -70,  math.pi),
    ("house_brick_a",   "hba_sw",        -88, -75,  D90),
    ("house_brick_b",   "hbb_sw",        -72, -90,  0),
    # west edge blocks
    ("shop_row",        "shops_w",       -79,  15,  0),
    ("house_two_story", "h2_w",          -79,  35,  0),
    ("warehouse",       "warehouse_w",   -79, -28,  D90),
    # south edge blocks
    ("house_plaster",   "hpl_s1",         16, -70,  D90),
    ("house_brick_b",   "hbb_s1",         36, -72,  0),
    ("house_two_story", "h2_s1",          24, -90,  0),
    ("shop_row",        "shops_s",       -26, -70,  math.pi),
    ("house_brick_a",   "hba_s2",        -14, -88,  0),
    ("house_plaster",   "hpl_s2",        -38, -88,  D90),
]
for model, name, x, y, yaw in B:
    include(model, name, x, y, yaw=yaw)

# ---- trees: along sidewalks and in yards ----
T = [(8.2, 20), (8.2, 34), (-8.2, 26), (-8.2, 42), (8.2, -20), (-8.2, -34),
     (20, 8.2), (34, -8.2), (-22, 8.2), (-38, -8.2), (44, 8.2), (-44, -8.2),
     (60.5, 30), (60.5, -12), (-60.5, 44), (-60.5, -44), (30, 60.5), (-14, 60.5),
     (-34, -60.5), (12, -60.5), (85, 88), (-85, 60), (92, -60), (-92, -85),
     (64, 64), (-64, -62), (40, 88), (-45, -75)]
for i, (x, y) in enumerate(T):
    include("tree", f"tree_{i}", x, y, yaw=(i * 0.83) % 6.28)

# ---- streetlights: two diagonal corners of each intersection, arm toward road ----
i = 0
for gx in GRID:
    for gy in GRID:
        include("streetlight", f"lamp_{i}", gx + 5.9, gy + 5.9, yaw=math.pi * 0.75); i += 1
        include("streetlight", f"lamp_{i}", gx - 5.9, gy - 5.9, yaw=-math.pi * 0.25); i += 1
# midblock lamps on the central roads
for y in (-30, 30):
    include("streetlight", f"lamp_{i}", 5.9, y, yaw=math.pi); i += 1
    include("streetlight", f"lamp_{i}", -5.9, -y, yaw=0.0); i += 1

# ---- parked cars: curbside + parking lots ----
C = [("car_red",   49.4, 26,  D90), ("car_blue",  54.6, -18, -D90),
     ("car_white", -2.6, 70,  D90), ("car_red",   2.6, -66, -D90),
     ("car_blue", -49.4, -33, D90), ("car_white", -54.6, 40, -D90),
     ("car_red",   30, 49.4,  0), ("car_white", -20, 54.6, math.pi),
     ("car_blue",  20, -49.4, 0),
     # parking lot NE (lot at 38,14; stall rows near x=34.5 / 41.5)
     ("car_white", 34.6, 9.5,  0), ("car_red", 34.6, 17.3, 0), ("car_blue", 41.4, 12.1, math.pi),
     # parking lot SE (lot at 38,-30)
     ("car_blue", 34.6, -27.4, 0), ("car_white", 41.4, -33.0, math.pi)]
for i, (m, x, y, yaw) in enumerate(C):
    include(m, f"car_{i}", x, y, yaw=yaw)

# ---- signs near intersections ----
S = [("street_sign", 5.9, -6.6, 0), ("street_sign_yellow", -57.9, 6.6, math.pi),
     ("street_sign", 46.1, 57.9, D90), ("street_sign_yellow", -46.1, -57.9, -D90),
     ("street_sign", 57.9, -46.1, 0), ("street_sign_yellow", -5.9, 58.6, math.pi)]
for i, (m, x, y, yaw) in enumerate(S):
    include(m, f"sign_{i}", x, y, yaw=yaw)

includes = "\n".join(inc)

world = f"""<?xml version="1.0"?>
<!--
  vo_town: photorealistic small-town world for UAV visual-odometry testing.
  200 x 200 m, static geometry only (no vehicles/robots/sensors).
  Generated assets live in models/vo_town; textures are CC0 from ambientCG.com.
  Launch: GZ_SIM_RESOURCE_PATH=$(pwd)/../models gz sim vo_town.sdf   (see models/vo_town/README.md)
-->
<sdf version="1.9">
  <world name="vo_town">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>
    <plugin name="gz::sim::systems::NavSat" filename="gz-sim-navsat-system"/>

    <scene>
      <ambient>0.65 0.65 0.65 1</ambient>
      <background>0.55 0.7 0.9 1</background>
      <sky/>
      <shadows>false</shadows>
    </scene>

    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>5.5645e-06 2.28758e-05 -4.23884e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <!-- single directional sun, no shadow maps (performance) -->
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.88 0.82 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.45 0.15 -0.88</direction>
      <intensity>1.0</intensity>
    </light>

    <!-- flat ground collision for the whole 200x200 area (visuals come from ground_* quadrants) -->
    <model name="ground_collision">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>200 200</size></plane>
          </geometry>
        </collision>
      </link>
    </model>

{includes}
  </world>
</sdf>
"""
with open(WORLD, "w") as f:
    f.write(world)
print(f"\nwrote {WORLD} with {len(inc)} includes")
