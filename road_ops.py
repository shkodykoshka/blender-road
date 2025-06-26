import bpy
import bmesh
import math
import mathutils
# Most information, equations, and tables are from AASHTO Greenbook 7th Edition (2018)


# Gets config from __init__.py (user settings for road)
def get_config():
    road_config = bpy.context.scene.road_config
    config_dict = {
        "speed" : road_config.design_speed,
        "superelevation" : road_config.design_superelevation,
        "road_width" : road_config.road_width,
        "use_vector_radius": road_config.use_vector_radius,
        "segments": road_config.segments,
        "runoff_segments": road_config.runoff_segments
    }
    return config_dict


# Gets selected vertex indices in the order they where selected
# Returns list ordered by: first (oldest) selected, ..., last (most recent) selected
def get_selected_vertex_indices():
    selected_verts = []
    # Get selected object and its mesh
    edit_object = bpy.context.object
    edit_object_data = edit_object.data
    edit_bmesh = bmesh.from_edit_mesh(edit_object_data)
    # Only get selected vertices
    for vert in reversed(edit_bmesh.select_history):
        if isinstance(vert, bmesh.types.BMVert):
            selected_verts.append(vert.index)
    return selected_verts


# Get bmvert from given index
def get_vert_at_index(index):
    edit_object = bpy.context.object
    return edit_object.data.vertices[index]


# Removes a road blender object and deletes mesh if it has no users
def remove_road(obj):
    if obj is not None:
        if obj.get("road_obj", None):
            bpy.data.objects.remove(obj["road_obj"])
    # Remove meshes
    for mesh_block in bpy.data.meshes:
        if mesh_block.users == 0:
            bpy.data.meshes.remove(mesh_block)


# Makes blender object with points given coordinate_list (list of coordinate tuples in x,y,z order)
def make_points(name, coordinate_list):
    scn = bpy.context.collection
    w_mesh = bpy.data.meshes.new(name)
    w_obj = bpy.data.objects.new(name, w_mesh)
    scn.objects.link(w_obj)
    # Making all points
    w_mesh.from_pydata(coordinate_list, [], [])
    w_mesh.update()


# Makes faces for road
def make_face(name, coordinate_list, segments):
    f_list = []
    # Create first segment
    for i in range(segments):
        f_list.append((i+segments+1, i+1, i))
        f_list.append((i+segments+1, i+segments+2, i+1))
    
    # The order is the same for the second segment, but the two segments should not be connected
    for i in range(segments+1, (segments*2)+1):
        f_list.append((i+segments+1, i+1, i))
        f_list.append((i+segments+1, i+segments+2, i+1))
    
    return make_model(name, coordinate_list, f_list)


# Creates a blender object (model)
def make_model(name, coordinate_list, face_list):
    scn = bpy.context.collection
    road_mesh = bpy.data.meshes.new(name)
    road_obj = bpy.data.objects.new(name, road_mesh)
    scn.objects.link(road_obj)
    # Making all points
    road_mesh.from_pydata(coordinate_list, [], face_list)
    road_mesh.update()
    return road_obj


# Clamps speed between 15 and 80 mph
# Clamped to 15 at lowest as that is the lowest speed most tables go to
def clamp_speed(speed):
    # Make sure speed is multiple of 5
    if speed % 5 != 0:
        speed -= (speed % 5)
    # Clamp speed to above range
    speed = max(min(80, speed), 15)
    return speed


# Convert feet to meter (with no rounding)
def to_meter(feet):
    return feet * 0.3048


# Finds vertical curve length in feet
# All parameters in this function are in U.S. Customary Units
def find_vertical_curve_feet(vec_1_grade_percent, vec_2_grade_percent, speed, a2):
    # Sight stopping distance from Table 3-35 in Greenbook (p. 3-170)
    # MPH speeds and feet distance
    design_ssd = {
        10 : 50,
        15 : 80,
        20 : 115,
        25 : 155,
        30 : 200,
        35 : 250,
        40 : 305,
        45 : 360,
        50 : 425,
        55 : 495,
        60 : 570,
        65 : 645,
        70 : 730,
        75 : 820,
        80 : 910
    }

    # Unsigned algebraic difference between two curves (vertical alignment, not horizontal)
    # The Greenbook uses 'a' as unsigned algebraic difference
    a1 = abs(a2)
    # Get sight stopping distance from speed
    sight_stopping_distance = design_ssd[speed]

    # These equations assume height of eye is 3.5 ft and height of object is 2.0 ft, 
    # Determine if sag or crest vertical curve
    if vec_1_grade_percent < vec_2_grade_percent:
        # Approaching grade% is less than departing grade% (create a sag curve)
        # Using sight stopping distance instead of light beam distance and assuming S (sight stopping distance) is less than L (length of curve)
        # Greenbook equation 3-49 (p. 3-173)
        vertical_curve_length_feet = (a1 * (speed**2)) / (400 + 3.5 * sight_stopping_distance)
    elif vec_1_grade_percent > vec_2_grade_percent:
        # Approaching grade% is greater than departing grade% (create crest curve)
        # Assuming S (sight distance) is less than L (length of curve). Greenbook equation 3-44 (p. 3-167)
        vertical_curve_length_feet = (a1 * (speed**2)) / 2158
    else:
        # They are the same: 0% grade difference
        vertical_curve_length_feet = 0
    return vertical_curve_length_feet


# Called when a turn needs to be drawn (all three points are not on a single line)
# Finds circle center, start and end angles for turn
def find_circle_center(angle_real_half, radius_meters, vec_1_horizontal, vec_2_horizontal, shared_point):
    # Get rotation direction
    # Angle_signed returns a positive value for clockwise rotation and negative for counterclockwise
    # Multiplying vector by -1 to get rotation direction to be negative for clockwise and positive for counterclockwise
    rotation_direction = -mathutils.Vector((vec_1_horizontal.x, vec_1_horizontal.y)).angle_signed(mathutils.Vector((vec_2_horizontal.x, vec_2_horizontal.y)))
    rotation_sign = 1
    if rotation_direction < 0:
        # Clockwise rotation
        rotation_sign = -1
    
    # Find angle between hypotenuse and ray perpendicular to vec_1
    angle_hypo = (math.pi/2) - angle_real_half

    # Distance of circle center to shared_point
    circle_center_dist = radius_meters / math.sin(angle_real_half)
    # Create vector of correct length, then rotate into correct position
    circle_center_vec = vec_2_horizontal.normalized() * circle_center_dist
    circle_center_vec.rotate(mathutils.Euler((0, 0, (angle_real_half * rotation_sign))))
    circle_center_vec += shared_point

    # Find point where arc is tangent with vec_1
    tangent_point = circle_center_vec.project(vec_1_horizontal)
    
    # Find start angle and end angles
    start_angle = math.atan2((tangent_point.y - circle_center_vec.y), (tangent_point.x - circle_center_vec.x))
    # Multiply angle_hypo by negative rotation_sign to get correct end angle
    end_angle = start_angle + (angle_hypo * 2 * (-rotation_sign))
    return (circle_center_vec, start_angle, end_angle)


# Finds the runoff length required for given design speed and superelevation
# Returns length in meters
def find_runoff(design_speed, superelevation, road_width):
    # Superelevation runoff is the section and distance for road to move from full super to flat (or other direction)
    # (Greenbook pp.3-61)

    # Since lane_width and speed need to be in same units, it makes sense to use feet for this equation and turn into meters later
    # This value can be hardcoded - (Greenbook pp. 3-65)
    lane_width = 12 # 12 feet is typical in US
    # num_lanes equal to half the lanes in cross section
    # There must be at least 1 lane in both directions (if making a 1 lane one-way road just pretend it is 2 lanes)
    num_lanes = max((road_width / 2) / (to_meter(lane_width)), 1)

    # Greenbook table 3-15 shows equation for b_w factor (pp. 3-64)
    b_w = (1 + 0.5*(num_lanes - 1))/num_lanes

    # Equation 3-32 asks for max relative gradient, Iowa DOT provides table (Table 2)
    # https://iowadot.gov/design/dmanual/02a-02.pdf (Accessed ~March 2024)
    # This pdf is no longer accessible. Updated version available below
    # max_relative_gradient = {
    #    15: 0.78,
    #    20: 0.74,
    #    25: 0.70,
    #    30: 0.66,
    #    35: 0.62,
    #    40: 0.58,
    #    45: 0.54,
    #    50: 0.50,
    #    55: 0.47,
    #    60: 0.45,
    #    65: 0.43,
    #    70: 0.40,
    #    75: 0.38,
    #    80: 0.38
    #}

    # IOWA DOT has released an updated version on Aug. 29, 2024. Accessed Nov. 24, 2024
    # https://iowadot.gov/design/dmanual/2B.pdf
    # Table 2B.2 features modified max relative gradient values from table 2 in previous version
    # Values are in percent and are based on speed in MPH
    max_relative_gradient = {
        15: 0.89,
        20: 0.80,
        25: 0.73,
        30: 0.67,
        35: 0.62,
        40: 0.57,
        45: 0.53,
        50: 0.50,
        55: 0.50,
        60: 0.50,
        65: 0.50,
        70: 0.50,
        75: 0.50,
        80: 0.50
    }

    # Since speed and lane_width are in feet, this equation returns feet and needs to be converted to meters
    runoff_length_feet = ((lane_width * num_lanes) * superelevation) / max_relative_gradient[design_speed] * b_w
    return to_meter(runoff_length_feet)

def find_tangent_runout(runoff_length, superelevation, cross_slope):
    # Tangent runout is the section and distance for road to move from normal cross slope to flat (or other direction)
    # (Greenbook pp.3-61)

    # Get percent from cross_slope
    cross_slope_percent = cross_slope * 100
    # Calculate runout_length - this is in the same units as the provided runoff_length
    # Greebook equation 3-24 for minumum runout length (pp. 3-70)
    runout_length = (cross_slope_percent / superelevation) * runoff_length
    return runout_length


# Gets all points (including vertical curve) and draws them
def make_turn(start_vec, start_angle, end_angle, min_radius, segments, runoff_segments, vec_1_grade_meter, a2, superelevation, road_width, vec_1, vec_2, speed):
    # With superelevation, the road width needs to be less than road_width/2 due to having height from superelevation
    radius_inc = math.sqrt(road_width**2 - ((superelevation/100)*road_width)**2)/2
    # Radius order needs to be outer radius, centerline, inner radius
    radius_list = [min_radius + radius_inc * 2, min_radius + radius_inc, min_radius]

    angle_increment = (start_angle-end_angle)/(segments)
    # Coordinate list
    c_list = []
    curr_radius = 0
    # Width of road between two radii
    inc = road_width/(len(radius_list)-1)
    for radius in radius_list:
        angle = start_angle
        x = 0
        # Get current point (distance)
        x_dist = math.sqrt((radius * (math.cos(angle) - math.cos(angle-angle_increment)))**2 + (radius*(math.sin(angle) - math.sin(angle-angle_increment)))**2)
        # See Desmos link for math
        grade_change_meter = (a2 / (x_dist * (segments - 1) * 2)) / 100
        temp_list = []

        # Make circle
        for i in range(segments + 1):
            circle_x = math.cos(angle) * radius + start_vec.x
            circle_y = math.sin(angle) * radius + start_vec.y
            # Z equation is from https://www.desmos.com/calculator/keegwfba0e
            # or https://www.desmos.com/calculator/g6ezjb4npe (for old version)
            # Simplifying vertical curve equation in graphs above gives:
            circle_z = (x * (vec_1_grade_meter + x * grade_change_meter - grade_change_meter) + grade_change_meter) + start_vec.z
            # Changing outer radius to be higher and inner radius to be lower to include cross slope
            circle_z -= (inc * curr_radius) * (superelevation/100)
            x += x_dist
            temp_list.append((circle_x, circle_y, circle_z))
            angle += angle_increment

        if (superelevation > 0):
            # runout -> runoff -> radius -> runoff -> runout
            # Get runout and runoff sections
            runout_1, runoff_1 = get_runout_runoff(curr_radius, -vec_1, mathutils.Vector(temp_list[0]), runoff_segments, road_width, speed, superelevation)
            runout_2, runoff_2 = get_runout_runoff(curr_radius, vec_2, mathutils.Vector(temp_list[-1]), runoff_segments, road_width, speed, superelevation)
            # Reverse first two lists (their points are made going away from curve, need them going towards curve)
            runout_1.reverse()
            runoff_1.reverse()
            c_list += runout_1 + runoff_1 + temp_list + runoff_2 + runout_2
            segment_amnt = segments + 2 * (runoff_segments + 4)
        else:
            c_list += temp_list
            segment_amnt = segments
        
        curr_radius += 1

    return make_face("turn", c_list, segment_amnt)


# Makes a straight (no horizontal curve) road (this is for drawing vertical curve when no horizontal curve is present)
def make_straight(start_vec, segments, vertical_curve_length, vec_1_grade_meter, a2, vec_3, road_width):
    c_list = []
    side_1 = []
    side_2 = []
    # VPC is start of curve
    # VPT is end of curve
    # VPC -> VPT is vector of length vertical_curve_length with same direction as point_a -> point_b vector (vec_3)
    # Needed for delta x and y
    dir_vec = vec_3.normalized() * vertical_curve_length
    delta_x = (dir_vec.x) / segments
    delta_y = (dir_vec.y) / segments

    # Make y vector by crossing positive z vector and direction vector then make its length road_width/2
    # This is to get the outer sides of the road
    perpendicular_1 = (dir_vec.cross(mathutils.Vector((0, 0, 1)))).normalized() * (road_width / 2)
    perpendicular_2 = (dir_vec.cross(mathutils.Vector((0, 0, -1)))).normalized() * (road_width / 2)
    
    x = 0
    x_dist = math.sqrt((delta_x * delta_x) + (delta_y * delta_y))
    # Check if zero (otherwise div by 0 will be caused when dividing a2)
    travel_dist = (x_dist * (segments - 1) * 2)
    if (travel_dist == 0):
        grade_change_meter = 0
    else:
        grade_change_meter = (a2 / travel_dist) / 100

    # make circle
    for i in range(segments + 1):
        # X and y positions for each vertex
        road_x = start_vec.x + (i * delta_x)
        road_y = start_vec.y + (i * delta_y)
        # Z position (see desmos link above for graph)
        road_z = (x * (vec_1_grade_meter + x * grade_change_meter - grade_change_meter) + grade_change_meter) + start_vec.z
        x += x_dist
        c_list.append((road_x, road_y, road_z))
        
        # Make side vectors
        vec_side_1 = perpendicular_1.copy()
        vec_side_2 = perpendicular_2.copy()
        # Delta vector is used to get to current position
        delta = mathutils.Vector((start_vec.x + i * delta_x, start_vec.y + i * delta_y, road_z))
        vec_side_1 += delta
        vec_side_2 += delta
        
        side_1.append((vec_side_1.x, vec_side_1.y, vec_side_1.z))
        side_2.append((vec_side_2.x, vec_side_2.y, vec_side_2.z))

    combined = side_2 + c_list + side_1
    return make_face("road", combined, segments)


def get_runout_runoff(curr_radius, base_vec, offset_vec, runoff_segments, road_width, speed, superelevation,):
    # Cross slope rarely changes, 2% is normal on roads
    cross_slope = 0.02

    # Get lengths
    runoff_length = find_runoff(speed, superelevation, road_width)
    runout_length = find_tangent_runout(runoff_length, superelevation, cross_slope)

    # Runoff_vec is parallel to directional (base) - can cross base_vec with world z (0, 0, 1) to get road_surface_parallel vector
    # Crossing directional (base) and road_surface_parallel will result in road_surface_normal - vector vertically perpendicular to road at that segment
    road_surface_parallel = base_vec.cross(mathutils.Vector((0, 0, 1)))
    road_surface_normal = (base_vec.cross(road_surface_parallel)).normalized()

    # Check that the normal vector z value is positive
    if (road_surface_normal.z < 0):
        # Z component must be positive - if not multiply by -1 for opposite direction
        road_surface_normal *= -1
    
    # Hardcode runout segments to 4 (it is a short section - does not need many segments)
    runout_segments = 4
    runoff_pts = []
    runout_pts = []

    # Create runoff section - outside edge of roadway goes from full super to flat
    # Check the radius num (assuming only 3 radii create road)
    if (curr_radius == 0): 
        # This is outside radius
        # End position is flat with middle of road for outside radius
        runoff_vertical_change_vec = road_surface_normal * ((road_width / 2) * (superelevation / 100)) / runoff_segments
    elif (curr_radius == 2):
        # This is middle radius
        # End position is normal cross slope for inside radius
        runoff_vertical_change_vec = ((road_surface_normal * (road_width / 2)) * ((-superelevation / 100) + cross_slope)) / runoff_segments
    else:
        # This is inside radius
        runoff_vertical_change_vec = mathutils.Vector((0, 0, 0))

    # Distance between each point
    runoff_length_itr = runoff_length/runoff_segments
    # Start position of runoff
    runoff_pos = offset_vec
    # How much vector changes
    runoff_pos_itr = base_vec.normalized() * runoff_length_itr - runoff_vertical_change_vec
    # Create points
    for i in range(runoff_segments):
        runoff_pos += runoff_pos_itr
        runoff_pts.append(runoff_pos.to_tuple())
    
    # Create runout section - outside edge of roadway goes from flat to normal cross slope
    if (curr_radius == 0):
        runout_vertical_change_vec = (road_surface_normal * (road_width/2) * cross_slope) / (runout_segments)
    else:
        runout_vertical_change_vec = mathutils.Vector((0, 0, 0))
    
    # Same as runoff section above
    runout_length_itr = runout_length/runout_segments
    runout_pos = runoff_pos
    runout_pos_itr = base_vec.normalized() * runout_length_itr - runout_vertical_change_vec

    for i in range(runout_segments):
        runout_pos += runout_pos_itr
        runout_pts.append(runout_pos.to_tuple())

    return runout_pts, runoff_pts


def draw_road(point_a, point_b, shared_point):
    # Set object attributes
    edit_object = bpy.context.object
    edit_object["point_a"] = point_a
    edit_object["point_b"] = point_b
    edit_object["shared_point"] = shared_point

    remove_road(edit_object)

    # Get vertex vector from index
    point_a = edit_object.matrix_world @ get_vert_at_index(point_a).co
    point_b = edit_object.matrix_world @ get_vert_at_index(point_b).co
    shared_point = edit_object.matrix_world @ get_vert_at_index(shared_point).co

    # Get config values
    config = get_config()
    # make sure speed is increment of 5 and between 15 and 80
    speed = clamp_speed(config["speed"])
    superelevation = config["superelevation"]
    road_width = config["road_width"]
    use_vector_radius = config["use_vector_radius"]
    segments = config["segments"]
    runoff_segments = config["runoff_segments"]

    # Get all vectors
    # point_a -> shared_point
    vec_1 = mathutils.Vector((shared_point-point_a))
    vec_1_horizontal = mathutils.Vector((shared_point.x-point_a.x, shared_point.y-point_a.y, 0))
    vec_1_2d = mathutils.Vector((vec_1_horizontal.x, vec_1_horizontal.y))
    # shared_point -> point_b
    vec_2 = mathutils.Vector((point_b-shared_point))
    vec_2_horizontal = mathutils.Vector((point_b.x-shared_point.x, point_b.y-shared_point.y, 0))
    # point_a -> point_b
    vec_3 = mathutils.Vector((point_b-point_a))
    vec_3_horizontal = mathutils.Vector((point_b.x-point_a.x, point_b.y-point_a.y, 0))
    vec_3_2d = mathutils.Vector((vec_3_horizontal.x, vec_3_horizontal.y))
    vec_3_horizontal_dist = vec_3_horizontal.length

    # Calculate grade of vec_1 and vec_2
    # grade = rise/run; grade% = grade*100
    vec_1_horizontal_dist = vec_1_horizontal.length
    vec_1_vertical_dist = shared_point.z - point_a.z
    vec_1_grade_meter = vec_1_vertical_dist/vec_1_horizontal_dist
    vec_1_grade_percent = vec_1_grade_meter*100

    vec_2_horizontal_dist = vec_2_horizontal.length
    vec_2_vertical_dist = point_b.z - shared_point.z
    vec_2_grade_meter = vec_2_vertical_dist/vec_2_horizontal_dist
    vec_2_grade_percent = vec_2_grade_meter*100

    # Get signed algebraic difference between the two grades
    a2 = vec_2_grade_percent - vec_1_grade_percent
    vertical_curve_length_feet = find_vertical_curve_feet(vec_1_grade_percent, vec_2_grade_percent, speed, a2)

    # Angle real is angle between sides vec_1 and vec_2
    angle_real = math.pi - math.acos((vec_1.dot(vec_2)) / (vec_1.length * vec_2.length))
    angle_real_half = angle_real / 2

    # Check if vec_1_2d and vec_3_2d are on the same line
    # If they are not on the same line, create a horizontal curve. otherwise only create a vertical curve
    if (abs(vec_1_2d.cross(vec_3_2d)) > 0.001):
        # Friction factor from Greenbook table 3-7 (pp. 3-34)
        f_max = {
            10 : 0.38,
            15 : 0.32,
            20 : 0.27,
            25 : 0.23,
            30 : 0.20,
            35 : 0.18,
            40 : 0.16,
            45 : 0.15,
            50 : 0.14,
            55 : 0.13,
            60 : 0.12,
            65 : 0.11,
            70 : 0.10,
            75 : 0.09,
            80 : 0.08
        }

        # Calculate minimum radius of turn with given speed and superelevation
        # Greenbook equation 3-8 (p. 3-33)
        min_radius_feet = (speed**2)/(15*(0.01*superelevation + f_max[speed]))
        
        # Decide what to use for radius
        if use_vector_radius == True:
            # Using the radius created by selected points (already in meters if Blender is set to use meters)
            min_radius_meter = vec_3_horizontal_dist
        else:
            # VPC to VPT distance will always be larger when using min_radius than using vertical_curve_length
            min_radius_meter = to_meter(min_radius_feet)

        # Need to get center of circle for turn and start and end angles where vec_1 and vec_2 are tangent to the circle
        circle_center, start_angle, end_angle = find_circle_center(angle_real_half, min_radius_meter, vec_1_horizontal, vec_2_horizontal, shared_point)
        # Add height to circle_center (circle_center is currently 2d vector in xy plane)
        circle_center.z = shared_point.z

        # Make road with a horizontal curve (turn)
        road_object = make_turn(circle_center, start_angle, end_angle, min_radius_meter, segments, runoff_segments, vec_1_grade_meter, a2, superelevation, road_width, vec_1, vec_2, speed)
        
    else:
        # Cannot make a vertical curve if its length is 0
        if (vertical_curve_length_feet == 0):
            return
        vertical_curve_length_meter = to_meter(vertical_curve_length_feet)

        if use_vector_radius == True:
            vertical_curve_length_meter = vec_3_horizontal_dist

        # Draw the road
        road_object = make_straight(point_a, segments, vertical_curve_length_meter, vec_1_grade_meter, a2, vec_3, road_width)
    
    # Add road to selected objects properties (to know which road to delete when updating)
    edit_object["road_obj"] = road_object


def update_road():
    obj = bpy.context.object
    # Get vertices saved in object
    point_a = obj["point_a"]
    point_b = obj["point_b"]
    shared_point = obj["shared_point"]
    draw_road(point_a, point_b, shared_point)


class DrawRoad(bpy.types.Operator):
    bl_idname = "road_ops.draw_road"
    bl_label = "Draw Road"
    bl_description = "Draw the Road"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_options = {"REGISTER"}

    def execute(self, context):        
        # Get selected vertices
        selected_verts = get_selected_vertex_indices()
        point_a = selected_verts[2]
        point_b = selected_verts[1]
        shared_point = selected_verts[0]
        
        draw_road(point_a, point_b, shared_point)
        return {"FINISHED"}


class UpdateRoad(bpy.types.Operator):
    bl_idname = "road_ops.manual_update_road"
    bl_label = "Update Road"
    bl_description = "Update the Road"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_options = {"REGISTER"}

    def execute(self, context):        
        update_road()
        return {"FINISHED"}