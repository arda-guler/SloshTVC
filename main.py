from tkinter import *
import time

from vector2 import *

drag_coeff = 1E-8
gravity = vec2(0, -9.81)  # m/s^2
dt = 0

########################
#       CAMERA         #
########################

class camera():
    def __init__(self, name, pos, zoom, state):
        self.name = name
        self.pos = pos
        self.zoom = zoom
        self.state = state

    def activate(self):
        self.state = "active"

    def deactivate(self):
        self.state = "standby"

    def set_pos(self, pos):
        self.pos = pos

    def set_zoom(self, zoom):
        self.zoom = zoom

    def move(self, movement):
        self.pos += movement

    def do_zoom(self, zoom):
        self.zoom *= zoom

    def get_state(self):
        return self.state

    def get_pos(self):
        return self.pos

    def get_zoom(self):
        return self.zoom

def get_active_cam():
    current_cam = None

    for cam in cameras:
        if cam.get_state() == "active":
            current_cam = cam
            break

    return cam

def move_current_cam_left(event=None):
    get_active_cam().move(vec2(-30 * get_active_cam().get_zoom(), 0))


def move_current_cam_right(event=None):
    get_active_cam().move(vec2(30 * get_active_cam().get_zoom(), 0))


def move_current_cam_up(event=None):
    get_active_cam().move(vec2(0, 30 * get_active_cam().get_zoom()))


def move_current_cam_down(event=None):
    get_active_cam().move(vec2(0, -30 * get_active_cam().get_zoom()))


def zoom_current_cam_out(event=None):
    get_active_cam().do_zoom(2)


def zoom_current_cam_in(event=None):
    get_active_cam().do_zoom(0.5)

########################
#       GROUND         #
########################

class ground():
    def __init__(self, height, color, elasticity, k):
        self.height = height
        self.color = color
        self.elasticity = elasticity
        self.k = k

    def get_height(self):
        return self.height

    def get_color(self):
        return self.color

    def apply_force(self, points):
        for p in points:
            # normal force
            if p.get_pos().y < self.height:
                p.apply_force(vec2(0, p.mass * p.vel.y * -1 * (self.elasticity + 1) / dt))
                p.apply_force(gravity * p.mass)
                p.pos.y = self.height

            # friction
            if p.get_pos().y <= self.height:
                p.apply_force(vec2(p.vel.x, 0).normalized() * -1 * p.mass * gravity.mag() * self.k)

########################
#       LINK           #
########################

class rigid_link():
    def __init__(self, name, p1, p2, color, k=1000, b=0):
        self.name = name
        self.p1 = p1
        self.p2 = p2
        self.dist = get_dist_between(p1, p2)
        # spring coefficient
        self.k = k
        self.b = b
        self.color = color

    def get_k(self):
        return self.k

    def get_name(self):
        return self.name

    def get_color(self):
        return self.color

    def apply_force(self):
        if get_dist_between(self.p1, self.p2) > self.dist:
            self.p1.apply_force(
                self.p1.get_unit_vector_towards(self.p2) * self.k * abs(get_dist_between(self.p1, self.p2) - self.dist))
            self.p2.apply_force(
                self.p2.get_unit_vector_towards(self.p1) * self.k * abs(get_dist_between(self.p1, self.p2) - self.dist))

        elif get_dist_between(self.p1, self.p2) < self.dist:
            self.p1.apply_force(self.p1.get_unit_vector_towards(self.p2) * -self.k * abs(
                get_dist_between(self.p1, self.p2) - self.dist))
            self.p2.apply_force(self.p2.get_unit_vector_towards(self.p1) * -self.k * abs(
                get_dist_between(self.p1, self.p2) - self.dist))

        # damping
        if not self.b == 0:
            rel_outvel = (self.p2.vel - self.p1.vel) - (self.p2.pos - self.p1.pos) * (self.p2.vel - self.p1.vel).dot((self.p2.pos - self.p1.pos).normalized())
            self.p2.apply_force(self.p2.get_unit_vector_towards(self.p1) * rel_outvel.mag() * self.b)
            self.p1.apply_force(self.p2.get_unit_vector_towards(self.p1) * rel_outvel.mag() * -self.b)

    def get_midpoint(self):
        return (self.p1.get_pos() + self.p2.get_pos()) / 2

########################
#     POINT MASS       #
########################

class point():
    def __init__(self, name, pos, vel, color, mass=1, static=False):
        self.name = name
        self.pos = pos
        self.vel = vel
        self.accel = vec2()
        self.mass = mass
        self.static = static
        self.color = color

        self.limit_axis = None

    def get_name(self):
        return self.name

    def get_pos(self):
        return self.pos

    def get_vel(self):
        return self.vel

    def get_mass(self):
        return self.mass

    def get_color(self):
        return self.color

    def get_unit_vector_towards(self, p2):
        return (p2.pos - self.pos) / (p2.pos - self.pos).mag()

    def get_vector_towards(self, p2):
        if type(p2) is point:
            return p2.pos - self.pos
        else:
            return p2 - self.pos

    def clear_accel(self):
        # call this every tick to not have residual forces from
        # previous frame
        self.accel = vec2(0, 0)

    def apply_force(self, force):
        self.accel += force / self.mass

    def apply_gravity(self):
        self.apply_force(gravity * self.mass)

    def apply_drag(self):
        self.apply_force((self.vel.normalized() * -1) * (self.vel.mag() ** 2) * drag_coeff)

    def update_vel(self):
        if not self.static:
            self.vel += self.accel * dt

        if self.limit_axis:
            self.vel = self.limit_axis * self.vel.dot(self.limit_axis)

    def update_pos(self):
        if not self.static:
            self.pos += self.vel * dt

    def set_limit_axis(self, vec):
        if vec == "x":
            self.limit_axis = vec2(1, 0)
        elif vec == "y":
            self.limit_axis = vec2(0, 1)
        else:
            self.limit_axis = vec.normalized()

########################
#    CONSTANT FORCE    #
########################

class const_force():
    def __init__(self, name, point, force):
        self.name = name
        self.point = point
        self.force = force

    def apply(self):
        self.point.apply_force(self.force)

def get_dist_between(p1, p2):
    if (type(p1) is point) and (type(p2) is point):
        return (p1.pos - p2.pos).mag()
    elif (type(p1) is point) and not (type(p2) is point):
        return (p1.pos - p2).mag()
    elif not (type(p1) is point) and (type(p2) is point):
        return (p1 - p2.pos).mag()
    else:
        return (p1 - p2).mag()

class propellant:
    def __init__(self, name, pos, vel, color, mass=1):
        self.name = name
        self.pos = pos
        self.vel = vel
        self.accel = vec2()
        self.mass = mass
        self.color = color

        self.limit_axis = None

    def get_name(self):
        return self.name

    def get_pos(self):
        return self.pos

    def get_vel(self):
        return self.vel

    def get_mass(self):
        return self.mass

    def get_color(self):
        return self.color

    def get_unit_vector_towards(self, p2):
        return (p2.pos - self.pos) / (p2.pos - self.pos).mag()

    def get_vector_towards(self, p2):
        if type(p2) is point:
            return p2.pos - self.pos
        else:
            return p2 - self.pos

    def clear_accel(self):
        # call this every tick to not have residual forces from
        # previous frame
        self.accel = vec2(0, 0)

    def apply_force(self, force):
        self.accel += force / self.mass

    def apply_gravity(self):
        self.apply_force(gravity * self.mass)

    def apply_drag(self):
        self.apply_force((self.vel.normalized() * -1) * (self.vel.mag() ** 2) * drag_coeff)

    def update_vel(self):
        self.vel += self.accel * dt

        if self.limit_axis:
            self.vel = self.limit_axis * self.vel.dot(self.limit_axis)

    def update_pos(self):
        self.pos += self.vel * dt

    def set_limit_axis(self, vec):
        if vec == "x":
            self.limit_axis = vec2(1, 0)
        elif vec == "y":
            self.limit_axis = vec2(0, 1)
        else:
            self.limit_axis = vec.normalized()

########################
#        THRUST        #
########################

class thrust:
    def __init__(self, magnitude, origin, p2, offset, offset_rate):
        self.magnitude = magnitude
        self.origin = origin
        self.p2 = p2
        self.offset = offset
        self.direction = self.origin.get_unit_vector_towards(self.p2)
        self.offset_rate = offset_rate

    def move_towards_offset(self, target, dt):
        if target > self.offset and target > self.offset + self.offset_rate * dt:
            self.offset += self.offset_rate * dt
        elif target < self.offset and target < self.offset - self.offset_rate * dt:
            self.offset -= self.offset_rate * dt
        elif (target > self.offset and target < self.offset + self.offset_rate * dt) or (target < self.offset and target > self.offset - self.offset_rate * dt):
            self.offset = target

    def apply_force(self):
        self.direction = self.origin.get_unit_vector_towards(self.p2)
        self.direction = self.direction.rotated(math.radians(self.offset))
        force = self.direction * self.magnitude
        self.origin.apply_force(force)

def space2canvas(space_coords):
    current_cam = get_active_cam()

    canvas_x = ((space_coords.x - current_cam.get_pos().x) / current_cam.get_zoom() + 900 / 2)
    canvas_y = ((-space_coords.y + current_cam.get_pos().y) / current_cam.get_zoom() + 500 / 2)
    return vec2(canvas_x, canvas_y)

def canvas2space(canvas_coords):
    current_cam = get_active_cam()

    space_x = (canvas_coords.x - 900 / 2) * current_cam.get_zoom() + current_cam.get_pos().x
    space_y = -((canvas_coords.y - 500 / 2) * current_cam.get_zoom() - current_cam.get_pos().y)

    return vec2(space_x, space_y)

def sign(number):
    if number >= 0:
        return 1
    else:
        return -1

def clicked_on_canvas(event):
    x = canvas2space(vec2(event.x, 0)).x
    y = canvas2space(vec2(0, event.y)).y

    if click_op.get() == "cp":
        create_point(x, y)

    elif click_op.get() == "dp":
        delete_point(x, y)

    elif click_op.get() == "cl":
        create_link(x, y)

    elif click_op.get() == "dl":
        delete_link(x, y)

    elif click_op.get() == "af":
        apply_force_with_mouse(x, y, "l")

    elif click_op.get() == "rf":
        delete_force(x, y)

    elif click_op.get() == "cm":
        adjust_com_buffer(x, y, "l")

def right_clicked_on_canvas(event):
    x = canvas2space(vec2(event.x, 0)).x
    y = canvas2space(vec2(0, event.y)).y

    if click_op.get() == "af":
        apply_force_with_mouse(x, y, "r")

    elif click_op.get() == "cm":
        adjust_com_buffer(x, y, "r")

def adjust_com_buffer(x, y, click):
    global calc_com_buffer

    if click == "l":
        if not get_closest_point_to_coords(x, y) in calc_com_buffer:
            calc_com_buffer.append(get_closest_point_to_coords(x, y))
    elif click == "r":
        if not len(calc_com_buffer) <= 0 and get_closest_point_to_coords(x, y) in calc_com_buffer:
            calc_com_buffer.remove(get_closest_point_to_coords(x, y))

def calc_com():
    global calc_com_buffer

    com_x = 0
    com_y = 0
    com_mass = 0

    for p in calc_com_buffer:
        com_mass += p.get_mass()
        com_x += p.get_pos().x * p.get_mass()
        com_y += p.get_pos().y * p.get_mass()

    com_x = com_x / com_mass
    com_y = com_y / com_mass

    return (vec2(com_x, com_y), com_mass)

def apply_force_with_mouse(x, y, click):
    global force_buffer

    if click == "r":
        if not get_closest_point_to_coords(x, y) in force_buffer:
            force_buffer.append(get_closest_point_to_coords(x, y))
        else:
            force_buffer.remove(get_closest_point_to_coords(x, y))

    elif click == "l":
        for p in force_buffer:
            create_force(x, y, p)
            force_buffer = []

def create_force(x, y, point):
    global forces
    forces.append(const_force(name_field.get("1.0", "end-1c"), point, point.get_vector_towards(vec2(x, y)) * 0.01))

def delete_force(x, y):
    global forces
    force_tbd = get_closest_force_to_coords(x, y)

    if force_tbd:
        forces.remove(force_tbd)
        del force_tbd

def create_link(x, y):
    global linking_buffer

    if len(linking_buffer) == 0:
        linking_buffer.append(get_closest_point_to_coords(x, y))
    elif len(linking_buffer) == 1:
        if not get_closest_point_to_coords(x, y) == linking_buffer[0]:
            linking_buffer.append(get_closest_point_to_coords(x, y))
            new_link = rigid_link(name_field.get("1.0", "end-1c"), linking_buffer[0], linking_buffer[1],
                                  link_color_field.get("1.0", "end-1c"), float(link_const_field.get("1.0", "end-1c")))
            links.append(new_link)

        linking_buffer = []

def delete_link(x, y):
    link_tbd = get_closest_link_to_coords(x, y)

    if link_tbd:
        links.remove(link_tbd)
        del link_tbd

def toggle_pause():
    global dt
    if dt > 0:
        dt = 0
    else:
        dt = 0.001

def get_closest_point_to_coords(x, y):
    result = None
    for p in points:
        if not result or (vec2(x, y) - p.get_pos()).mag() < (vec2(x, y) - result.get_pos()).mag():
            result = p

    return result

def get_closest_link_to_coords(x, y):
    result = None
    for l in links:
        if not result or (vec2(x, y) - l.get_midpoint()).mag() < (result.get_midpoint() - vec2(x, y)).mag():
            result = l

    return result

def get_closest_force_to_coords(x, y):
    result = None
    for f in forces:
        if not result or ((vec2(x, y) - (f.point.get_pos() + f.force * 100)).mag() < (
                vec2(x, y) - (result.point.get_pos() + result.force * 100)).mag()):
            result = f

    return result

def create_point(x, y):
    new_point = point(name_field.get("1.0", "end-1c"), vec2(x, y), vec2(), "seagreen",
                      float(point_mass_field.get("1.0", "end-1c")), staticPoint.get())
    points.append(new_point)

def delete_point(x, y):
    point_tbd = get_closest_point_to_coords(x, y)

    if point_tbd:

        # if point is an end of a link, delete the link
        # as well
        for l in links:
            if l.p1 == point_tbd or l.p2 == point_tbd:
                links.remove(l)
                del l

        points.remove(point_tbd)
        del point_tbd

root = Tk()
root.title("Mechuilibria SloshTVC")
root.geometry("1150x600")

# label controls
labelsLabel = Label(root, text="Labels")
labelsLabel.grid(row=0, column=0)

pointLabels = IntVar()
linkLabels = IntVar()

staticPoint = IntVar()

pointLabelType = StringVar()
linkLabelType = StringVar()

pointLabelType.set("n")
linkLabelType.set("n")

pointsLabelCheck = Checkbutton(root, text="Points", variable=pointLabels)
pointsLabelCheck.grid(row=1, column=0)

point_label_type_name = Radiobutton(root, text="Names", value="n", var=pointLabelType)
point_label_type_mass = Radiobutton(root, text="Masses", value="m", var=pointLabelType)

point_label_type_name.grid(row=2, column=0)
point_label_type_mass.grid(row=3, column=0)

linkLabelCheck = Checkbutton(root, text="Links", variable=linkLabels)
linkLabelCheck.grid(row=4, column=0)

link_label_type_name = Radiobutton(root, text="Names", value="n", var=linkLabelType)
link_label_type_k = Radiobutton(root, text="Spring Consts.", value="k", var=linkLabelType)

link_label_type_name.grid(row=5, column=0)
link_label_type_k.grid(row=6, column=0)

# pause-resume
pauseResumeButton = Button(root, text="Pause/Resume", command=toggle_pause)
pauseResumeButton.grid(row=7, column=0)

tk_canvas = Canvas(root, width=900, height=500, bg="white")
tk_canvas.grid(row=0, column=1, rowspan=15, columnspan=5)

main_cam = camera("main_cam", vec2(100, 50), 1, "active")

# canvas click
click_op = StringVar(root, "cp")
click_op_cp = Radiobutton(root, text="Create Point", value="cp", var=click_op)
click_op_dp = Radiobutton(root, text="Delete Point", value="dp", var=click_op)
click_op_cl = Radiobutton(root, text="Create Link", value="cl", var=click_op)
click_op_dl = Radiobutton(root, text="Delete Link", value="dl", var=click_op)

click_op_af = Radiobutton(root, text="Apply Force", value="af", var=click_op)
click_op_rf = Radiobutton(root, text="Remove Force", value="rf", var=click_op)

click_op_cm = Radiobutton(root, text="Calc. CoM", value="cm", var=click_op)

click_op_label = Label(root, text="Mouse Click Operation")
click_op_label.grid(row=0, column=6)

click_op_cp.grid(row=1, column=6)
click_op_dp.grid(row=2, column=6)
click_op_cl.grid(row=3, column=6)
click_op_dl.grid(row=4, column=6)

click_op_af.grid(row=5, column=6)
click_op_rf.grid(row=6, column=6)

click_op_cm.grid(row=7, column=6)

instruction = StringVar()
instruction_field = Label(root, textvariable=instruction)
instruction_field.grid(row=8, column=6, rowspan=3, padx=10)

bottom_options_label = Label(root, text="Create Point/Link Options")
bottom_options_label.grid(row=16, column=1)

name_field_label = Label(root, text="Name")
name_field_label.grid(row=17, column=1)
name_field = Text(root, height=1, width=20)
name_field.grid(row=18, column=1)

point_mass_field_label = Label(root, text="Point Mass (kg)")
point_mass_field_label.grid(row=17, column=2)
point_mass_field = Text(root, height=1, width=20)
point_mass_field.grid(row=18, column=2)

point_static_field = Checkbutton(root, text="Static Point", variable=staticPoint)
point_static_field.grid(row=19, column=1)

link_const_field_label = Label(root, text="Link Spring Constant")
link_const_field_label.grid(row=17, column=3)
link_const_field = Text(root, height=1, width=20)
link_const_field.grid(row=18, column=3)

link_color_field_label = Label(root, text="Link Color")
link_color_field_label.grid(row=17, column=4)
link_color_field = Text(root, height=1, width=20)
link_color_field.grid(row=18, column=4)

tk_canvas.bind('<Button-1>', clicked_on_canvas)
tk_canvas.bind('<Button-3>', right_clicked_on_canvas)

# camera controls
root.bind("<Up>", move_current_cam_up)
root.bind("<Down>", move_current_cam_down)
root.bind("<Left>", move_current_cam_left)
root.bind("<Right>", move_current_cam_right)
root.bind("<Control_L>", zoom_current_cam_out)
root.bind("<Shift_L>", zoom_current_cam_in)

# rocket
K_gimbal = 35
K_angvel = 1e-2
max_target_angvel = 0.5
K_orient = 1

rocket_mass = 500
payload_mass = 20
rocket_length = 70
rocket_rigidity = 15e6
rocket_damping = 1e-4
pt_mass = rocket_mass / 14
propellant_mass = 5000
propellant_bumparoundability = 15e4
propellant_sloshcosity = 50

p00 = point("p00", vec2(-2, 0), vec2(), "seagreen", pt_mass)
p01 = point("p01", vec2(-2, 15), vec2(), "seagreen", pt_mass)
p02 = point("p02", vec2(-2, 40), vec2(), "seagreen", pt_mass)
p03 = point("p03", vec2(-2, 50), vec2(), "seagreen", pt_mass)
p04 = point("p04", vec2(-2, 60), vec2(), "seagreen", pt_mass)
p05 = point("p05", vec2(-2, 65), vec2(), "seagreen", pt_mass)

p20 = point("p20", vec2(2, 0), vec2(), "seagreen", pt_mass)
p21 = point("p21", vec2(2, 15), vec2(), "seagreen", pt_mass)
p22 = point("p22", vec2(2, 40), vec2(), "seagreen", pt_mass)
p23 = point("p23", vec2(2, 50), vec2(), "seagreen", pt_mass)
p24 = point("p24", vec2(2, 60), vec2(), "seagreen", pt_mass)
p25 = point("p25", vec2(2, 65), vec2(), "seagreen", pt_mass)

p10 = point("p10", vec2(0, 7), vec2(), "seagreen", propellant_mass * 0.3)
p11 = point("p11", vec2(0, 40-13), vec2(), "seagreen", propellant_mass * 0.4)
p12 = point("p12", vec2(0, 45), vec2(), "seagreen", propellant_mass * 0.1)
p13 = point("p13", vec2(0, 55), vec2(), "seagreen", propellant_mass * 0.2)
p14 = point("p14", vec2(0, 65), vec2(), "seagreen", payload_mass)
p15 = point("p15", vec2(0, 70), vec2(), "seagreen", pt_mass)

pt = point("pt", vec2(0,0), vec2(), "seagreen", pt_mass)

s01 = rigid_link("s0", p00, pt, "skyblue", rocket_rigidity, rocket_damping)
s02 = rigid_link("s0", p20, pt, "skyblue", rocket_rigidity, rocket_damping)
s03 = rigid_link("s0", p01, pt, "skyblue", rocket_rigidity, rocket_damping)
s04 = rigid_link("s0", p21, pt, "skyblue", rocket_rigidity, rocket_damping)
s1 = rigid_link("s1", p01, p21, "skyblue", rocket_rigidity, rocket_damping)
s2 = rigid_link("s2", p02, p22, "skyblue", rocket_rigidity, rocket_damping)
s3 = rigid_link("s3", p03, p23, "skyblue", rocket_rigidity, rocket_damping)
s4 = rigid_link("s4", p04, p24, "skyblue", rocket_rigidity, rocket_damping)

v1 = rigid_link("v1", p00, p01, "skyblue", rocket_rigidity, rocket_damping)
v2 = rigid_link("v2", p01, p02, "skyblue", rocket_rigidity, rocket_damping)
v3 = rigid_link("v3", p02, p03, "skyblue", rocket_rigidity, rocket_damping)
v4 = rigid_link("v4", p03, p04, "skyblue", rocket_rigidity, rocket_damping)
v5 = rigid_link("v5", p04, p05, "skyblue", rocket_rigidity, rocket_damping)
v6 = rigid_link("v6", p20, p21, "skyblue", rocket_rigidity, rocket_damping)
v7 = rigid_link("v7", p21, p22, "skyblue", rocket_rigidity, rocket_damping)
v8 = rigid_link("v8", p22, p23, "skyblue", rocket_rigidity, rocket_damping)
v9 = rigid_link("v9", p23, p24, "skyblue", rocket_rigidity, rocket_damping)
v10 = rigid_link("v10", p24, p25, "skyblue", rocket_rigidity, rocket_damping)

tip1 = rigid_link("tip1", p05, p15, "skyblue", rocket_rigidity, rocket_damping)
tip2 = rigid_link("tip2", p15, p25, "skyblue", rocket_rigidity, rocket_damping)
tip3 = rigid_link("tip3", p05, p25, "skyblue", rocket_rigidity, rocket_damping)

adapter1 = rigid_link("adapter1", p04, p14, "skyblue", rocket_rigidity, rocket_damping)
adapter2 = rigid_link("adapter1", p24, p14, "skyblue", rocket_rigidity, rocket_damping)

c1 = rigid_link("c1", p00, p21, "skyblue", rocket_rigidity, rocket_damping)
c2 = rigid_link("c2", p01, p22, "skyblue", rocket_rigidity, rocket_damping)
c3 = rigid_link("c3", p02, p23, "skyblue", rocket_rigidity, rocket_damping)
c4 = rigid_link("c4", p03, p24, "skyblue", rocket_rigidity, rocket_damping)
c5 = rigid_link("c1", p04, p25, "skyblue", rocket_rigidity, rocket_damping)

c6 = rigid_link("c6", p01, p20, "skyblue", rocket_rigidity, rocket_damping)
c7 = rigid_link("c7", p02, p21, "skyblue", rocket_rigidity, rocket_damping)
c8 = rigid_link("c8", p03, p22, "skyblue", rocket_rigidity, rocket_damping)
c9 = rigid_link("c9", p04, p23, "skyblue", rocket_rigidity, rocket_damping)
c10 = rigid_link("c10", p05, p24, "skyblue", rocket_rigidity, rocket_damping)

pl01 = rigid_link("pl01", p00, p10, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl02 = rigid_link("pl02", p20, p10, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl03 = rigid_link("pl03", p21, p10, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl04 = rigid_link("pl04", p01, p10, "orange", propellant_bumparoundability, propellant_sloshcosity)

pl11 = rigid_link("pl11", p01, p11, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl12 = rigid_link("pl12", p21, p11, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl13 = rigid_link("pl13", p22, p11, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl14 = rigid_link("pl14", p02, p11, "orange", propellant_bumparoundability, propellant_sloshcosity)

pl21 = rigid_link("pl21", p02, p12, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl22 = rigid_link("pl22", p22, p12, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl23 = rigid_link("pl23", p23, p12, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl24 = rigid_link("pl24", p03, p12, "orange", propellant_bumparoundability, propellant_sloshcosity)

pl31 = rigid_link("pl31", p03, p13, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl32 = rigid_link("pl32", p23, p13, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl33 = rigid_link("pl33", p24, p13, "orange", propellant_bumparoundability, propellant_sloshcosity)
pl34 = rigid_link("pl34", p04, p13, "orange", propellant_bumparoundability, propellant_sloshcosity)

f1 = thrust((rocket_mass + propellant_mass) * 30, pt, p15, 0, 25)

cameras = [main_cam]
main_cam.do_zoom(0.2)

floor = ground(-100, "green", 0.5, 0.8)
points = [p00, p01, p02, p03, p04, p05,
          p10, p11, p12, p13, p14, p15,
          p20, p21, p22, p23, p24, p25,
          pt]

links = [s01, s02, s03, s04,
         s1, s2, s3, s4,
         v1, v2, v3, v4, v5, v6, v7, v8, v9, v10,
         tip1, tip2, tip3,
         adapter1, adapter2,
         c1, c2, c3, c4, c5, c6, c7, c8, c9, c10,
         pl01, pl02, pl03, pl04,
         pl11, pl12, pl13, pl14,
         pl21, pl22, pl23, pl24,
         pl31, pl32, pl33, pl34]
forces = []
thrusts = [f1]
force_buffer = []
linking_buffer = []
calc_com_buffer = []
sim_time = 0
cycle = 0

while True:

    if click_op.get() == "cp":
        instruction.set("Click to create point at\nmouse cursor position.\nSet name and mass in\ninput fields.")
    elif click_op.get() == "dp":
        instruction.set("Click to remove point \nclosest to mouse cursor.")
    elif click_op.get() == "cl":
        instruction.set("Click to select points\nto link. Set name and\nspring constant in\ninput fields.")
    elif click_op.get() == "dl":
        instruction.set("Click to remove link\nclosest to mouse cursor.")
    elif click_op.get() == "af":
        instruction.set("Right click to select\npoints to apply force to.\nLeft click to set the\nforce vector.")
    elif click_op.get() == "rf":
        instruction.set("Click to remove force\nclosest to mouse cursor.")
    elif click_op.get() == "cm":
        instruction.set("Left click to choose\nmasses to calculate\ncenter of mass. Right\nclick to remove mass.")

    if not dt == 0:
        floor.apply_force(points)

    if space2canvas(vec2(0, floor.get_height())).y < 500:
        tk_canvas.create_rectangle(-1000, space2canvas(vec2(0, floor.get_height())).y,
                                    1000, 500,
                                    fill=floor.get_color())

    for f in forces:
        tk_canvas.create_line(space2canvas(f.point.get_pos()).x, space2canvas(f.point.get_pos()).y,
                              space2canvas(vec2(f.point.get_pos().x + f.force.x * 100, f.point.get_pos().y)).x,
                              space2canvas(vec2(f.point.get_pos().x, f.point.get_pos().y + f.force.y * 100)).y,
                              fill="blue", arrow=LAST)
        f.apply()

    # TVC
    t = f1

    if t.origin.pos.y < 500:
        desired_flight_angle = 10
    elif t.origin.pos.y < 1500:
        desired_flight_angle = 25
    elif t.origin.pos.y < 5000:
        desired_flight_angle = 45
    else:
        desired_flight_angle = 60

    tip_rvel = t.p2.vel - t.origin.vel
    tip_rpos = t.p2.pos - t.origin.pos
    ang_vel = tip_rvel - tip_rpos.normalized() * tip_rvel.dot(tip_rpos.normalized())
    if ang_vel.x > 0:
        if ang_vel.y < 0:
            angvels = ang_vel.mag() / rocket_length
        else:
            angvels = -ang_vel.mag() / rocket_length
    else:
        if ang_vel.y < 0:
            angvels = ang_vel.mag() / rocket_length
        else:
            angvels = -ang_vel.mag() / rocket_length

    desired_dir = vec2(0, 1).rotated(math.radians(desired_flight_angle)).normalized()
    current_dir = (t.p2.pos - t.origin.pos).normalized()
    current_angle = -math.atan2((t.p2.pos - t.origin.pos).x, (t.p2.pos - t.origin.pos).y)
    correction = (desired_dir - current_dir).normalized()
    # correction_mag = (desired_dir - current_dir).mag()

    correction_mag = desired_flight_angle - math.degrees(current_angle)

    if correction.x > 0:
        if correction.y > 0:
            target_angvel = -K_angvel * correction_mag
        else:
            target_angvel = K_angvel * correction_mag
    else:
        if correction.y > 0:
            target_angvel = -K_angvel * correction_mag
        else:
            target_angvel = K_angvel * correction_mag

    if target_angvel < -max_target_angvel:
        target_angvel = -max_target_angvel
    elif target_angvel > max_target_angvel:
        target_angvel = max_target_angvel

    angvel_error = (angvels - target_angvel) * K_orient
    target_offset = angvel_error * K_gimbal

    t.move_towards_offset(target_offset, dt)

    for t in thrusts:
        ox = space2canvas(t.origin.pos).x
        oy = space2canvas(t.origin.pos).y
        p = t.origin.pos - t.direction * t.magnitude * 0.0005 * main_cam.get_zoom()
        px = space2canvas(p).x
        py = space2canvas(p).y
        n = t.origin.pos - (t.p2.pos - t.origin.pos).normalized() * 20
        nx = space2canvas(n).x
        ny = space2canvas(n).y
        tk_canvas.create_line(ox, oy, px, py, fill="blue", arrow=FIRST)
        tk_canvas.create_line(ox, oy, nx, ny, fill="red", dash=True)
        t.apply_force()

    for p in force_buffer:
        tk_canvas.create_oval(space2canvas(p.get_pos()).x - 5, space2canvas(p.get_pos()).y - 5,
                              space2canvas(p.get_pos()).x + 5, space2canvas(p.get_pos()).y + 5,
                              fill="blue")

    for p in linking_buffer:
        tk_canvas.create_oval(space2canvas(p.get_pos()).x - 5, space2canvas(p.get_pos()).y - 5,
                              space2canvas(p.get_pos()).x + 5, space2canvas(p.get_pos()).y + 5,
                              fill="red")

    if len(calc_com_buffer):
        for p in calc_com_buffer:
            tk_canvas.create_oval(space2canvas(p.get_pos()).x - 5, space2canvas(p.get_pos()).y - 5,
                                  space2canvas(p.get_pos()).x + 5, space2canvas(p.get_pos()).y + 5,
                                  fill="#ffc100")

        com_pos, com_mass = calc_com()
        tk_canvas.create_line(space2canvas(com_pos).x - 8, space2canvas(com_pos).y - 8,
                              space2canvas(com_pos).x + 8, space2canvas(com_pos).y + 8,
                              fill="#ffc100")

        tk_canvas.create_line(space2canvas(com_pos).x - 8, space2canvas(com_pos).y + 8,
                              space2canvas(com_pos).x + 8, space2canvas(com_pos).y - 8,
                              fill="#ffc100")

    cam_pos = main_cam.pos
    uphundred_x = int(math.ceil(cam_pos.x / 100.0)) * 100 + 50
    uphundred_y = int(math.ceil(cam_pos.y / 100.0)) * 100 - 150
    uphundred = vec2(uphundred_x, uphundred_y)
    for i in range(20):
        tk_canvas.create_line(space2canvas(uphundred).x - 20 / main_cam.get_zoom() * i, 0,
                              space2canvas(uphundred).x - 20 / main_cam.get_zoom() * i, 500)

    for i in range(20):
        tk_canvas.create_line(0, space2canvas(uphundred).y - 20 / main_cam.get_zoom() * i,
                              900, space2canvas(uphundred).y - 20 / main_cam.get_zoom() * i)

    for link in links:
        if not dt == 0:
            link.apply_force()
        tk_canvas.create_line(space2canvas(link.p1.get_pos()).x, space2canvas(link.p1.get_pos()).y,
                              space2canvas(link.p2.get_pos()).x, space2canvas(link.p2.get_pos()).y,
                              fill=link.get_color())

    for p in points:
        tk_canvas.create_oval(space2canvas(p.get_pos()).x - 1, space2canvas(p.get_pos()).y - 1,
                              space2canvas(p.get_pos()).x + 1, space2canvas(p.get_pos()).y + 1,
                              fill=p.get_color())

        if not dt == 0:
            p.apply_gravity()
            p.apply_drag()
            p.update_vel()
            p.update_pos()

    tk_canvas.create_text(100, 15, text="Target flight angle: " + str(desired_flight_angle))
    tk_canvas.create_text(100, 30, text="Current flight angle: " + str(round(math.degrees(current_angle), 2)))
    tk_canvas.create_text(100, 45, text="Target angular velocity: " + str(round(target_angvel, 2)))
    tk_canvas.create_text(100, 60, text="Current angular velocity: " + str(round(angvels, 2)))
    tk_canvas.create_text(100, 75, text="Thruster gimbal target: " + str(round(target_offset, 2)))
    tk_canvas.create_text(100, 90, text="Current thruster gimbal: " + str(round(f1.offset, 2)))
    tk_canvas.create_text(100, 105, text="Time: " + str(round(sim_time, 2)))

    if pointLabels.get():
        if pointLabelType.get() == "n":
            for p in points:
                tk_canvas.create_text(space2canvas(p.get_pos()).x - 10, space2canvas(p.get_pos()).y - 10,
                                      text=p.get_name())
        elif pointLabelType.get() == "m":
            for p in points:
                tk_canvas.create_text(space2canvas(p.get_pos()).x - 10, space2canvas(p.get_pos()).y - 10,
                                      text=str(p.get_mass()))

    if linkLabels.get():
        if linkLabelType.get() == "n":
            for l in links:
                tk_canvas.create_text((space2canvas(l.p1.get_pos()).x + space2canvas(l.p2.get_pos()).x) / 2,
                                      (space2canvas(l.p1.get_pos()).y + space2canvas(l.p2.get_pos()).y) / 2,
                                      text=l.get_name(), fill=l.get_color())
        elif linkLabelType.get() == "k":
            for l in links:
                tk_canvas.create_text((space2canvas(l.p1.get_pos()).x + space2canvas(l.p2.get_pos()).x) / 2,
                                      (space2canvas(l.p1.get_pos()).y + space2canvas(l.p2.get_pos()).y) / 2,
                                      text=str(l.get_k()), fill=l.get_color())

    for c in cameras:
        c.set_pos((f1.origin.pos + f1.p2.pos) * 0.5)

    if cycle % 10 == 0:
        root.update()
    tk_canvas.delete("all")

    for p in points:
        p.clear_accel()

    sim_time += dt
    cycle += 1

root.mainloop()
