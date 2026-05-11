from math import cos
from math import sin
from math import pi
import os
from dotenv import load_dotenv
load_dotenv()

tcl = str(os.getenv("TCL_LIBRARY"))
tK = str(os.getenv("TK_LIBRARY"))
os.environ["TCL_LIBRARY"] = tcl
os.environ["TK_LIBRARY"] = tK

import tkinter as tk

#differential drive, tank or differential, for simple, traditional wheels. wheel sets can be driven together.
#return list will be [left, right]
def flat_wheel (tank: bool, left_x: float, left_y: float, right_x: float, right_y: float) -> list[float]:
    #control each side independently
    def tank_drive () -> list[float]:
        return [left_y, right_y]

    # left stick forward/backward, right stick turn
    def differential_drive () -> list[float]:
        left = left_y + right_x
        right = left_y - right_x

        #normalize between [1, -1]
        equalizer = max(left, right)
        if equalizer > 1:
            left /= equalizer
            right /= equalizer
        return [left, right]

    if tank:
        return tank_drive()
    else:
        return differential_drive()

#return list will be [front_left, back_left, front_right, back_right]
def holonomic_wheel (field_oriented: bool, left_x: float, left_y: float, right_x: float, right_y: float, current_heading: float) -> list[float]:
    #will NOT normalize forward/backward/side
    def robot_oriented_drive () -> list[float]:
        #when moving forward, front_left will move forward (+)
        #when moving right, front_left will move backward (-)
        #when turning right, front_left will move forward (+)
        #same logic applies to other wheels
        front_left  = left_y - left_x + right_x
        back_left   = left_y + left_x + right_x
        front_right = left_y + left_x - right_x
        back_right  = left_y - left_x - right_x

        #normalize between [-1,1]
        equalizer = max(max(front_left, back_left), max(front_right, back_right))
        if abs(equalizer) > 1:
            front_left /= equalizer
            back_left /= equalizer
            front_right /= equalizer
            back_right /= equalizer
        return [front_left, back_left, front_right, back_right]

    #normalizes so that, even when the bot is turned, forward is still forward.
    def field_oriented_drive () -> list[float]:
        #apply rotational matrix
        lateral_global = left_x * cos(current_heading) - left_y * sin(current_heading)
        forward_global = left_x * sin(current_heading) + left_y * cos(current_heading)

        #when moving forward, front_left will move forward (+)
        #when moving right, front_left will move backward (-)
        #when turning right, front_left will move forward (+)
        #same logic applies to other wheels
        front_left  = forward_global - lateral_global + right_x
        back_left   = forward_global + lateral_global + right_x
        front_right = forward_global + lateral_global - right_x
        back_right  = forward_global - lateral_global - right_x

        #normalize between [-1,1]
        equalizer = max(max(front_left, back_left), max(front_right, back_right))
        if abs(equalizer) > 1:
            front_left /= equalizer
            back_left /= equalizer
            front_right /= equalizer
            back_right /= equalizer
        return [front_left, back_left, front_right, back_right]

    if field_oriented:
        return field_oriented_drive()
    else:
        return robot_oriented_drive()

#return list will be [front_left, back_left, front_right, back_right]
def quad_kiwi_drive (field_oriented: bool, left_x: float, left_y: float, right_x: float, right_y: float, current_heading: float) -> list[float]:
    # will NOT normalize forward/backward/side
    def robot_oriented_drive() -> list[float]:
        front_left  = left_y + left_x + right_x
        back_left   = left_y - left_x + right_x
        front_right = left_y - left_x - right_x
        back_right  = left_y + left_x - right_x

        # normalize between [-1,1]
        equalizer = max(max(front_left, back_left), max(front_right, back_right))
        if abs(equalizer) > 1:
            front_left /= equalizer
            back_left /= equalizer
            front_right /= equalizer
            back_right /= equalizer

        return [front_left, back_left, front_right, back_right]

    # normalizes so that, even when the bot is turned, forward is still forward.
    def field_oriented_drive() -> list[float]:
        # apply rotational matrix
        lateral_global = left_x * cos(current_heading) - left_y * sin(current_heading)
        forward_local = left_x * sin(current_heading) + left_y * cos(current_heading)

        # when moving forward, front_left will move forward (+)
        # when moving right, front_left will move backward (+)
        # when turning right, front_left will move forward (+)
        # same logic applies to other wheels
        front_left  = forward_local + lateral_global + right_x
        back_left   = forward_local - lateral_global + right_x
        front_right = forward_local - lateral_global - right_x
        back_right  = forward_local + lateral_global - right_x

        # normalize between [-1,1]
        equalizer = max(max(front_left, back_left), max(front_right, back_right))
        if abs(equalizer) > 1:
            front_left /= equalizer
            back_left /= equalizer
            front_right /= equalizer
            back_right /= equalizer

        return [front_left, back_left, front_right, back_right]

    if field_oriented:
        return field_oriented_drive()
    else:
        return robot_oriented_drive()

#return list will be [back, left, right]
def triple_kiwi_drive (field_oriented: bool, left_x: float, left_y: float, right_x: float, right_y: float, current_heading: float) -> list[float]:
    # apply rotational matrix if field oriented
    if field_oriented:
        lat_x = left_x * cos(current_heading) - left_y * sin(current_heading)
        left_y = left_x * sin(current_heading) + left_y * cos(current_heading)
        left_x = lat_x
    right = right_x + left_x - left_y
    left = left_y + right_x + left_x
    #back wheel exists only to help turn or strafe
    back = right_x - left_x

    equalizer = max(max(back, left), right)
    if abs(equalizer) > 1:
        back /= equalizer
        left /= equalizer
        right /= equalizer

    return [back, left, right]

#return list will be the wheels counterclockwise, assuming the front of the robot is the point of the normal pentagon and wheel zero
#see README for diagram
def penta_kiwi_drive (field_oriented:bool, left_x: float, left_y: float, right_x: float, right_y: float, current_heading: float) -> list[float]:
    # apply rotational matrix if field oriented
    if field_oriented:
        lat_x = left_x * cos(current_heading) - left_y * sin(current_heading)
        left_y = left_x * sin(current_heading) + left_y * cos(current_heading)
        left_x = lat_x

    zero = left_x + right_x
    one = right_x + left_x - left_y
    two = right_x - left_y - left_x
    three = left_y - left_x + right_x
    four = left_y+ right_x+ left_x

    return [zero, one, two, three, four]




def interactive_test () :
    wheels = int(input(
"""
Select a wheel set:
 - Normal  (1)
 - Mecanum (2)
 - Kiwi (3)
"""
    ))

    drive = int(input(
        " - Field oriented (1)\n - Robot oriented (2)" if wheels != 1 else " - Tank (1)\n - Differential (2)\n"
    ))

    def create_popup():
        root = tk.Tk()
        root.title("Drive Visualizer")

        canvas = tk.Canvas(root, width=400, height=400, bg="white")
        canvas.pack()

        state = {
            "a": 0, "d": 0, "w": 0, "s": 0,
            "left": 0, "right": 0, "up": 0, "down": 0,
            "heading_deg": 0.0,
        }

        def axis(name1, name2):
            return state[name1] - state[name2]

        def wheel_color(value):
            value = max(-1.0, min(1.0, value))

            if value < 0:
                t = -value
                r = 255
                g = int(255 * (1 - t))
            else:
                t = value
                r = int(255 * (1 - t))
                g = 255

            return f"#{r:02x}{g:02x}00"

        def rotate_point(dx, dy, angle):
            c = cos(angle)
            s = sin(angle)
            return dx * c - dy * s, dx * s + dy * c

        def rect_points(cx, cy, w, h, angle):
            hw = w / 2
            hh = h / 2
            pts = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]

            out = []
            for dx, dy in pts:
                rx, ry = rotate_point(dx, dy, angle)
                out.extend((cx + rx, cy + ry))
            return out

        def render():
            canvas.delete("all")

            cx = 200
            cy = 200
            heading = state["heading_deg"] * pi / 180

            left_x = axis("d", "a")
            right_x = axis("right", "left")
            left_y = axis("w", "s")
            right_y = axis("up", "down")

            if wheels == 1:
                powers = flat_wheel(
                    tank=(drive == 1),
                    left_x=left_x,
                    left_y=left_y,
                    right_x=right_x,
                    right_y=right_y,
                )
                wheel_values = [powers[0], powers[0], powers[1], powers[1]]
            if wheels == 2:
                wheel_values = holonomic_wheel(
                    field_oriented=(drive == 1),
                    left_x=left_x,
                    left_y=left_y,
                    right_x=right_x,
                    right_y=right_y,
                    current_heading=heading,
                )
            if wheels == 3:
                wheel_values = quad_kiwi_drive(
                    field_oriented=(drive == 1),
                    left_x=left_x,
                    left_y=left_y,
                    right_x=right_x,
                    right_y=right_y,
                    current_heading=heading,
                )


            # body
            body_size = 90
            canvas.create_polygon(
                rect_points(cx, cy, body_size, body_size, heading),
                fill="lightgray",
                outline="black",
            )

            # front direction indicator
            front_x, front_y = rotate_point(0, -body_size / 2, heading)

            canvas.create_line(
                cx,
                cy,
                cx + front_x,
                cy + front_y,
                width=4,
                fill="blue",
            )

            canvas.create_oval(
                cx + front_x - 5,
                cy + front_y - 5,
                cx + front_x + 5,
                cy + front_y + 5,
                fill="blue",
            )

            # wheels: front-left, back-left, front-right, back-right
            wheel_w = 16
            wheel_h = 28
            offset_x = body_size / 2 + wheel_w / 2
            offset_y = body_size / 4

            wheel_positions = [
                (-offset_x, -offset_y),
                (-offset_x, offset_y),
                (offset_x, -offset_y),
                (offset_x, offset_y),
            ]

            for value, (dx, dy) in zip(wheel_values, wheel_positions):
                wx, wy = rotate_point(dx, dy, heading)
                canvas.create_polygon(
                    rect_points(cx + wx, cy + wy, wheel_w, wheel_h, heading),
                    fill=wheel_color(value),
                    outline="black",
                )

        def key_change(event, value):
            key = event.keysym.lower()
            if key in state:
                state[key] = value
                render()

        def set_heading(value):
            state["heading_deg"] = float(value)
            render()

        heading_slider = tk.Scale(
            root,
            from_=-180,
            to=180,
            orient="horizontal",
            label="Heading",
            command=set_heading,
            resolution=1,
        )
        heading_slider.pack(fill="x")
        heading_slider.set(0)

        root.bind_all("<KeyPress>", lambda e: key_change(e, 1))
        root.bind_all("<KeyRelease>", lambda e: key_change(e, 0))

        root.focus_force()
        render()
        root.mainloop()
    create_popup()


interactive_test()