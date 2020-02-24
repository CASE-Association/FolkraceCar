import pyrealsense2 as rs
import numpy as np
import cv2
import math
import time

from main import _camera_car_offset, frame_height, frame_width


class AppState:
    def __init__(self, camera_offset=None):
        """
        Original implementation by Intel Corporation.
        modified by CASE Association.
        ""
        This class models the maps of the car/camera from 3D - 2D
        :param camera_offset: Offset between camera and car origin
        """
        if camera_offset is None:
            camera_offset = [0, 0, 0]
        self.WIN_NAME = 'RealSense'
        self.offset = camera_offset
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array(self.offset, dtype=np.float32)
        self.distance = 3
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation = np.array(self.offset, dtype=np.float32)

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)


out = np.empty((frame_height, frame_width, 3), dtype=np.uint8)


#state = AppState(Car.camera_offset)
state = AppState(_camera_car_offset)

def project(v):
    """project 3d vector array to 2d"""
    h, w = out.shape[:2]
    view_aspect = float(h) / w

    # ignore divide by zero for invalid depth
    with np.errstate(divide='ignore', invalid='ignore'):
        proj = v[:, :-1] / v[:, -1, np.newaxis] * \
               (w * view_aspect, h) + (w / 2.0, h / 2.0)

    # near clipping
    znear = 0.03
    proj[v[:, 2] < znear] = np.nan
    return proj


def view(v):
    """apply view transformation on vector array"""
    return np.dot(v - state.pivot, state.rotation) + state.pivot - state.translation


def line3d(out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
    """draw a 3d line from pt1 to pt2"""
    p0 = project(pt1.reshape(-1, 3))[0]
    p1 = project(pt2.reshape(-1, 3))[0]
    if np.isnan(p0).any() or np.isnan(p1).any():
        return
    p0 = tuple(p0.astype(int))
    p1 = tuple(p1.astype(int))
    rect = (0, 0, out.shape[1], out.shape[0])
    inside, p0, p1 = cv2.clipLine(rect, p0, p1)
    if inside:
        cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)


def grid(out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
    """draw a grid on xz plane"""
    pos = np.array(pos)
    s = size / float(n)
    s2 = 0.5 * size
    for i in range(0, n + 1):
        x = -s2 + i * s
        line3d(out, view(pos + np.dot((x, 0, -s2), rotation)),
               view(pos + np.dot((x, 0, s2), rotation)), color)
    for i in range(0, n + 1):
        z = -s2 + i * s
        line3d(out, view(pos + np.dot((-s2, 0, z), rotation)),
               view(pos + np.dot((s2, 0, z), rotation)), color)


def axes(out, pos, rotation=np.eye(3), size=0.075, thickness=2):
    """draw 3d axes"""
    line3d(out, pos, pos +
           np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
    line3d(out, pos, pos +
           np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
    line3d(out, pos, pos +
           np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)


def frustum(out, intrinsics, color=(0x40, 0x40, 0x40)):
    """draw camera's frustum"""
    orig = view([0, 0, 0])
    w, h = intrinsics.width, intrinsics.height

    for d in range(1, 6, 2):
        def get_point(x, y):
            p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
            line3d(out, orig, view(p), color)
            return p

        top_left = get_point(0, 0)
        top_right = get_point(w, 0)
        bottom_right = get_point(w, h)
        bottom_left = get_point(0, h)

        line3d(out, view(top_left), view(top_right), color)
        line3d(out, view(top_right), view(bottom_right), color)
        line3d(out, view(bottom_right), view(bottom_left), color)
        line3d(out, view(bottom_left), view(top_left), color)


def pointcloud(out, verts, texcoords, color, painter=True):
    """draw point cloud with optional painter's algorithm"""
    if painter:
        # Painter's algo, sort points from back to front

        # get reverse sorted indices by z (in view-space)
        # https://gist.github.com/stevenvo/e3dad127598842459b68
        v = view(verts)
        s = v[:, 2].argsort()[::-1]
        proj = project(v[s])
    else:
        proj = project(view(verts))

    if state.scale:
        proj *= 0.5 ** state.decimate

    h, w = out.shape[:2]

    # proj now contains 2d image coordinates
    j, i = proj.astype(np.uint32).T

    # create a mask to ignore out-of-bound indices
    im = (i >= 0) & (i < h)
    jm = (j >= 0) & (j < w)
    m = im & jm

    cw, ch = color.shape[:2][::-1]
    if painter:
        # sort texcoord with same indices as above
        # texcoords are [0..1] and relative to top-left pixel corner,
        # multiply by size and add 0.5 to center
        v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
    else:
        v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
    # clip texcoords to image
    np.clip(u, 0, ch - 1, out=u)
    np.clip(v, 0, cw - 1, out=v)

    # perform uv-mapping
    out[i[m], j[m]] = color[u[m], v[m]]


def cicle(out, pos, coord, radius, rotation=np.eye(3), color=None, thickness=-1):
    #  fixme wheel angle
    if color is None:
        color = [75, 75, 75]
    [x, y, z] = coord
    pt_hub = view(pos + np.dot((x, y, z), rotation))
    pt_rad = view(pos + np.dot((x, y - radius, z - radius), rotation))
    p0 = project(pt_hub.reshape(-1, 3))[0]
    p1 = project(pt_rad.reshape(-1, 3))[0]
    if np.isnan(p0).any() or np.isnan(p1).any():
        return
    rad = np.subtract(p0, p1)
    p = tuple(p0.astype(int))

    ang = np.tan(np.abs(rad[0] / rad[1])) * 90 / np.pi

    rad = tuple(np.abs(rad).astype(int))
    cv2.ellipse(out, p, rad, ang, 0.0, 360.0, color, thickness)


def car_model(out, pos=None, car_size=None, rotation=np.eye(3), color=(0x40, 0x180, 0x80)):
    """ Draw Car model """
    if pos is None:
        pos = [0, 0, 1]
    if car_size is None:
        car_size = [2, 1.5, 7.5]

    # Generate wheels
    def wheel(side):
        iw = side
        radius = 0.05
        width = 0  # 0.1 add extra circle for depth
        for k in [0, width]:  # inside/outside of wheel
            for jw in [-1, 1]:
                cicle(out, pos, ((1.1 + k) * iw * x, y, 0.65 * jw * z), radius=radius)
                cicle(out, pos, ((1.1 + k) * iw * x, y, 0.65 * jw * z), radius=radius, color=[10, 10, 10],
                      thickness=2)
                if k:
                    pass
                    # pt1 = view(pos + np.dot(((1.1 + k) * iw * x, y+radius, 0.65 * jw * z), rotation))
                    # pt2 = view(pos + np.dot(((1.1) * iw * x, y+ radius, 0.65 * jw * z), rotation))
                    # line3d(out, pt1, pt2, color=[250, 10, 25], thickness=2)
                    # cicle(out, pos, ((1.11 + k) * iw * x, y, 0.65 * jw * z), radius=0.01, color=[0, 10, 25], thickness=4)

    def body():
        # Generate body
        for i in [-1, 1]:
            for j in [-1, 1]:
                line3d(out, view(pos + np.dot((j * x, i * y, -z), rotation)),
                       view(pos + np.dot((j * x, i * y, z), rotation)), color, thickness=2)

                line3d(out, view(pos + np.dot((-x, i * y, j * z), rotation)),
                       view(pos + np.dot((x, i * y, j * z), rotation)), color, thickness=1)

                line3d(out, view(pos + np.dot((i * x, -y, 0.9 * j * z), rotation)),
                       view(pos + np.dot((i * x, y, j * z), rotation)), [40, 120, 40], thickness=2)

    pos = np.array(pos)
    [w, h, l, _] = car_size
    # center of car
    x, y, z = w / 2, h / 2, l / 2
    # get viewed side of car
    side = np.sign(-view(pos + np.dot((x, 0, 0), rotation))[0]).astype(int)
    # fixme sidechange not considering width of car
    # Generate car
    wheel(-side)
    body()
    wheel(side)


def viewer(pipeline, Car, path_planner, camera=True, draw=None):
    """
    OpenCV and Numpy Point cloud Software Renderer

    This sample is mostly for demonstration and educational purposes.
    It really doesn't offer the quality or performance that can be
    achieved with hardware acceleration.

    Usage:
    ------
    Mouse:
        Drag with left button to rotate around pivot (thick small axes),
        with right button to translate and the wheel to zoom.

    Keyboard:
        [p]     Pause
        [r]     Reset View
        [d]     Cycle through decimation values
        [z]     Toggle point scaling
        [c]     Toggle color source
        [s]     Save PNG (./out.png)
        [e]     Export points to ply (./out.ply)
        [q\ESC] Quit
    """

    if draw is None:  # Draw all
        draw = ['car', 'grid', 'axes', 'frustum']


    try:
        if not camera:
            raise RuntimeError

        # Get stream profile and camera intrinsics
        profile = pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        # Processing blocks
        pc = rs.pointcloud()
        decimate = rs.decimation_filter()
        decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
        colorizer = rs.colorizer()
    except RuntimeError as err:
        print(err)
        w, h = 648, 480
        pc = None

    def mouse_cb(event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            state.mouse_btns[0] = True

        if event == cv2.EVENT_LBUTTONUP:
            state.mouse_btns[0] = False

        if event == cv2.EVENT_RBUTTONDOWN:
            state.mouse_btns[1] = True

        if event == cv2.EVENT_RBUTTONUP:
            state.mouse_btns[1] = False

        if event == cv2.EVENT_MBUTTONDOWN:
            state.mouse_btns[2] = True

        if event == cv2.EVENT_MBUTTONUP:
            state.mouse_btns[2] = False

        if event == cv2.EVENT_MOUSEMOVE:

            h, w = out.shape[:2]
            dx, dy = x - state.prev_mouse[0], y - state.prev_mouse[1]

            if state.mouse_btns[0]:
                state.yaw += float(dx) / w * 2
                state.pitch -= float(dy) / h * 2

            elif state.mouse_btns[1]:
                dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
                state.translation -= np.dot(state.rotation, dp)

            elif state.mouse_btns[2]:
                dz = math.sqrt(dx ** 2 + dy ** 2) * math.copysign(0.01, -dy)
                state.translation[2] += dz
                state.distance -= dz

        if event == cv2.EVENT_MOUSEWHEEL:
            dz = math.copysign(0.1, flags)
            state.translation[2] += dz
            state.distance -= dz

        state.prev_mouse = (x, y)

    cv2.namedWindow(state.WIN_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.resizeWindow(state.WIN_NAME, w, h)
    cv2.setMouseCallback(state.WIN_NAME, mouse_cb)


    while True:
        # Grab camera data
        if not state.paused:
            if pc:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()

                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                depth_frame = decimate.process(depth_frame)

                # Grab new intrinsics (may be changed by decimation)
                depth_intrinsics = rs.video_stream_profile(
                    depth_frame.profile).get_intrinsics()
                w, h = depth_intrinsics.width, depth_intrinsics.height

                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                depth_colormap = np.asanyarray(
                    colorizer.colorize(depth_frame).get_data())

                if state.color:
                    mapped_frame, color_source = color_frame, color_image
                else:
                    mapped_frame, color_source = depth_frame, depth_colormap

                points = pc.calculate(depth_frame)
                pc.map_to(mapped_frame)

                # Pointcloud data to arrays
                v, t = points.get_vertices(), points.get_texture_coordinates()
                verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
                texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

        # Render
        now = time.time()

        out.fill(0)

        if 'axes'in draw:
            axes(out, view(np.zeros((1, 3))), state.rotation, size=0.1, thickness=1)

        if pc:
            if not state.scale or out.shape[:2] == (h, w):
                pointcloud(out, verts, texcoords, color_source)
            else:
                tmp = np.zeros((h, w, 3), dtype=np.uint8)
                pointcloud(tmp, verts, texcoords, color_source)
                tmp = cv2.resize(
                    tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
                np.putmask(out, tmp > 0, tmp)
        # todo get grid to show est "driving plane"
        if 'grid' in draw:
            if path_planner:
                ground_pos = path_planner.ground_plane[0]  # Get new est center of plane
                ground_rot = path_planner.ground_plane[1]
                #axes(out, ground_pos, ground_rot, thickness=2)
                #line3d(out,ground_pos, path_planner.ground_plane[2],color=[0, 0, 255], thickness=3)
                grid(out, pos=ground_pos, rotation=ground_rot, size=1, n=15)
            else:
                ground_pos = [state.offset[0],
                       state.offset[1] + Car.size[1] / 2 + Car.size[3],
                       state.offset[2] + 2]
                grid(out, pos=ground_pos, rotation=ground_rot , size=1, n=15)

        # fixme get np.dot to work and hardcoded +2 offset
        if 'car' in draw:
            car_model(out, pos=[state.offset[0], state.offset[1], state.offset[2] + 2], car_size=Car.size)

        if pc: frustum(out, depth_intrinsics)

        if any(state.mouse_btns):
            axes(out, view(state.pivot), state.rotation, thickness=4)

        dt = time.time() - now

        if not pc:
            cv2.setWindowTitle(
                state.WIN_NAME, "FolkraceCar | Blind mode")
        else:
            try:
                cv2.setWindowTitle(
                    state.WIN_NAME, "FolkraceCar | (%dx%d) %dFPS (%.2fms) %s" %
                                    (w, h, 1.0 / dt, dt * 1000, "PAUSED" if state.paused else ""))
                cv2.putText(out, 'Key: [p] Pause '
                                 '[r] Reset View '
                                 '[d] Decimation '
                                 '[z] Scaling '
                                 '[c] Color '
                                 '[s] Save PNG '
                                 '[e] Export cloud', (5, 10), cv2.FONT_HERSHEY_COMPLEX, 0.37, (110, 250, 110), 1)
            except:
                pass

        cv2.imshow(state.WIN_NAME, out)
        key = cv2.waitKey(1)

        if key == ord("r"):
            state.reset()

        if key == ord("p"):
            state.paused ^= True

        if key == ord("d"):
            state.decimate = (state.decimate + 1) % 3
            decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)

        if key == ord("z"):
            state.scale ^= True

        if key == ord("c"):
            state.color ^= True

        if key == ord("s"):
            cv2.imwrite('./out.png', out)

        if key == ord("e"):
            points.export_to_ply('./out.ply', mapped_frame)

        if key in (27, ord("q")) or cv2.getWindowProperty(state.WIN_NAME, cv2.WND_PROP_AUTOSIZE) < 0:
            break



    # Stop streaming
    if pc: pipeline.stop()
