import matplotlib.pyplot as plt
import numpy as np
from multiprocessing import Process, Queue
import time
import sys
import PySimpleGUI as sg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, FigureCanvasAgg
from matplotlib.figure import Figure
import inspect

from roboteam_embedded_messages.python.RobotCommand import RobotCommand
from roboteam_embedded_messages.python.RobotFeedback import RobotFeedback
from roboteam_embedded_messages.python.RobotStateInfo import RobotStateInfo

MIN_WINDOW_SIZE = 5.  # minimum time window to show in plots [sec]
MAX_WINDOW_SIZE = 60.  # maximum time window to show in plots [sec]
DEFAULT_WINDOW_SIZE = 30.  # default time window to show in plots [sec]
TIME_DIFF = 1. / 60  # time difference between sample points [sec]

""" POSSIBLE IMPROVEMENTS 
        -> Record button to save data that is received (in Elias' format)
        -> Option to switch between 'auto' and 'manual' y-limits
        -> Option to open new window
        -> Use blit for higher frequency plotting
"""


class Data:
    def __init__(self):
        self.n = int(MAX_WINDOW_SIZE / TIME_DIFF)  # number of points to show
        self.time = 0  # current time [sec]
        self.start_time = None
        rc, rf, rsi = RobotCommand(), RobotFeedback(), RobotStateInfo()
        self.rc_keys = [tup[0] for tup in inspect.getmembers(rc)
                        if tup[0][0] != '_' and not inspect.isfunction(tup[1]) and not inspect.ismethod(tup[1])]
        self.rf_keys = [tup[0] for tup in inspect.getmembers(rf)
                        if tup[0][0] != '_' and not inspect.isfunction(tup[1]) and not inspect.ismethod(tup[1])]
        self.rsi_keys = [tup[0] for tup in inspect.getmembers(rsi)
                         if tup[0][0] != '_' and not inspect.isfunction(tup[1]) and not inspect.ismethod(tup[1])]
        self.rsi_keys += ["vel_x", "vel_y"]     # velocities calculated from wheel speeds
        self.rc_keys += ["vel_x", "vel_y"]      # velocities calculated from rho and theta
        self.rf_keys += ["vel_x", "vel_y"]  # velocities calculated from rho and theta
        self.dct = {"rc": {k: np.nan * np.zeros(self.n) for k in self.rc_keys},
                    "rf": {k: np.nan * np.zeros(self.n) for k in self.rf_keys},
                    "rsi": {k: np.nan * np.zeros(self.n) for k in self.rsi_keys},
                    "time": np.nan * np.zeros(self.n)}
        self.all_keys = ["rc-" + k for k in self.rc_keys] + ["rf-" + k for k in self.rf_keys] + ["rsi-" + k for k in
                                                                                                 self.rsi_keys]

    def update(self, rc, rf, rsi):
        if self.start_time is None:
            self.start_time = time.time()
        self.dct["time"][:-1] = self.dct["time"][1:]
        self.dct["time"][-1] = time.time() - self.start_time

        for packet, k1 in zip([rc, rf, rsi], ["rc", "rf", "rsi"]):
            for k2 in self.dct[k1]:
                if hasattr(packet, k2):
                    self.dct[k1][k2][:-1] = self.dct[k1][k2][1:]
                    self.dct[k1][k2][-1] = getattr(packet, k2)
        # Add x- and y-velocities to rsi packet
        self.dct["rsi"]["vel_x"], self.dct["rsi"]["vel_y"] = self.compute_rsi_velocities()
        self.dct["rc"]["vel_x"], self.dct["rc"]["vel_y"] = self.compute_rc_velocities()
        self.dct["rf"]["vel_x"], self.dct["rf"]["vel_y"] = self.compute_rf_velocities()

    def get_var(self, key):
        if key == 'time':
            return self.dct["time"]
        k1, k2 = key.split('-')
        return self.dct[k1][k2]

    def compute_rsi_velocities(self):
        # wheels2Body
        rad_wheel, front_angle, back_angle = 0.028, 30. / 180. * np.pi, 60. / 180. * np.pi
        denominatorA = rad_wheel / (2 * np.cos(front_angle) ** 2 + np.cos(back_angle) ** 2)
        denominatorB = rad_wheel / (2 * (np.sin(front_angle) + np.sin(back_angle)))

        RF, RB = self.get_var("rsi-wheelSpeed1"), self.get_var("rsi-wheelSpeed2")
        LB, LF = self.get_var("rsi-wheelSpeed3"), self.get_var("rsi-wheelSpeed4")
        vx = (np.cos(front_angle) * RF + np.cos(back_angle) * RB - np.cos(back_angle) * LB - np.cos(
            front_angle) * LF) * denominatorA
        vy = (RF - RB - LB + LF) * denominatorB

        # local2Global TODO: check if this corresponds with global x/y definitions
        yaw = self.get_var("rsi-xsensYaw")
        vx_g = np.cos(yaw) * vx - np.sin(yaw) * vy
        vy_g = np.sin(yaw) * vx + np.cos(yaw) * vy
        return vx_g, vy_g

    def compute_rc_velocities(self):
        rho, theta = self.get_var("rc-rho"), self.get_var("rc-theta")
        vx, vy = rho * np.cos(theta), rho * np.sin(theta)
        return vx, vy

    def compute_rf_velocities(self):
        rho, theta, yaw = self.get_var("rf-rho"), self.get_var("rf-theta"), self.get_var("rsi-xsensYaw")
        theta = theta + yaw  # local to global frame of reference TODO: check whether it should be + or -
        vx, vy = rho * np.cos(theta), rho * np.sin(theta)
        return vx, vy


class RealTimePlotter:
    def __init__(self):
        self.data = Data()

        # Open new process to do the plotting (at a lower frequency)
        self.queue = Queue()
        p = Process(target=do_plotting, args=(self.queue,))
        p.start()

    def update(self, command, feedback, state_info):
        self.data.update(command, feedback, state_info)
        if self.queue.empty():
            self.queue.put(self.data)


class PlotterGUI:
    def __init__(self, queue):
        self.queue = queue
        self.n = int(MAX_WINDOW_SIZE / TIME_DIFF)  # number of points to show
        self.start_time = None
        self.data = Data()
        self.menu_def = [['Add', ['RobotCommand', [k + "::add-rc" for k in self.data.rc_keys],
                                  'RobotFeedback', [k + "::add-rf" for k in self.data.rf_keys],
                                  'RobotStateInfo', [k + "::add-rsi" for k in self.data.rsi_keys]]],
                         ['Remove', ['RobotCommand', [], 'RobotFeedback', [], 'RobotStateInfo', [], 'All::remove-all']]]
        self.fig, self.ax, self.handles, self.window = self.init_figs()

    def init_figs(self):
        sg.theme('Dark')
        # define the form layout
        layout = [
            [sg.Menu(self.menu_def)],
            [sg.Canvas(size=(640, 480), key='-CANVAS-')],
            [sg.Text('Time window', pad=((0, 10), (10, 10))),
             sg.Slider(range=(MIN_WINDOW_SIZE, MAX_WINDOW_SIZE), default_value=DEFAULT_WINDOW_SIZE,
                       size=(30, 10), orientation='h', key='-SLIDER-DATAPOINTS-')],
        ]
        # create the form and show it without the plot
        window = sg.Window('Real-Time Plotter', layout, resizable=True, finalize=True)

        canvas_elem = window['-CANVAS-']
        canvas = canvas_elem.TKCanvas

        # draw the initial plot in the window
        fig = Figure()
        ax = fig.add_subplot(111)
        ax.set_xlabel("Time [s]")
        ax.grid()
        ax.set_facecolor('lightgrey')
        fig_agg = draw_figure(canvas, fig)

        lines = {k: ax.plot([], [], '-', lw=1, label=k) for k in self.data.all_keys}
        handles = {"fig": fig_agg, "lines": lines, "menu": layout[0][0]}
        return fig, ax, handles, window

    def update_plots(self):
        # make a bunch of random data points
        event, values = self.window.read(timeout=10)
        if event in ('Exit', None):
            exit(69)

        if "::add" in event or "::remove" in event:
            self.update_menu(event)

        if not self.queue.empty():
            self.data = self.queue.get()
            if self.start_time is None:
                self.start_time = time.time()
        # Update plots
        shown_keys = get_shown_keys(self.handles["menu"].MenuDefinition)
        cm = plt.get_cmap('Dark2', len(shown_keys))
        colors = {k: cm(i) for i, k in enumerate(shown_keys)}
        for k in self.data.all_keys:
            ln, = self.handles["lines"][k]
            if k in shown_keys:
                ln.set_xdata(self.data.get_var("time"))
                ln.set_ydata(self.data.get_var(k))
                ln.set_color(colors[k])
            else:
                ln.set_xdata([])
                ln.set_ydata([])

        now = 0 if self.start_time is None else time.time() - self.start_time
        win_size = int(values['-SLIDER-DATAPOINTS-'])
        self.ax.set_xlim([now - win_size, now])
        self.ax.set_ylim(self.compute_limits(now - win_size))
        handles = [ln for ln, in [self.handles["lines"][k] for k in get_shown_keys(self.handles["menu"].MenuDefinition)]]
        self.ax.legend(handles=handles, ncol=-(len(handles) // -2), loc='lower left', bbox_to_anchor=(0, 1.01))
        self.handles["fig"].draw()

    def update_menu(self, event):
        menu_def = self.handles["menu"].MenuDefinition
        packet_name = event[event.index('-')+1:]
        packet_idx = {"rc": 1, "rf": 3, "rsi": 5, "all": 6}[packet_name]
        if "add" in event:
            menu_def[0][1][packet_idx].remove(event)
            menu_def[1][1][packet_idx].append(event.replace("add", "remove"))
        elif event == "All::remove-all":
            for i in [1, 3, 5]:
                elems = [s for s in menu_def[1][1][i]]
                for elem in elems:
                    menu_def[1][1][i].remove(elem)
                    menu_def[0][1][i].append(elem.replace("remove", "add"))
        elif "remove" in event:
            menu_def[1][1][packet_idx].remove(event)
            menu_def[0][1][packet_idx].append(event.replace("remove", "add"))
        self.handles["menu"].update(menu_def)

    def compute_limits(self, min_time, default=1.):
        """ Compute plot limits for a given dataset """
        lim = 0.
        menu_def = self.handles["menu"].MenuDefinition
        t_arr = self.data.get_var("time")
        for k in get_shown_keys(menu_def):
            arr = [abs(val) for val, t in zip(self.data.get_var(k), t_arr) if t > min_time and not np.isnan(val)]
            lim = lim if len(arr) == 0 else max(lim, max(arr))
        lim = default if lim == 0. else lim
        return [-1.1 * lim, 1.1 * lim]


def get_shown_keys(menu_def):
    return ["rc-" + k[:k.index('::')] for k in menu_def[1][1][1]] \
                 + ["rf-" + k[:k.index('::')] for k in menu_def[1][1][3]] \
                 + ["rsi-" + k[:k.index('::')] for k in menu_def[1][1][5]]


def do_plotting(queue):
    pltr = PlotterGUI(queue)
    try:
        while True:
            pltr.update_plots()
    except KeyboardInterrupt:
        print("\nExiting by user request.\n")
        sys.exit(69)


def draw_figure(canvas, figure, loc=(0, 0)):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg


def run_demo():
    rc, rf, rsi = RobotCommand(), RobotFeedback(), RobotStateInfo()
    rtp = RealTimePlotter()
    while True:
        for obj, keys in zip([rc, rf, rsi], [rtp.data.rc_keys, rtp.data.rf_keys, rtp.data.rsi_keys]):
            for k in keys:
                if hasattr(obj, k):
                    setattr(obj, k, getattr(obj, k) + 0.01*np.random.randn())
        rtp.update(rc, rf, rsi)
        time.sleep(0.01)


if __name__ == "__main__":
    run_demo()
