import matplotlib.pyplot as plt
import numpy as np
from multiprocessing import Process, Queue
import time
import PySimpleGUI as sg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import datetime
import os

from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand as RobotCommand
from roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback as RobotFeedback
from roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo as RobotStateInfo
from roboteam_embedded_messages.generator.packets import packets

MIN_WINDOW_SIZE = 5.  # minimum time window to show in plots [sec]
MAX_WINDOW_SIZE = 60.  # maximum time window to show in plots [sec]
DEFAULT_WINDOW_SIZE = 30.  # default time window to show in plots [sec]
TIME_DIFF = 1. / 60  # time difference between sample points [sec]

""" POSSIBLE IMPROVEMENTS 
        -> Option to open new window
        -> Use blit for higher frequency plotting
"""


class Data:
    def __init__(self):
        self.n = int(MAX_WINDOW_SIZE / TIME_DIFF)  # number of points to show
        self.time = 0  # current time [sec]
        self.start_time = None

        self.dct = {p: {k[0]: np.nan * np.zeros(self.n) for k in packets[p]} for p in packets}
        self.dct["Extra"] = {k: np.nan * np.zeros(self.n) for k in ["xvel (command)", "yvel (command)",
                                                                    "xvel (feedback)", "yvel (feedback)"]}
        self.time = np.nan * np.zeros(self.n)

        self.all_keys = []
        for p in self.dct:
            self.all_keys += [p+"-"+k for k in self.dct[p]]

    def update(self, pckts: list):
        if self.start_time is None:
            self.start_time = time.time()
        self.time[:-1] = self.time[1:]
        self.time[-1] = time.time() - self.start_time

        for p in pckts:
            tp = str(type(p))
            p_type = tp.split('.')[-1].split('\'')[0]
            for k in self.dct[p_type]:
                if hasattr(p, k):
                    self.dct[p_type][k][:-1] = self.dct[p_type][k][1:]
                    self.dct[p_type][k][-1] = getattr(p, k)

        # add extra's
        self.dct["Extra"]["xvel (command)"], self.dct["Extra"]["yvel (command)"] = self.compute_rc_velocities()
        self.dct["Extra"]["xvel (feedback)"], self.dct["Extra"]["yvel (feedback)"] = self.compute_rf_velocities()

    def get_var(self, key, i=None):
        if key == 'time':
            return self.time
        k1, k2 = key.split('-')
        return self.dct[k1][k2] if i is None else self.dct[k1][k2][i]

    def compute_rsi_velocities(self):
        # wheels2Body
        rad_wheel, front_angle, back_angle = 0.028, 30. / 180. * np.pi, 60. / 180. * np.pi
        denominatorA = rad_wheel / (2 * np.cos(front_angle) ** 2 + np.cos(back_angle) ** 2)
        denominatorB = rad_wheel / (2 * (np.sin(front_angle) + np.sin(back_angle)))

        RF, RB = self.get_var("REM_RobotStateInfo-wheelSpeed1"), self.get_var("REM_RobotStateInfo-wheelSpeed2")
        LB, LF = self.get_var("REM_RobotStateInfo-wheelSpeed3"), self.get_var("REM_RobotStateInfo-wheelSpeed4")
        vx = (np.cos(front_angle) * RF + np.cos(back_angle) * RB - np.cos(back_angle) * LB - np.cos(
            front_angle) * LF) * denominatorA
        vy = (RF - RB - LB + LF) * denominatorB

        # local2Global TODO: check if this corresponds with global x/y definitions
        yaw = self.get_var("REM_RobotStateInfo-xsensYaw")
        vx_g = np.cos(yaw) * vx - np.sin(yaw) * vy
        vy_g = np.sin(yaw) * vx + np.cos(yaw) * vy
        return vx_g, vy_g

    def compute_rc_velocities(self):
        rho, theta = self.get_var("REM_RobotCommand-rho"), self.get_var("REM_RobotCommand-theta")
        vx, vy = rho * np.cos(theta), rho * np.sin(theta)
        return vx, vy

    def compute_rf_velocities(self):
        rho, theta, yaw = self.get_var("REM_RobotFeedback-rho"), self.get_var("REM_RobotFeedback-theta"), self.get_var("REM_RobotStateInfo-xsensYaw")
        theta = theta + yaw  # local to global frame of reference TODO: check whether it should be + or -
        vx, vy = rho * np.cos(theta), rho * np.sin(theta)
        return vx, vy


class RealTimePlotter:
    def __init__(self):
        self.data = Data()
        self.still_running = True

        # Open new process to do the plotting (at a lower frequency)
        self.queue, self.quit_queue = Queue(), Queue()
        self.p = Process(target=do_plotting, args=(self.queue, self.quit_queue))
        self.p.start()

    def update(self, pckts) -> bool:
        self.data.update(pckts)
        if self.queue.empty():
            self.queue.put(self.data)
        if not self.quit_queue.empty():
            self.quit()
        return self.still_running

    def quit(self):
        self.p.terminate()
        while not self.queue.empty():
            self.queue.get()
        self.still_running = False


class PlotterGUI:
    def __init__(self, queue, quit_queue):
        self.queue = queue
        self.quit_queue = quit_queue
        self.n = int(MAX_WINDOW_SIZE / TIME_DIFF)  # number of points to show
        self.start_time = None
        self.data = Data()
        add_menu, remove_menu = [], []
        for p_type in self.data.dct:
            add_menu.append(p_type)
            add_menu.append([k + "::add-" + p_type + "-" + k for k in self.data.dct[p_type]])
            remove_menu.append(p_type)
            remove_menu.append([])
        remove_menu.append("All::remove-all")
        self.menu_def = [['Add', add_menu], ['Remove', remove_menu]]
        self.use_auto_ylim = True
        self.is_recording = False
        self.rec_queue = Queue()
        self.rec_process = None
        self.fig, self.ax, self.handles, self.window = self.init_figs()

    def init_figs(self):
        sg.theme('Dark')
        # define the form layout
        lim_values = [np.round(val, 2) for val in np.arange(0.1, 100, 0.1)]
        layout = [
            [sg.Menu(self.menu_def)],
            [sg.Canvas(size=(640, 480), key='-CANVAS-')],
            [sg.Text('Time window: ', pad=((0, 10), (10, 10))),
             sg.Slider(range=(MIN_WINDOW_SIZE, MAX_WINDOW_SIZE), default_value=DEFAULT_WINDOW_SIZE,
                       size=(20, 10), orientation='h', key='-SLIDER-DATAPOINTS-'),
             sg.Text('  Y-Limits: '),
             sg.Button('Auto', size=(5, 1), button_color=('white', 'green'), key='-YLIM_BUTTON-'),
             sg.Spin(lim_values, 1.0, enable_events=True, disabled=True, key='-YLIM_SPIN-'),
             sg.Text('  '),
             sg.Button('Record', size=(7, 1), button_color=('red', 'white'), key='-REC_BUTTON-')
             ],
        ]
        # create the form and show it without the plot
        window = sg.Window('Real-Time Plotter -- press ESC to terminate', layout, resizable=True, finalize=True,
                           return_keyboard_events=True)

        canvas_elem = window['-CANVAS-']
        canvas = canvas_elem.TKCanvas

        # draw the initial plot in the window
        fig = Figure()
        ax = fig.add_subplot(111)
        ax.set_xlabel("Time [s]")
        ax.grid()
        ax.set_facecolor('lightgrey')
        ax.plot([-100, 1e8], [0, 0], '-k', lw=1)
        fig_agg = draw_figure(canvas, fig)

        abbrv = {"REM_RobotCommand": "(command)", "REM_RobotFeedback": "(feedback)", "REM_RobotStateInfo": "(stateInfo)",
                 "REM_RobotBuzzer": "(buzzer)", "PIDConfiguration": "(pid)", "REM_BasestationStatistics": "(stats)",
                 "REM_BasestationGetStatistics": "(getStats)", "REM_BasestationLog": "(BSLog)", "REM_RobotLog": "(BotLog)",
                 "REM_BasestationGetConfiguration": "(BSGetConfig)", "REM_BasestationConfiguration": "(BSConfig)",
                 "REM_BasestationSetConfiguration": "(BSSetConfig)", "Extra": ""}
        labels = []
        for p in self.data.dct:
            labels += [(p+"-"+k, k + " {:s}".format(abbrv[p])) for k in self.data.dct[p]]
        lines = {k: ax.plot([], [], '-', lw=1, label=lb) for k, lb in labels}
        handles = {"fig": fig_agg, "lines": lines, "menu": layout[0][0], "window": window}
        return fig, ax, handles, window

    def update_plots(self):
        event, values = self.window.read(timeout=10)
        if event in ('Exit', None):
            return 0

        # Handle GUI events
        if "::add" in event or "::remove" in event:
            self.update_menu(event)
        elif event == '-YLIM_BUTTON-':
            self.use_auto_ylim = not self.use_auto_ylim
            bc = ('white', ('red', 'green')[self.use_auto_ylim])
            self.handles["window"].Element('-YLIM_BUTTON-').Update(('Manual', 'Auto')[self.use_auto_ylim], button_color=bc)
            self.handles["window"].Element('-YLIM_SPIN-').Update(disabled=self.use_auto_ylim)
        elif event == '-REC_BUTTON-':
            self.is_recording = not self.is_recording
            bc = (('red', 'white')[self.is_recording], ('white', 'red')[self.is_recording])
            self.handles["window"].Element('-REC_BUTTON-').Update(('Record', 'Stop')[self.is_recording],
                                                                   button_color=bc)
            if self.is_recording:
                self.rec_process = Process(target=do_recording, args=(self.rec_queue,))
                self.rec_process.start()
            else:
                self.rec_queue.put(None)
                self.handles["window"].Element('-REC_BUTTON-').Update('Saving..', button_color=('white', 'blue'))
                self.handles["fig"].draw()
                while self.rec_process.is_alive():
                    pass
                self.handles["window"].Element('-REC_BUTTON-').Update('Record', button_color=('red', 'white'))
                self.rec_process.terminate()
        elif event == "Escape:27":
            self.quit_queue.put('quit')

        # Get data form other process
        if not self.queue.empty():
            self.data = self.queue.get()
            if self.start_time is None:
                self.start_time = time.time()
            if self.is_recording:
                self.rec_queue.put(self.data)

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
        manual_lims = None if type(values["-YLIM_SPIN-"]) != np.float64 else (-values["-YLIM_SPIN-"], values["-YLIM_SPIN-"])
        self.ax.set_ylim(self.compute_limits(now - win_size) if self.use_auto_ylim else manual_lims)
        handles = [ln for ln, in [self.handles["lines"][k] for k in get_shown_keys(self.handles["menu"].MenuDefinition)]]
        self.ax.legend(handles=handles, ncol=-(len(handles) // -2), loc='lower left', bbox_to_anchor=(0, 1.01))
        self.handles["fig"].draw()
        return 1

    def update_menu(self, event):
        menu_def = self.handles["menu"].MenuDefinition
        packet_name = event.split('-')[1]
        packet_idx = -1 if packet_name == 'all' else menu_def[0][1].index(packet_name) + 1
        if "add" in event:
            menu_def[0][1][packet_idx].remove(event)
            menu_def[1][1][packet_idx].append(event.replace("add", "remove"))
        elif event == "All::remove-all":
            for i in range(1, len(menu_def[1][1]), 2):
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
    sk = []
    for i in range(1, len(menu_def[1][1]), 2):
        sk += ["-".join(k.split("-")[-2:]) for k in menu_def[1][1][i]]
    return sk


def do_plotting(*args):
    pltr = PlotterGUI(*args)
    while pltr.update_plots():
        pass


def do_recording(queue):
    times, lines, all_keys = [], [], None
    while True:
        data = queue.get()
        if not data:
            break
        all_keys = ["time"] + data.all_keys
        if len(times) == 0:
            times = [data.time[-1]]
        for i in range(len(data.time)):
            if data.time[i] > times[-1]:
                times.append(data.time[i])
                lines.append([data.time[i]] + [data.get_var(k, i) for k in data.all_keys])
    txt = "" if not all_keys else ",".join(all_keys) + "\n"
    txt += "\n".join([",".join([str(v) for v in row]) for row in lines])
    now = datetime.datetime.now()
    now_str = "{:04d}{:02d}{:02d}{:02d}{:02d}{:02d}".format(now.year, now.month, now.day, now.hour, now.minute, now.second)
    if not os.path.exists('RTP_recordings'):
        os.mkdir('RTP_recordings')
    with open('RTP_recordings/RTPREC' + now_str + '.csv', 'w') as f:
        f.writelines(txt)
    print("Succesfully saved {:.1f} seconds of data!".format(times[-1]-times[1]))


def draw_figure(canvas, figure, loc=(0, 0)):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg


def run_demo():
    rc, rf, rsi = RobotCommand(), RobotFeedback(), RobotStateInfo()
    rc_keys = [elem[0] for elem in packets["REM_RobotCommand"]]
    rf_keys = [elem[0] for elem in packets["REM_RobotFeedback"]]
    rsi_keys = [elem[0] for elem in packets["REM_RobotStateInfo"]]
    rtp = RealTimePlotter()
    while True:
        for obj, keys in zip([rc, rf, rsi], [rc_keys, rf_keys, rsi_keys]):
            for k in keys:
                if hasattr(obj, k):
                    setattr(obj, k, getattr(obj, k) + 0.01*np.random.randn())
        if not rtp.update([rc, rf, rsi]):
            break
        time.sleep(0.01)


if __name__ == "__main__":
    try:
        run_demo()
    except KeyboardInterrupt:
        print("Quitting demo...")
