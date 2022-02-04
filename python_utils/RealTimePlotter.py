import matplotlib.pyplot as plt
import numpy as np
from multiprocessing import Process, Queue
import time
import sys
import matplotlib
matplotlib.use('qt5agg')

WINDOW_SIZE = 15.   # time window to show in plots [sec]
TIME_DIFF = 1./60   # time difference between sample points [sec]


class RealTimePlotter:
    def __init__(self):
        self.n = int(WINDOW_SIZE / TIME_DIFF)  # number of points to show
        self.time = 0   # current time [sec]
        var_names = ["time", "cmd_w", "bot_w", "cmd_yaw", "bot_yaw"]
        self.data = {k: [np.nan for _ in range(self.n)] for k in var_names}
        self.start_time = None

        # Open new process to do the plotting (at a lower frequency)
        self.queue = Queue()
        p = Process(target=do_plotting, args=(self.queue,))
        p.start()

    def update(self, command, feedback, state_info):
        if self.start_time is None:
            self.start_time = time.time()

        # Update variables
        self.data["cmd_w"] = self.data["cmd_w"][1:] + [command.angle if command.angularControl == 0 else np.nan]
        self.data["bot_w"] = self.data["bot_w"][1:] + [state_info.rateOfTurn]
        self.data["cmd_yaw"] = self.data["cmd_yaw"][1:] + [command.angle if command.angularControl == 1 else np.nan]
        self.data["bot_yaw"] = self.data["bot_yaw"][1:] + [state_info.xsensYaw]
        self.data["time"] = self.data["time"][1:] + [time.time() - self.start_time]

        if self.queue.empty():
            self.queue.put(self.data)


class Plotter:
    def __init__(self, queue):
        self.queue = queue
        self.n = int(WINDOW_SIZE / TIME_DIFF)  # number of points to show

        self.var_names = {"cmd_w": "Angular velocity command",
                          "bot_w": "Angular velocity feedback",
                          "cmd_yaw": "Yaw command",
                          "bot_yaw": "Yaw feedback"}
        self.figs, self.axs, self.lines = self.init_figs()
        self.var_keys = [k for k in self.lines]
        self.start_time = None
        self.data = {}
        plt.ion()

    def init_figs(self):
        colors = {"cmd_w": [0, .5, 0],
                  "bot_w": [0, .9, 0],
                  "cmd_yaw": [0, 0, .3],
                  "bot_yaw": [0, 0, 1]}
        line_styles = {"cmd_w": '-',
                       "bot_w": '-',
                       "cmd_yaw": '-',
                       "bot_yaw": '-'}

        fig1, ax1 = plt.subplots()
        fig2, ax2 = plt.subplots()
        figs = {"cmd_w": fig1, "bot_w": fig1, "cmd_yaw": fig2, "bot_yaw": fig2}
        axs = {"cmd_w": ax1, "bot_w": ax1, "cmd_yaw": ax2, "bot_yaw": ax2}
        lines = {k: axs[k].plot([], [], line_styles[k], color=colors[k], lw=1, label=self.var_names[k])
                 for k in axs}

        # Angular velocity layout
        ax1.set_ylim([-4*np.pi, 4*np.pi])
        ax1.grid()
        ax1.set_ylabel('Angular velocity [rad/s]', fontsize=14)
        ax1.set_xlabel('Time [s]', fontsize=14)
        ax1.legend()

        # Yaw layout
        ax2.set_ylim([-np.pi, np.pi])
        ax2.grid()
        ax2.set_ylabel('Yaw [rad]', fontsize=14)
        ax2.set_xlabel('Time [s]', fontsize=14)
        ax2.legend()
        return figs, axs, lines

    def update_plots(self):
        if not self.queue.empty():
            self.data = self.queue.get()
            if self.start_time is None:
                self.start_time = time.time()
        # Update plots
        for k in self.lines:
            ln, = self.lines[k]
            ln.set_xdata([] if k not in self.data else self.data["time"])
            ln.set_ydata([] if k not in self.data else self.data[k])
            now = 0 if self.start_time is None else time.time() - self.start_time
            self.axs[k].set_xlim([now - WINDOW_SIZE, now])
        plt.pause(0.01)


def do_plotting(queue):
    pltr = Plotter(queue)
    try:
        while True:
            pltr.update_plots()
    except KeyboardInterrupt:
        print("\nExiting by user request.\n")
        sys.exit(0)


def run_demo():
    from roboteam_embedded_messages.python.RobotCommand import RobotCommand
    from roboteam_embedded_messages.python.RobotFeedback import RobotFeedback
    from roboteam_embedded_messages.python.RobotStateInfo import RobotStateInfo

    amp = 1
    t = -1
    rc, rf, rsi = RobotCommand(), RobotFeedback(), RobotStateInfo()
    rtp = RealTimePlotter()
    while True:
        st = time.perf_counter()
        t += 1

        if t % 700 == 0:
            time.sleep(5)

        freq = 10 + np.sin(t / 100)
        amp += (np.random.rand() - 0.5) * 0.1
        amp = min(max(-np.pi, amp), np.pi)
        rsi.xsensYaw = amp * np.sin(t / freq)
        rc.angle = amp * np.cos(t / freq)
        rc.angularControl = 1
        rsi.rateOfTurn = amp / freq * np.cos(t / freq)
        rtp.update(rc, rf, rsi)
        time.sleep(0.01)
        # print("tick: {:.3f} ms".format((time.perf_counter() - st) * 1000))

if __name__ == "__main__":
    run_demo()

