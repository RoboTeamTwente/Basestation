import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import PySimpleGUI as sg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import sys
import os
import glob

COLORMAP = 'tab10'


class PlotterGUI:
    def __init__(self, df, filename):
        self.df = df
        self.filename = filename
        self.all_keys = [k for k in self.df.keys() if k != 'time']
        self.all_packets = list(set([k.split('-')[0] for k in self.all_keys]))
        self.all_packets.sort()
        add_menu, remove_menu = [], []
        for p in self.all_packets:
            add_menu.append(p)
            add_menu.append([k.split('-')[1] + "::add-" + k for k in self.all_keys if k.split('-')[0] == p])
            remove_menu.append(p)
            remove_menu.append([])
        remove_menu.append("All::remove-all")
        self.menu_def = [['Add', add_menu], ['Remove', remove_menu]]
        self.use_auto_ylim = True
        self.fig, self.ax, self.handles, self.window = self.init_figs()

    def init_figs(self):
        sg.theme('Dark')
        # define the form layout
        lim_values = [np.round(val, 2) for val in np.arange(0.1, 100, 0.1)]
        layout = [
            [sg.Menu(self.menu_def)],
            [sg.Canvas(size=(640, 480), key='-CANVAS-')],
            [
             sg.Text('  Y-Limits: '),
             sg.Button('Auto', size=(5, 1), button_color=('white', 'green'), key='-YLIM_BUTTON-'),
             sg.Spin(lim_values, 1.0, enable_events=True, disabled=True, key='-YLIM_SPIN-'),
             sg.Text('  '),
             sg.Button('Open in pyplot', size=(13, 1), button_color=('blue', 'white'), key='-OPEN_BUTTON-')
             ],
        ]
        # create the form and show it without the plot
        window = sg.Window("Recording Plotter - " + self.filename, layout, resizable=True, finalize=True)

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
        for k in self.all_keys:
            k1, k2 = k.split('-')[:2]
            labels.append((k, k2 + " {:s}".format(abbrv[k1])))
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
        elif event == '-OPEN_BUTTON-':
            plt.figure()
            plt.xlabel('Time [s]', fontsize=14)
            plt.grid()
            plt.plot(self.df.time, np.zeros(self.df.time.size), '-k', label=None)
            shown_keys = get_shown_keys(self.handles["menu"].MenuDefinition)
            cm = plt.get_cmap(COLORMAP, 10)
            colors = {k: cm(i) for i, k in enumerate(shown_keys)}
            for k in shown_keys:
                plt.plot(self.df.time, self.df[k], color=colors[k], label=k.split('-')[1], lw=1.5)
            plt.legend()
            plt.title(self.filename)
            plt.xlim([self.df.time.min(), self.df.time.max()])

        # Update plots
        shown_keys = get_shown_keys(self.handles["menu"].MenuDefinition)
        cm = plt.get_cmap(COLORMAP, 10)
        colors = {k: cm(i) for i, k in enumerate(shown_keys)}
        for k in self.all_keys:
            ln, = self.handles["lines"][k]
            if k in shown_keys:
                ln.set_xdata(self.df["time"])
                ln.set_ydata(self.df[k])
                ln.set_color(colors[k])
            else:
                ln.set_xdata([])
                ln.set_ydata([])

        manual_lims = None if type(values["-YLIM_SPIN-"]) != np.float64 else (-values["-YLIM_SPIN-"], values["-YLIM_SPIN-"])
        self.ax.set_xlim([self.df.time.min(), self.df.time.max()])
        self.ax.set_ylim(self.compute_limits() if self.use_auto_ylim else manual_lims)
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

    def compute_limits(self, default=1.):
        """ Compute plot limits for a given dataset """
        lim = 0.
        menu_def = self.handles["menu"].MenuDefinition
        t_arr = self.df.time
        for k in get_shown_keys(menu_def):
            arr = [abs(val) for val, t in zip(self.df[k], t_arr) if not np.isnan(val)]
            lim = lim if len(arr) == 0 else max(lim, max(arr))
        lim = default if lim == 0. else lim
        return [-1.1 * lim, 1.1 * lim]


def get_shown_keys(menu_def):
    sk = []
    for i in range(1, len(menu_def[1][1]), 2):
        sk += ["-".join(k.split("-")[-2:]) for k in menu_def[1][1][i]]
    return sk


def draw_figure(canvas, figure, loc=(0, 0)):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg


def make_plot(filepath):
    plt.ion()
    df = pd.read_csv(filepath)
    gui = PlotterGUI(df, filepath)
    while gui.update_plots():
        pass


def get_latest_recording():
    list_of_files = glob.glob('RTP_recordings/*.csv')  # * means all if need specific format then *.csv
    return max(list_of_files, key=os.path.getctime)


if __name__ == "__main__":
    args = sys.argv
    fpath = get_latest_recording() if len(args) < 2 else args[1]
    make_plot(fpath)
