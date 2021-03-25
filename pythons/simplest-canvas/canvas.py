"""
Done:
- Split code into seperated modules.
- Database thread.
- Show detail information.
- Nice time stamp.
- Auto adjust the image size.
- Select list item to show the specified one.
- Manage the real list.
- Show received data.
- Click list item.
- Image refreshing.
- Threads communication.
"""
import tkinter as _tkinter
import math as _math


class VisualizerPanel(_tkinter.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.__master = master
        self.__canvas_width = 400
        self.__canvas_height = 300
        self.__current_image = None
        self.__current_selection = None
        self.__histories = []
        self.pack()
        self.__create_widgets()
        self.__master.title('Visualizer')

    def __create_widgets(self):
        self.__master.protocol('WM_DELETE_WINDOW', self.__quit)
        self.__create_image()
        self.__create_buttons()

    def __create_image(self):
        self.__canvas = _tkinter.Canvas(
            self, width=self.__canvas_width,
            height=self.__canvas_height,
        )
        self.__canvas.pack(side=_tkinter.LEFT, expand=1)
        _draw(self.__canvas)

    def __create_buttons(self):
        quit_button = _tkinter.Button(
            self, text='Quit', fg='black',
            command=self.__quit,
        )
        quit_button.pack(side=_tkinter.BOTTOM)

    def __quit(self):
        self.__is_listening = False
        self.__master.destroy()


def _draw(canvas: _tkinter.Canvas):
    n = 320
    n_1 = n - 1
    last_x = 0
    last_y = 0
    for i in range(n):
        y = 300 - _math.sqrt((1 - _math.cos(_math.pi * 2 * (i / n_1))) * 0.5) * 300
        canvas.create_line(last_x, last_y, i, y)
        last_x = i
        last_y = y


def __main():
    gui = VisualizerPanel(_tkinter.Tk())
    gui.mainloop()


if __name__ == '__main__':
    __main()
