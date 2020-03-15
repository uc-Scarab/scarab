import tkinter


class App:
    def __init__(self, master):

        self.master = master
        width = 500
        height = 300
        xoffset = 0
        yoffset = 0

        self.master.geometry('%dx%d%+d%+d' % (width, height, xoffset, yoffset))
        self.dynamixel = tkinter.Scale(master, from_=0, to_=4095, orient="horizontal")

        self.dynamixel.grid(row=0, column=0)


root = tkinter.Tk()

app = App(root)

root.mainloop()
