import tkinter


class App:
    def __init__(self, master):

        self.master = master

        self.dynamixel = tkinter.Scrollbar(master)

        self.dynamixel2_entry = tkinter.Entry()

        self.dynamixel1.grid(row=0, column=0)


root = tkinter.Tk()

app = App(root)

root.mainloop()
