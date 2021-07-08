from tkinter import *
import model


def run_btn1():
    theta = []
    s = '当前角度值为：'
    for j in range(arm.size):
        theta.append(float(inp[j].get()))

    is_legal = arm.set_angle(theta)
    theta = arm.get_angle()
    for j in range(arm.size):
        if is_legal[j] == 1:
            txt.insert(END, f"theta{str(j + 1)} outbound!")
            txt.insert(INSERT, '\n')
            inp[j].delete(0, END)
            inp[j].insert(0, theta[j])
        s += str("%.2f" % (theta[j])) + ' '
    txt.insert(END, s)
    txt.insert(INSERT, '\n')
    position = arm.get_position()
    for j in range(3):
        lb2[j].configure(text="%.2f" % float(position[j][0]))


def run_btn2(open_):
    if not open_:
        arm.robot_initial()
        txt.insert(END, "Robot initial successfully!")
        status[0] = True
        position = arm.get_position()
        for j in range(3):
            lb2[j].configure(text=str(float(position[j][0])))
    else:
        txt.insert(END, "Robot already initialized!")
    txt.insert(INSERT, '\n')


def run_btn3():
    arm.robot_initial()
    txt.insert(END, "Robot restart successful!")
    txt.insert(INSERT, '\n')


def run_btn4():
    run_btn1()
    arm.run_robot_angle()


root = Tk()
root.title('机械臂控制台')
root.geometry('640x480')
arm = model.Motivation([0, 0, 0, 0, 0, 0])
status = [False]

inp = []
for i in range(arm.size):
    inp.append(Entry(root))
    inp[i].place(relx=0.04 + 0.06 * i, rely=0.1, relwidth=0.05, relheight=0.05)
    inp[i].insert(0, '0')

btn1 = Button(root, text='更新角度值', command=run_btn1)
btn1.place(relx=0.155, rely=0.2, relwidth=0.12, relheight=0.06)

lb1 = []
sh = ['x', 'y', 'z']
for i in range(3):
    lb1.append(Label(root, text=sh[i] + ':'))
    lb1[i].place(relx=0.02, rely=0.3 + 0.05 * i, relwidth=0.05, relheight=0.05)

lb2 = []
for i in range(3):
    lb2.append(Label(root, text='', fg='blue'))
    lb2[i].place(relx=0.07, rely=0.3 + 0.05 * i, relwidth=0.1, relheight=0.05)


btn2 = Button(root, text='开机', command=lambda: run_btn2(status[0]))
btn2.place(relx=0.5, rely=0.1, relwidth=0.16, relheight=0.08)

btn3 = Button(root, text='复位', command=run_btn3)
btn3.place(relx=0.7, rely=0.1, relwidth=0.16, relheight=0.08)

btn4 = Button(root, text='移动到当前角度', command=run_btn4)
btn4.place(relx=0.5, rely=0.2, relwidth=0.16, relheight=0.08)

txt = Text(root)
txt.place(relx=0, rely=0.6, relwidth=0.4, relheight=0.4)

root.mainloop()
