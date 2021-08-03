from tkinter import Tk, Text, END, INSERT, Label, Entry, Button, filedialog, Canvas
from tkinter.messagebox import showwarning, showerror, askyesno
from PIL import Image, ImageTk
import threading
import cv2
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
    for j in range(6):
        # lb2[j].configure(text="%.2f" % float(position[j]))
        inp2[j].delete(0, END)
        inp2[j].insert(0, position[j])
    if is_legal != [0, 0, 0, 0, 0, 0]:
        showwarning(title="警告", message="角度越界！已更改为边界值！")
        s = "角度越界！"
        txt.insert(END, s)
        txt.insert(INSERT, '\n')


def run_btn2(open_):
    if not open_:
        ret = arm.robot_initial()
        if ret == -1:
            showerror(title="出错", message="请先通电并连接机械臂！")
            s = "出错：请先通电并连接机械臂！"
            txt.insert(END, s)
        else:
            txt.insert(END, "Robot initial successfully!")
            status[0] = True
            position = arm.get_position()
            for j in range(6):
                # lb2[j].configure(text=str(float(position[j])))
                inp2[j].delete(0, END)
                inp2[j].insert(0, position[j])
    else:
        txt.insert(END, "机器人成功启动！")
    txt.insert(INSERT, '\n')


def run_btn3():
    if arm.get_robot_status() is None:
        showerror(title="出错", message="请先开机！")
        s = "出错：请先开机！"
        txt.insert(END, s)
        txt.insert(INSERT, '\n')
    else:
        arm.robot_initial()
        for j in range(arm.size):
            inp[j].delete(0, END)
            inp[j].insert(0, "0")
        run_btn1()
        txt.insert(END, "机器人重启成功！")
        txt.insert(INSERT, '\n')


def run_btn4():
    if arm.get_robot_status() is None:
        showerror(title="出错", message="请先开机！")
        s = "出错：请先开机！"
        txt.insert(END, s)
        txt.insert(INSERT, '\n')
    else:
        run_btn1()
        arm.run_robot()


def run_btn5():
    position = []
    s = '当前姿态为：'
    for j in range(arm.size):
        position.append(float(inp2[j].get()))
        s += str("%.2f" % (position[j])) + " "
    txt.insert(END, s)
    txt.insert(INSERT, '\n')

    is_legal = arm.set_position(position)
    if is_legal != [0, 0, 0, 0, 0, 0]:
        showwarning(title="警告", message="姿态越界！")
        s = "警告：姿态越界！"
        txt.insert(END, s)
        txt.insert(INSERT, '\n')
    else:
        theta = arm.get_angle()
        s = '当前角度值为：'
        for j in range(arm.size):
            # if is_legal[j] == 1:
            #     txt.insert(END, f"theta{str(j+1)} outbound!")
            #     txt.insert(INSERT, '\n')
            inp[j].delete(0, END)
            inp[j].insert(0, theta[j])
            s += str("%.2f" % (theta[j])) + " "
        txt.insert(END, s)
        txt.insert(INSERT, '\n')


root = Tk()
root.title('机械臂控制台')
root.geometry('1280x960')
root.resizable(0, 0)

arm = model.Motivation()
status = [False, False]
# status[0]:机械臂是否开机, status[1]:摄像头是否打开

inp = []
for i in range(arm.size):
    inp.append(Entry(root))
    inp[i].place(relx=0.04 + 0.06 * i, rely=0.1, relwidth=0.05, relheight=0.03)
    inp[i].insert(0, '0')

btn1 = Button(root, text='更新角度值', command=run_btn1)
btn1.place(relx=0.05, rely=0.32, relwidth=0.12, relheight=0.06)

lb1 = []
sh = ['x', 'y', 'z', chr(945), chr(946), chr(947)]
for i in range(3):
    lb1.append(Label(root, text=sh[i]))
    lb1[i].place(relx=0.02, rely=0.15 + 0.05 * i, relwidth=0.05, relheight=0.05)
for i in range(3):
    lb1.append(Label(root, text=sh[i + 3]))
    lb1[i + 3].place(relx=0.2, rely=0.15 + 0.05 * i, relwidth=0.05, relheight=0.05)

lb2 = []
sh2 = [chr(952) + '1', chr(952) + '2', chr(952) + '3', chr(952) + '4', chr(952) + '5', chr(952) + '6']
for i in range(arm.size):
    lb2.append(Label(root, text=sh2[i]))
    lb2[i].place(relx=0.03 + 0.06 * i, rely=0.05, relwidth=0.05, relheight=0.05)

inp2 = []
for i in range(3):
    inp2.append(Entry(root, text='', fg='blue'))
    inp2[i].place(relx=0.07, rely=0.15 + 0.05 * i, relwidth=0.1, relheight=0.05)
for i in range(3):
    inp2.append(Entry(root, text='', fg='blue'))
    inp2[i + 3].place(relx=0.25, rely=0.15 + 0.05 * i, relwidth=0.1, relheight=0.05)

btn2 = Button(root, text='开机', command=lambda: run_btn2(status[0]))
btn2.place(relx=0.5, rely=0.1, relwidth=0.16, relheight=0.08)

btn3 = Button(root, text='复位', command=run_btn3)
btn3.place(relx=0.7, rely=0.1, relwidth=0.16, relheight=0.08)

btn4 = Button(root, text='移动到当前角度', command=run_btn4)
btn4.place(relx=0.5, rely=0.2, relwidth=0.16, relheight=0.08)

btn5 = Button(root, text='更新姿态', command=run_btn5)
btn5.place(relx=0.2, rely=0.32, relwidth=0.12, relheight=0.06)

txt = Text(root)
txt.place(relx=0, rely=0.4, relwidth=0.4, relheight=0.6)

# camera

canvas = Canvas(root, bg='#c4c2c2')
canvas.place(relx=0.4, rely=0.7, relwidth=0.4, relheight=0.3)


def cc():
    capture = cv2.VideoCapture(0)
    s = "摄像头打开成功！"
    txt.insert(END, s)
    txt.insert(INSERT, '\n')
    while status[1]:
        _, frame = capture.read()
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        img = Image.fromarray(img)
        image_file = ImageTk.PhotoImage(img)
        canvas.create_image(0, 0, anchor='nw', image=image_file)
        canvas.image = image_file  # 惊人地发现就此一行代码可以解决视频闪烁的问题


def video_capture(flag):
    if not flag:
        t = threading.Thread(target=cc)
        t.start()
        status[1] = True
        btn6.configure(text='关闭摄像头')
        s = "打开摄像头中......"
        txt.insert(END, s)
        txt.insert(INSERT, '\n')
    else:
        ##
        status[1] = False
        btn6.configure(text='打开摄像头')
        s = "关闭摄像头中......"
        txt.insert(END, s)
        txt.insert(INSERT, '\n')
        s = "摄像头关闭成功！"
        txt.insert(END, s)
        txt.insert(INSERT, '\n')


def closeWindow():  # 关闭窗口函数
    ans = askyesno(title='提示', message='关闭控制台？')
    if ans:
        if status[1]:
            showwarning(title="警告", message="请先关闭摄像头!")
        else:
            root.destroy()
    else:
        return


btn6 = Button(root, text='打开摄像头', command=lambda: video_capture(status[1]))
btn6.place(relx=0.4, rely=0.67, relwidth=0.06, relheight=0.03)
root.protocol('WM_DELETE_WINDOW', closeWindow)


root.mainloop()
