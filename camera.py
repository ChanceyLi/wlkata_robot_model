import cv2
import numpy as np
from skimage import morphology
import glob


def capture_pictures():  # 拍照存储
    capture = cv2.VideoCapture(0)
    index = 0
    while True:
        grabbed, image = capture.read()
        cv2.imshow('img', image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('j'):
            index += 1
            filename = 'E:/lenovo/Documents/pythondata/mirobot-py-master/imageForOdd/img' + str(index) + '.jpg'
            cv2.imwrite(filename, image)
            print("write:", filename)
        if key == ord('q'):
            break
    capture.release()
    cv2.destroyAllWindows()


def camera_calibration():  # 获取相机畸变参数
    weight = 9  # 10 - 1
    height = 6  # 7 - 1
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((weight * height, 3), np.float32)
    objp[:, :2] = np.mgrid[0:weight, 0:height].T.reshape(-1, 2)
    objp = objp * 18.1  # 18.1mm

    objpoints = []
    imgpoints = []
    images = glob.glob('E:/lenovo/Documents/pythondata/mirobot-py-master/imageForOdd/*.jpg')
    index = 0
    u, v = 0, 0
    for fname in images:
        image = cv2.imread(fname)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        u, v = image.shape[:2]
        ret, corners = cv2.findChessboardCorners(gray, (weight, height), None)
        if ret:
            print("index:", index)
            index += 1
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners)
            cv2.drawChessboardCorners(image, (weight, height), corners, ret)
            cv2.namedWindow('findCorners', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('findCorners', 640, 480)
            cv2.imshow('findCorners', image)
            cv2.waitKey(200)
    cv2.destroyAllWindows()

    print('正在计算')
    res, mtx, dist, rvecs, tvecs = \
        cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("ret:", res)
    print("matrix内参数矩阵:\n", mtx)
    print("dist畸变值：\n", dist)
    print("rvecs旋转向量外参：\n", rvecs)
    print("tvecs平移向量外参：\n", tvecs)
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (u, v), 0, (u, v))
    print("newcameraMatrix外参", new_camera_matrix)
    return res, mtx, dist, rvecs, tvecs


def reshape_pictures(capture, mtx, dist):  # 还原相机拍摄的照片
    grabbed, frame = capture.read()
    u, v = frame.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (u, v), 0, (u, v))
    dst1 = cv2.undistort(frame, mtx, dist, None, new_camera_matrix)
    map_x, map_y = cv2.initUndistortRectifyMap(mtx, dist, None, new_camera_matrix, (v, u), 5)
    dst2 = cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR)

    x, y, w1, h1 = roi
    dst1 = dst1[y: y + h1, x: x + w1]

    cv2.imshow('dst2', dst2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite("E:/lenovo/Documents/pythondata/mirobot-py-master/imageForOdd/calibrated/img.jpg", dst1)


class Camera:
    def __init__(self):
        self.__lines = []
        self.__lines_picture = 0
        self.__directions = [[1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1], [0, 1], [1, 1]]
        self.__h = 768
        self.__w = 1024
        self.__kernel = np.ones((5, 5), np.uint8)

    def lines(self):
        return self.__lines

    def lines_picture(self):
        return self.__lines_picture

    def get_lines(self, image, min_length):  # 骨架提取加线检测
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        erosion = cv2.erode(gray, self.__kernel, iterations=1)
        _, binary = cv2.threshold(erosion, 200, 255, cv2.THRESH_BINARY_INV)
        # cv2.imwrite("binary.png", binary)

        binary[binary == 255] = 1
        skeleton0 = morphology.skeletonize(binary)
        skeleton = skeleton0.astype(np.uint8) * 255
        # cv2.imshow('skeleton', skeleton)
        # cv2.waitKey(0)
        self.__h = skeleton.shape[0]
        self.__w = skeleton.shape[1]
        my_map = np.zeros((self.__h + 2, self.__w + 2), np.uint8)  # 将骨架扩维，保证搜索不越界
        res = []
        for i in range(self.__h):
            for j in range(self.__w):
                my_map[i + 1][j + 1] = skeleton[i][j]
        next_line = []  # 存储线段中的分叉点
        index = 0
        for j in range(1, self.__w + 1):
            for i in range(1, self.__h + 1):
                if my_map[i][j] == 255:
                    line = [[i, j]]
                    my_map[i][j] = 0
                    new_i, new_j = i, j
                    next_point = True
                    while next_point:
                        next_point = False
                        direction_num = 0
                        for k in self.__directions:
                            if my_map[k[0] + new_i][k[1] + new_j] == 255:
                                if direction_num == 0:
                                    new_i, new_j = k[0] + new_i, k[1] + new_j
                                    my_map[new_i][new_j] = 0
                                    line.append([new_i, new_j])
                                    next_point = True
                                    direction_num += 1
                                else:
                                    next_line.append([k[0] + new_i, k[1] + new_j])
                    if len(line) >= min_length:
                        res.append(line)

                    while index < len(next_line):
                        line = [[next_line[index][0], next_line[index][1]]]
                        if my_map[next_line[index][0]][next_line[index][1]] == 0:
                            index += 1
                        else:
                            my_map[next_line[index][0]][next_line[index][1]] = 0
                            new_i, new_j = next_line[index][0], next_line[index][1]
                            next_point = True
                            while next_point:
                                next_point = False
                                direction_num = 0
                                for k in self.__directions:
                                    if my_map[k[0] + new_i][k[1] + new_j] == 255:
                                        if direction_num == 0:
                                            new_i, new_j = k[0] + new_i, k[1] + new_j
                                            my_map[new_i][new_j] = 0
                                            line.append([new_i, new_j])
                                            next_point = True
                                            direction_num += 1
                                        else:
                                            next_line.append([k[0] + new_i, k[1] + new_j])
                            index += 1
                        if len(line) >= min_length:
                            res.append(line)
        self.__lines = res

    def draw_lines(self):
        pic = np.zeros((self.__h, self.__w, 3), np.uint8)
        img_root = 'E:/lenovo/Documents/pythondata/mirobot-py-master/images/'
        fps = 24
        index = 0
        for line in self.__lines:
            my_color = np.random.randint(128, 255, size=[1, 3])
            for point in line:
                pic[point[0] - 1][point[1] - 1] = my_color
            index += 1
            cv2.imwrite(img_root + str(index) + '.jpg', pic)

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        videoWriter = cv2.VideoWriter('draw_lines.mp4', fourcc, fps, (self.__w, self.__h))
        for i in range(index):
            frame = cv2.imread(img_root + str(i + 1) + '.jpg')
            frame = cv2.erode(255 - frame, self.__kernel, iterations=1)
            cv2.imshow('frame', frame)
            videoWriter.write(frame)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                break
        videoWriter.release()
        cv2.destroyAllWindows()

        pic = 255 - pic
        pic = cv2.erode(pic, self.__kernel, iterations=1)
        self.__lines_picture = pic


img = cv2.imread(r"E:\lenovo\Documents\pythondata\mirobot-py-master\test.png", None)
my_camera = Camera()
my_camera.get_lines(img, 5)
my_camera.draw_lines()
# cv2.imshow("picture", my_camera.lines_picture())
# cv2.waitKey(0)
