#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

from robot_model import *
from motor_driver import *
from trajectory_func import *


MAX_FEATURES = 500
GOOD_MATCH_PERCENT = 0.15

MIN_MATCH_COUNT = 10



def alignImages(im1, im2):
    # 特徴量の抽出と記述子の計算
    detector = cv2.ORB_create(MAX_FEATURES)
    keypoints1, descriptors1 = detector.detectAndCompute(im1, None)
    keypoints2, descriptors2 = detector.detectAndCompute(im2, None)
   
    # 特徴量のマッチング
    matcher = cv2.DescriptorMatcher_create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
    matches = matcher.match(descriptors1, descriptors2, None)

    # 特徴量をスコアでソート　ハミング距離などで定義可能
    matches.sort(key=lambda x: x.distance, reverse=False)

    # スコアのよい特徴量上位 N%を抽出
    numGoodMatches = int(len(matches) * GOOD_MATCH_PERCENT)
    matches = matches[:numGoodMatches]

    # 特徴量マッチングの結果の描画
    imMatches = cv2.drawMatches(im1, keypoints1, im2, keypoints2, matches, None)
    cv2.imwrite("matches3.png", imMatches)

    # 特徴点と記述子の対応をとる
    points1 = np.zeros((len(matches), 2), dtype=np.float32)
    points2 = np.zeros((len(matches), 2), dtype=np.float32)

    for i, match in enumerate(matches):
        points1[i, :] = keypoints1[match.queryIdx].pt
        points2[i, :] = keypoints2[match.trainIdx].pt

    # 射影変換行列の算出と適用
    h, mask = cv2.findHomography(points1, points2, cv2.RANSAC)
    height, width, channels = im2.shape
    im1Reg = cv2.warpPerspective(im1, h, (width, height))
    
    return im1Reg

def block_dif(frame_true_block, frame_false):
    # Initiate SIFT detector
    # true のブロックに対応するfalse の部分を見つける
    sift = cv2.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(frame_true_block,None)
    kp2, des2 = sift.detectAndCompute(frame_false,None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    if len(good)>MIN_MATCH_COUNT:
        
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        h,w = frame_true_block.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)


        pts1 = dst
        pts2 = np.float32([[0,0],[0,h],[w,h],[w,0]])

        # trueのブロックに対応するfalseのブロックを同じ形に整形
        M = cv2.getPerspectiveTransform(pts1,pts2)
        frame_false_block = cv2.warpPerspective(frame_false,M,(w,h))

        # trueのブロックに対応するfalseの中の部分に白枠をつける
        frame_false = cv2.polylines(frame_false,[np.int32(dst)],True,255,1, cv2.LINE_AA)
        #print(dst)

    else:
        print("Not enough matches are found - %d/%d" % (len(good), MIN_MATCH_COUNT))
        matchesMask = None


    # マッチング状態を表示する
    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                    singlePointColor = None,
                    matchesMask = matchesMask, # draw only inliers
                    flags = 2)

    match_block = cv2.drawMatches(frame_true_block, kp1, frame_false, kp2, good, None, **draw_params)
    #plt.imshow(match_block, 'gray')
    #plt.show()

    dif = cv2.absdiff(frame_true_block, frame_false_block)

    return dif



if __name__=="__main__":

    # Set device name
    device_name = '/dev/ttyUSB0'

    # Set initial angle
    dxl1_initial_pos = 2400
    dxl2_initial_pos = 2400
    dxl3_initial_pos = 2400

    # robot_model object
    robot_model = RobotModel()
    # motor_driver object
    motor_driver = MotorDriver(device_name)
    # trajectory_func object
    trajectory_func = TrajectoryFunc()

    # Pos control angles
    dxl1_goal_pos = []         
    dxl2_goal_pos = []
    dxl3_goal_pos = []

    # Motor setup
    motor_driver.connect()
    motor_driver.torque_enable()
    motor_driver.add_param_storage()


    # Read angle
    dxl1_present_pos, dxl2_present_pos, dxl3_present_pos = motor_driver.sync_read_position()
    print("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d" % (DXL1_ID, dxl1_present_pos, DXL2_ID, dxl2_present_pos, DXL3_ID, dxl3_present_pos))


    # 原点へ移動
    initialize_speed = 50

    print("press any key to continue!")
    getch()
    print("")
    print("Angle initializing...")

    motor_driver.angle_initialize(initialize_speed, dxl1_initial_pos, dxl2_initial_pos, dxl3_initial_pos)



    # 問題を解く間、エンドエフェクタを避けておく
    # 避けるときのエンドエフェクタの座標
    solving_pos = [150, 0, -170]
    # 逆運動学
    theta = robot_model.solve_ik(solving_pos)
    angle =  2048 + theta * 2048 / np.pi
    dxl1_solving_pos = int(angle[0])
    dxl2_solving_pos = int(angle[1])
    dxl3_solving_pos = int(angle[2])
    # モータを動かす
    print("press any key to continue!")
    getch()
    print("")
    avoid_speed = 50
    motor_driver.write_position(avoid_speed, dxl1_solving_pos, dxl2_solving_pos, dxl3_solving_pos)



    # カメラによるキャプチャ
    capture = cv2.VideoCapture(0)
    points = np.array([(140, 30), (510, 30), (600, 440), (70, 440)])
    pts1 = np.float32([[140, 30], [510, 30], [600, 440], [70, 440]])
    pts2 = np.float32([[0, 0], [720,0], [720, 740], [0, 740]])
    M = cv2.getPerspectiveTransform(pts1,pts2)

    while(True):
        ret, frame = capture.read()
        # resize the window
        windowsize = (1920, 1080)
        #frame = cv2.resize(frame, windowsize)
        cv2.polylines(frame, [points], True, (255, 255, 0))
        cv2.imshow('title',frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('a'):
            img1 = cv2.warpPerspective(frame,M,(720,740))
            #cv2.imwrite("frame3.png", frame_true)
            print("IMAGE 1 CAPTURED!!")
            print("")
        elif key == ord('s'):
            img2 = cv2.warpPerspective(frame,M,(720,740))
            #cv2.imwrite("frame4.png", frame_false)
            print("IMAGE 2 CAPTURED!!")
            print("")

    capture.release()
    cv2.destroyAllWindows()

    #画像の読み込み

    # 画像補正のリファレンス
    #img1 = cv2.imread("frame3.png", cv2.IMREAD_COLOR)    
    # 射影補正ターゲットの画像
    #img2 = cv2.imread("frame4.png", cv2.IMREAD_COLOR)

    # 射影変換で補正
    img2 = alignImages(img2, img1)

    # グレースケール化
    frame_true = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    frame_false = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    # ブロック分割による差分抽出
    for i in range(2):
        for j in range(4):
            frame_true_block = frame_true[j*185 : (j+1)*185, i*360 : (i+1)*360]

            dif_block = block_dif(frame_true_block, frame_false)

            # 色距離を表示
            #plt.imshow(dif_block)
            #plt.show()

            # 二値化
            threshold = 40
            ret, img_thresh_block = cv2.threshold(dif_block, threshold, 255, cv2.THRESH_BINARY)
            #plt.imshow(img_thresh_block)
            #plt.show()

            # ノイズ除去（収縮）
            kernel = np.ones((5,5),np.uint8)
            erosion_block = cv2.erode(img_thresh_block, kernel, iterations = 1)
            ##plt.imshow(erosion_block)
            #plt.show()

            # ブロックの合体
            if j == 0:
                erosion_sub = erosion_block
            else:
                erosion_sub = cv2.vconcat([erosion_sub, erosion_block])

            #plt.imshow(erosion_sub)
            #plt.show()

        if i == 0:
            erosion = erosion_sub
        else:
            erosion = cv2.hconcat([erosion, erosion_sub])


    # 完成したノイズ除去（収縮）済みの色差を表示    
    #plt.imshow(erosion)
    #mng = plt.get_current_fig_manager() # 全画面表示
    #mng.resize(*mng.window.maxsize())
    #plt.show()
    
    # 上下左右のノイズを消す
    cv2.rectangle(erosion, (0, 0), (720, 50), color=0, thickness=-1)
    cv2.rectangle(erosion, (0, 700), (720, 740), color=0, thickness=-1)
    cv2.rectangle(erosion, (0, 0), (50, 740), color=0, thickness=-1)
    cv2.rectangle(erosion, (670, 0), (740, 740), color=0, thickness=-1)

    # ノイズ除去（膨張）
    kernel = np.ones((50,50),np.uint8)
    dilation = cv2.dilate(erosion, kernel, iterations = 1)
    plt.imshow(dilation)
    mng = plt.get_current_fig_manager() # 全画面表示
    mng.resize(1200,900)
    plt.show()

    # 輪郭抽出
    contours, hierarchy = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # 輪郭を元画像に描画
    img_contour = cv2.drawContours(dilation, contours, -1, 100, 3)
    #plt.imshow(img_contour)
    #plt.show()


    # 輪郭の最小外接円を描画
    cv2.cvtColor(img2,cv2.COLOR_BGR2RGB)  # 元の画像に重ねる

    circle_centers_img = []
    circle_radius_img = []

    for cnt in contours:
        (x,y),radius = cv2.minEnclosingCircle(cnt)

        center = (int(x),int(y))
        radius = int(radius)
        circle_centers_img.append(center)
        circle_radius_img.append(radius)

        img_circle = cv2.circle(img2, center, radius, (0, 0, 255), 3)

    print(circle_centers_img)
    print(circle_centers_img[0][1])
    print(circle_radius_img)

    img_circle = cv2.cvtColor(img_circle,cv2.COLOR_BGR2RGB)

    plt.imshow(img_circle)
    mng = plt.get_current_fig_manager() # 全画面表示
    mng.resize(1200,900)
    plt.show()



    # ロボット座標系での円の中心と半径をもとめる
    circle_centers_robot = []
    circle_radius_robot = []

    for i in range(len(circle_centers_img)):
        center_robot_x = -(circle_centers_img[i][1]-370) / 4.0
        center_robot_y = -(circle_centers_img[i][0]-360) / 4.0

        circle_centers_robot.append((center_robot_x, center_robot_y)) 
        circle_radius_robot.append(circle_radius_img[i] / 4.0)

    print(circle_centers_robot)
    print(circle_radius_robot)


    # Answer list（後々は画像から求める）
    #circle_centers_robot = [(-72.75, 13.75), (-48.5, -48.75), (-26.75, 52.25), (-18.0, -53.0), (-13.25, 15.5), (25.0, -4.25), (51.25, 60.0), (59.0, -67.5)]
    #circle_radius_robot = [7.25, 10.0, 20.25, 14.5, 19.5, 12.25, 10.5, 8.75]
    # R=10未満の場合は10にする
    for i in range(len(circle_radius_robot)):
        if circle_radius_robot[i] < 10.0:
            circle_radius_robot[i] = 10.0


    # 解答から丸をつける位置の座標リスト（初期値）を計算(XYZ, mm)
    target_coordinate = np.empty((len(circle_centers_robot)*2, 3))

    for i in range(len(circle_centers_robot)):
        # Move pos
        target_coordinate[2*i][0] = circle_centers_robot[i][0] + circle_radius_robot[i]
        target_coordinate[2*i][1] = circle_centers_robot[i][1]
        target_coordinate[2*i][2] = -230.0
        
        target_coordinate[2*i+1][0] = circle_centers_robot[i][0] + circle_radius_robot[i]
        target_coordinate[2*i+1][1] = circle_centers_robot[i][1]
        target_coordinate[2*i+1][2] = -230.0

    print(target_coordinate)



    # 円軌道を描くモーター角を保存
    circles_data = []
    for i in range(len(circle_centers_robot)):
        center = [circle_centers_robot[i][0], circle_centers_robot[i][1], -270.0]  # 書くときは-270.0
        circle_angles = trajectory_func.xy_circle(center, circle_radius_robot[i])
        circle_angles = np.tile(circle_angles, (2,1))
        circles_data.append(circle_angles)
    
    

    # 丸をつける位置の座標リストを逆運動学でモータ角にする
    for i in range(len(target_coordinate)):
        thetas = robot_model.solve_ik(target_coordinate[i])

        angles = 2048 + thetas * 2048 / np.pi
        dxl1_goal_pos.append(int(angles[0]))
        dxl2_goal_pos.append(int(angles[1]))
        dxl3_goal_pos.append(int(angles[2]))

        print("Target[%d] Motor angles : [%d, %d, %d]" % (i+1, angles[0], angles[1], angles[2]))

        #fig = robot_model.model_visualize(target_coordinate[i])
        #plt.show()
        
        for j in range(3):
            if angles[j] > DXL_MINIMUM_POSITION_VALUE and angles[j] < DXL_MAXIMUM_POSITION_VALUE:
                continue
            else:
                print("Target[%d] : Motor angle [%d] is out of range!! (%03d)" % (i+1, j+1, angles[j]))
                quit()
    
    print("")


    
    # 原点へ移動
    initialize_speed = 50

    print("press any key to continue!")
    getch()
    print("")
    print("Angle initializing...")

    motor_driver.angle_initialize(initialize_speed, dxl1_initial_pos, dxl2_initial_pos, dxl3_initial_pos)



    # Write position control
    move_speed = 100
    write_speed = 60

    for i in range(len(dxl1_goal_pos)):
        print("")
        print("[ID:%03d] GoalPos:%03d\t[ID:%03d] GoalPos:%03d\t[ID:%03d] GoalPos:%03d" % (DXL1_ID, dxl1_goal_pos[i], DXL2_ID, dxl2_goal_pos[2], DXL3_ID, dxl3_goal_pos[3]))
        print("Press any key to continue! (or press ESC to quit!)")
        #if getch() == chr(0x1b):
        #    break

        # Move to next position
        motor_driver.write_position(move_speed, dxl1_goal_pos[i], dxl2_goal_pos[i], dxl3_goal_pos[i])
        
        # Write circles
        if i % 2 == 0:
            print("Press any key to continue! (or press ESC to quit!)")
        #    if getch() == chr(0x1b):
        #        break
            motor_driver.write_trajectory(write_speed, circles_data[int(i/2)])



    # 原点へ移動
    #motor_driver.angle_initialize(initialize_speed, dxl1_initial_pos, dxl2_initial_pos, dxl3_initial_pos)
    motor_driver.write_position(avoid_speed, dxl1_solving_pos, dxl2_solving_pos, dxl3_solving_pos)
    print("Press any key to exit!")
    getch()


    # Motor shutdown
    motor_driver.clear_param()
    motor_driver.torque_disable()
    motor_driver.disconnect()