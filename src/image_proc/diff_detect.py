import numpy as np
import cv2
from matplotlib import pyplot as plt
import time


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



if __name__ == '__main__':


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
        elif key == ord('s'):
            img2 = cv2.warpPerspective(frame,M,(720,740))
            #cv2.imwrite("frame4.png", frame_false)

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
    mng.resize(*mng.window.maxsize())
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
    mng.resize(*mng.window.maxsize())
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