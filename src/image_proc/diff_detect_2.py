import cv2
import numpy as np
from matplotlib import pyplot as plt


MAX_FEATURES = 500
GOOD_MATCH_PERCENT = 0.15

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

#実行用
if __name__ == '__main__':
    # 画像補正のリファレンス
    img1 = cv2.imread("frame3.png", cv2.IMREAD_COLOR)
    
    # 射影補正ターゲットの画像
    img2 = cv2.imread("frame4.png", cv2.IMREAD_COLOR)
    
    # 射影変換の適用
    imgReg = alignImages(img2, img1)
    


    
    img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2RGB)
    img1 = img1/255

    imgReg = cv2.cvtColor(imgReg,cv2.COLOR_BGR2RGB)
    #plt.imshow(imgReg)
    #plt.show()
    imgReg = imgReg / 255

    dif = cv2.absdiff(img1,imgReg)
    #dif[dif>0.1] = 1

    
    
    plt.imshow(dif)
    plt.show()

    
    # 補正した画像の保存
    #outFilename = "frame5.png"
    #cv2.imwrite(outFilename, imgReg)