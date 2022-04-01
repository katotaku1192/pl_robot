import numpy as np
import cv2
from matplotlib import pyplot as plt

# 画像補正のリファレンス
im1 = cv2.imread("cropped3.jpg", cv2.IMREAD_COLOR)
    
# 射影補正ターゲットの画像
im2 = cv2.imread("3.jpg", cv2.IMREAD_COLOR)

# 特徴量の抽出と記述子の計算
detector = cv2.ORB_create(500)
keypoints1, descriptors1 = detector.detectAndCompute(im1, None)
keypoints2, descriptors2 = detector.detectAndCompute(im2, None)

# 特徴量のマッチング
matcher = cv2.DescriptorMatcher_create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
matches = matcher.match(descriptors1, descriptors2, None)

# 特徴量をスコアでソート　ハミング距離などで定義可能
matches.sort(key=lambda x: x.distance, reverse=False)

# スコアのよい特徴量上位 N%を抽出
numGoodMatches = int(len(matches) * 0.15)
matches = matches[:numGoodMatches]

# 特徴量マッチングの結果の描画
imMatches = cv2.drawMatches(im1, keypoints1, im2, keypoints2, matches, None)
#plt.imshow(imMatches)
#plt.show()
cv2.imwrite("matches2.png", imMatches)

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

plt.imshow(im1Reg)
plt.show()