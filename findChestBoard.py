#!/usr/bin/python
import cv2.cv as cv
import sys

filename = sys.argv[1]
im = cv.LoadImage(filename, cv.CV_LOAD_IMAGE_GRAYSCALE)
im3 = cv.LoadImage(filename, cv.CV_LOAD_IMAGE_COLOR)
chessboard_dim = (9, 6)
found_all, corners = cv.FindChessboardCorners( im, chessboard_dim )
print found_all,corners
cv.DrawChessboardCorners( im3, chessboard_dim, corners, found_all )
cv.ShowImage("Chessboard with corners", im3)
cv.WaitKey()