import numpy as np
import cv2
import math
from scipy import ndimage

parallel_threshold = 30
same_line_threshold = 110
img_before = cv2.imread('Image10.jpg')
is_left = False
#cv2.imshow("Before", img_before)    
#key = cv2.waitKey(0)
def dis(l, p3):
	p1, p2 = None, None
	for x1, y1, x2, y2 in l:
		p1 = np.array([x1, y1])
		p2 = np.array([x2, y2])
	return abs(np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1))


def determine_line(l, p1, p2):
	d1 = dis(l, p1)
	d2 = dis(l, p2)
	if abs(d1 - d2) > parallel_threshold:	# Not parallel
		return 1
	elif d1 > same_line_threshold or d2 > same_line_threshold: # Parallel but on different side
		return 2
	else: # Same side and parallel
		return 3


def determine_side(l, p):
	x = p[0]
	y = p[1]
	x1 = l[0][0]
	y1 = l[0][1]
	x2 = l[0][2]
	y2 = l[0][3]
	return (x-x1)*(y2-y1)-(y-y1)*(x2-x1)


img_gray = cv2.cvtColor(img_before, cv2.COLOR_BGR2GRAY)
img_edges = cv2.Canny(img_gray, 100, 100, apertureSize=3)
lines = cv2.HoughLinesP(img_edges, 1, math.pi / 173.0, 10, minLineLength=80, maxLineGap=60)
A, B, C, L = None, None, None, None
Middle = None
side = None
angles = []
if lines is None:
	print("No any line detected")
elif len(lines) < 3:
	print("Not enough lines")
else:
	A = lines[0]
	for line in lines:
		for x1, y1, x2, y2 in line:
			p1 = np.array([x1, y1])
			p2 = np.array([x2, y2])
			L = determine_line(A, p1, p2)
			if L == 1: # not parallel
				if B is not None:
					if determine_line(B, p1, p2) == 2:
						Middle = A
						C = line
						cv2.line(img_before, (x1, y1), (x2, y2), (0, 255, 0), 3)
					elif determine_line(B, p1, p2) == 3:
						B[0][0] = (B[0][0] + x1)/2.
						B[0][1] = (B[0][1] + y1)/2.
						B[0][2] = (B[0][2] + x2)/2.
						B[0][3] = (B[0][3] + y2)/2.
						cv2.line(img_before, (x1, y1), (x2, y2), (255, 0, 0), 3)
					else:
						cv2.line(img_before, (x1, y1), (x2, y2), (255, 255, 255), 3)
				else:
					B = line
					cv2.line(img_before, (x1, y1), (x2, y2), (255, 0, 0), 3)
			elif L == 2: # parallel but different side
				C = line
				Middle = B
				cv2.line(img_before, (x1, y1), (x2, y2), (0, 255, 0), 3)
			elif L == 3: # Same side and parallel
				A[0][0] = (A[0][0] + x1)/2.
				A[0][1] = (A[0][1] + y1)/2.
				A[0][2] = (A[0][2] + x2)/2.
				A[0][3] = (A[0][3] + y2)/2.
				cv2.line(img_before, (x1, y1), (x2, y2), (0, 255, 255), 3)
	if Middle is not None:
		angle = math.degrees(math.atan2(Middle[0][3] - Middle[0][1], Middle[0][2] - Middle[0][0]))
		p1 = np.array([C[0][0], C[0][1]])
		p2 = np.array([C[0][2], C[0][3]])
		d1 = dis(Middle, p1)
		d2 = dis(Middle, p2)
		if d1 > d2:
			side = determine_side(Middle, p1)
		else:
			side = determine_side(Middle, p2)
		if side < 0:
			angle = angle + 180.
		img_rotated = ndimage.rotate(img_before, angle)
		print "Angle is {}".format(angle)
		cv2.imwrite('rotated.jpg', img_rotated) 
	else:
		print "Not enough lines"