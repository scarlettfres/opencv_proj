import cv2
import numpy as np
import glob
TAUX_FILTRATION=10
FINESSE=0.1# a affiner !!!! TODO 
"""enum CALIB_CB_ADAPTIVE_THRESH = 1, CALIB_CB_NORMALIZE_IMAGE = 2,
       CALIB_CB_FILTER_QUADS = 4, CALIB_CB_FAST_CHECK = 8 """


class Repere(object):
	def __init__(self):
			self.mtx = np.matrix([[817.54075361,0,315.75807001],
 				[0,817.46341799,251.12921578],
				[0,0,1]])
			self.dist= np.array([-0.07120803,0.47729777,-0.00135327,-0.00359282,-0.46284666])
			self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)	
			#termi_criteria permet de stopper traitement donnes quand lim atteinte

			self.objp = np.zeros((6*7,3), np.float32)
			self.objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

			self.axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
			print axis
			self.ret_rvecs=np.array([[0.],[0.],[0.]])
			self.ret_tvecs=np.array([[0.],[0.],[0.]])
			#self.old_vect=[np.array([[0.],[0.],[0.]]),np.array([[0.],[0.],[0.]]),0]
			self.vect_pot=[[np.array([[0.],[0.],[0.]]),np.array([[0.],[0.],[0.]]),0]]
			self.it=0
			#self.axis = np.float([0,0,0],[5,0,0],[0,5,0],[0,0,5])


	def filtre(self,rvecs,tvecs):	# si vect trouve environ egal au current 
		self.it += 1
		exist=0
		print"TAILLE" , len(self.vect_pot)
		print [self.vect_pot[k][2] for k in range(len(self.vect_pot))]

		for i in range (0,len(self.vect_pot)):
			print"============================================"
			print "premiere boucle i = ", i 
			if all(abs(self.vect_pot[i][0].ravel()-rvecs.ravel())<FINESSE) and all(abs(self.vect_pot[i][1].ravel()-tvecs.ravel())<FINESSE):
				self.vect_pot[i][2] += 1
				print "entree dans le if, self.vect_pot[i][2] =", self.vect_pot[i][2]
				exist=1


			if i==(len(self.vect_pot)-1) and exist==0:	# si apres avoir tout parcouru trouve 0 corrsepondance
				print "entree dans le else , taille avant append  vectpot = ", len(self.vect_pot)
				self.vect_pot.append([rvecs,tvecs,0])
				print "entree dans le else , taille APRES append  vectpot = ", len(self.vect_pot)




		print self.it
		if self.it==TAUX_FILTRATION:	
			print "==============> RAZZZZ <================"
			for i in range (0,len(self.vect_pot)):
				if self.vect_pot[i][2]== max([self.vect_pot[k][2] for k in range(len(self.vect_pot))]):
					self.ret_rvecs=self.vect_pot[i][0]
					self.ret_tvecs=self.vect_pot[i][1]
					#self.old_vect[0]=rvecs
					#self.old_vect[1]=tvecs
			del self.vect_pot

			self.vect_pot=[[np.array([[0.],[0.],[0.]]),np.array([[0.],[0.],[0.]]),0]]
			print"deleteddddddd,,," , len(self.vect_pot)
			self.it=0
		return self.ret_rvecs,self.ret_tvecs


	def find(self,cap):
		while(True):
		# Capture frame-by-frame
			r, frame = cap.read() 
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			corn=[]
			ret, corners = cv2.findChessboardCorners(gray, (6,9),8)

			if ret == True:

				#cette fonction la modifie les valeurs de corners
				cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.criteria)# sans ca gros lag 
				# Find the rotation and translation vectors.
				

				rvecs, tvecs, inliers = cv2.solvePnPRansac(self.objp, corners, self.mtx, self.dist)
				# project 3D points to image plane
				rvecs, tvecs = self.filtre(rvecs,tvecs)
				imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.mtx, self.dist)
				
				#print imgpts
				#draw
				cv2.circle(frame, tuple(corners[0].ravel()), 4 ,(255,0,0))
				one = (10,10)
				two=(60,10)
				three=(10,60)
				cv2.line(frame, one, two,(0,0,255))
				cv2.line(frame, one, three,(0,255,0))

				corner = tuple(corners[0].ravel())
				#corner=tuple(imgpts[0].ravel())
				#print "AAAAAAAAAAA", imgpts	#= points des 3 vect a dessiner
				cv2.line(frame, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
				cv2.line(frame, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
				cv2.line(frame, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
				#print "rvec=", rvecs	#ot et traslation par rapport au plan cam
				#print "tvec",tvecs
			cv2.imshow('img',frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
					break




if __name__ == '__main__':
	origine=Repere()
	cap = cv2.VideoCapture(0)
	origine.find(cap)
	cap.release()
	cv2.destroyAllWindows()
        
