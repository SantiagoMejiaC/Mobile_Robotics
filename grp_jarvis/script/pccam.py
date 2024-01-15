
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

# charger l'image dans laquelle on cherche l'objet
img_rgb = cv.imread('green.png')
img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)

# charger le template de l'objet à rechercher
template = cv.imread('mask.png',0)

# Récupération des dimensions de l'image
w, h = template.shape[::-1]

# Application du template atching
res = cv.matchTemplate(img_gray,template,cv.TM_CCOEFF_NORMED)

# Sélection des meilleurs matched objects
threshold = 0.8
loc = np.where( res >= threshold)

# Affichage de la boite englobante de chaque objet détecté
for pt in zip(*loc[::-1]):
    cv.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
