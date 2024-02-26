import numpy as np
from imutils.perspective import four_point_transform
from imutils import contours
import imutils
import cv2
from recognizer import prediction, model

cap = cv2.VideoCapture(1 + cv2.CAP_DSHOW)
result = ''
while True:
    _, img = cap.read()
    img = cv2.blur(img, (3, 3))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edged = cv2.Canny(gray, 50, 200)
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    displayCnt = None

    # выделить рамку вокруг цифр (в физмат корпусе номера кабинетов ввиде: белый фон -> черная рамка-> цифры
    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(approx) == 4:
            displayCnt = approx
            break

    # вырезать roi цифр и скормить сетке
    # roi определяем на edged а вырезаем из gray, т.к модель обучена на чб
    if displayCnt is not None:


        #########
        gray = four_point_transform(gray, displayCnt.reshape(4, 2))
        edged2 = cv2.Canny(gray, 50, 200)
        cnts, _ = cv2.findContours(edged2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.imshow("gray", edged2)
        cv2.waitKey(25)
        #########
        # вычисляем bound box для цифр
        digits = []
        for c in cnts:
            (x, y, w, h) = cv2.boundingRect(c)
            if (20 <= w < 150) and (10 < h < 100):
                image = gray[y-10:y + h+5, x-3:x + w+4]
                digits.append(image)

        if digits:
            tmp = ''
            digits = contours.sort_contours(digits, method="left-to-right")[0]

            for image in digits:
                # еще раз препроцессим подгоняем под mnist
                # if image.shape[0] >0 and image.shape[1]>0:
                #     cv2.waitKey(0)
                #     cv2.imshow("To model nam", image)
                res, prob = prediction(image, model)
                tmp += str(res)
                if len(tmp) == 3:
                    result = tmp[::-1]

    cv2.putText(img, f"Pred: {result}", (40, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0))
    cv2.imshow("canny", edged)
    cv2.imshow("input", img)
    k = cv2.waitKey(25)
    if k == 27:
        break
