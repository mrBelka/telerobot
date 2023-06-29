import cv2
import numpy as np
from keras.models import load_model


def prediction(image, model):
    try:
        im = cv2.resize(image, (20, 20), cv2.INTER_AREA)
        im = cv2.copyMakeBorder(im, 4, 4, 4, 4, cv2.BORDER_CONSTANT, None, value=255)
        img = np.array(im).reshape(1, 28, 28, 1)
    except Exception:
        return (0, 0)
    predict = model.predict(img)
    prob = np.amax(predict)
    # class_index = model.predict_classes(img)
    class_index = np.argmax(predict, axis=1)
    res = class_index[0]
    if prob < 0.75:
        res = 0
        prob = 0
    return res, prob

model = load_model('Package\digits.h5')
if __name__ == "__main__":
    cap = cv2.VideoCapture(1 + cv2.CAP_DSHOW)
    WIDTH = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    HEIGHT = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    while True:
        _, frame = cap.read()
        frame_copy = frame.copy()

        bbox_size = (60, 60)
        bbox = [(int(WIDTH // 2 - bbox_size[0] // 2), int(HEIGHT // 2 - bbox_size[1] // 2)),
                (int(WIDTH // 2 + bbox_size[0] // 2), int(HEIGHT // 2 + bbox_size[1] // 2))]

        img_cropped = frame[bbox[0][1]:bbox[1][1], bbox[0][0]:bbox[1][0]]
        img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY)
        img_gray = cv2.resize(img_gray, (200, 200))
        cv2.imshow("cropped", img_gray)

        result, prob, ne = prediction(img_gray, model)
        cv2.putText(frame_copy, f"Pred: {result}", (40, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0))

        cv2.rectangle(frame_copy, bbox[0], bbox[1], (0, 0, 255), 3)

        cv2.imshow("input", frame_copy)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cv2.destroyAllWindows()
