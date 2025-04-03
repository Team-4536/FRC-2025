import cv2
import numpy as np

# global variables go here:
testVar = 0
camX1, camY1 = 0, 75  # Top-left corner
camX2, camY2 = 360, 240  # Bottom-right corner
MINIMUM_SIZE = 50
MAX_SIZE = 750
MAX_WIDTH = 10
guessLeft = False
MIDDLE = 25


# To change a global variable inside a function,
# re-declare it with the 'global' keyword
def incrementTestVar():
    global testVar
    testVar = testVar + 1
    if testVar == 100:
        print("test")
    if testVar >= 200:
        print("print")
        testVar = 0


def drawDecorations(croppedImage):
    cv2.putText(
        croppedImage,
        "Limelight python script!",
        (0, 230),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        1,
        cv2.LINE_AA,
    )


# runPipeline() is called every frame by Limelight's backend.
def runPipeline(croppedImage, llrobot):
    croppedImage = croppedImage[camY1:camY2, camX1:camX2]
    img_hsv = cv2.cvtColor(croppedImage, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, (130, 100, 0), (160, 255, 200))

    contours, _ = cv2.findContours(
        img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    llpython = [0, 0, 0, 0, 0, 0, 0, 0]
    largestContour = []

    if len(contours) > 0:
        cv2.drawContours(croppedImage, contours, -1, 255, 2)
        secondX = -1
        largestX = -1
        if len(contours) >= 2:
            rightMostLeft = []
            leftMostRight = []
            rightLeftestContour = []
            leftRightestContour = []

            for con in contours[2:]:
                area = cv2.contourArea(con)
                lX, lY, width, lH = cv2.boundingRect(con)
                if area < MAX_SIZE and width < MAX_WIDTH and area > MINIMUM_SIZE:
                    if lX < MIDDLE:
                        rightMostLeft.append(con)
                    elif lX > MIDDLE:
                        leftMostRight.append(con)

            leftRightest = -1
            for con in rightMostLeft:
                lX, lY, width, lH = cv2.boundingRect(con)
                if lX > leftRightest:
                    leftRightest = lX
                    leftRightestContour = con
            rightLeftest = 500
            for con in leftMostRight:
                lX, lY, width, lH = cv2.boundingRect(con)
                if lX < rightLeftest:
                    rightLeftest = lX
                    rightLeftestContour = con
            if len(rightLeftestContour) > 0:
                secondX, y2, w2, h2 = cv2.boundingRect(rightLeftestContour)
                cv2.rectangle(
                    croppedImage,
                    (secondX, y2),
                    (secondX + w2, y2 + h2),
                    (0, 255, 255),
                    2,
                )
                llpython = [leftRightest]
            else:
                guessLeft = False
            if len(leftRightestContour) > 0:
                firstX, y1, w1, h1 = cv2.boundingRect(leftRightestContour)
                cv2.rectangle(
                    croppedImage, (firstX, y1), (firstX + w1, y1 + h1), (0, 255, 255), 2
                )
                llpython = [rightLeftest]
            elif guessLeft:
                leftRightest = rightLeftest - 150
            if not (rightLeftest == 500) and not (leftRightest == -1):
                llpython = [rightLeftest, leftRightest]
        else:
            guessLeft = True
            largestContour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largestContour) < MINIMUM_SIZE:
                largestContour = []
            else:
                largestX, y, w, h = cv2.boundingRect(largestContour)
                cv2.rectangle(
                    croppedImage, (largestX, y), (largestX + w, y + h), (0, 255, 255), 2
                )
            if guessLeft:
                leftRightest = largestX - 150

            llpython = [largestX]

    incrementTestVar()
    drawDecorations(croppedImage)

    # make sure to return a contour,
    # an croppedImage to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    return largestContour, croppedImage, llpython
