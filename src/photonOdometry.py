import photonlibpy
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from ntcore import NetworkTableInstance
import wpimath.geometry
import numpy
import robotpy
import wpilib

photonTable = NetworkTableInstance.getDefault()
aprilTagX = [
    0,
    657.37,
    657.37,
    455.15,
    365.20,
    365.20,
    530.49,
    546.87,
    530.49,
    497.77,
    481.39,
    497.77,
    33.51,
    33.51,
    325.68,
    325.68,
    235.73,
    160.39,
    144.00,
    160.39,
    193.10,
    209.49,
    193.10,
]
aprilTagY = [
    0,
    25.80,
    291.20,
    317.15,
    241.64,
    75.39,
    130.17,
    158.50,
    186.83,
    186.83,
    158.50,
    130.17,
    25.80,
    291.20,
    241.64,
    75.39,
    -0.15,
    130.17,
    158.50,
    186.83,
    186.83,
    158.50,
    130.17,
]
aprilTagDegrees = [
    -180,
    126,
    234,
    270,
    0,
    0,
    300,
    0,
    60,
    120,
    180,
    240,
    54,
    306,
    180,
    180,
    90,
    240,
    180,
    120,
    60,
    0,
    300,
]


class photonVision:

    photonTable = NetworkTableInstance.getDefault()

    def __init__(self, cameraName, camPitch, intCamX, intCamY, intCamZ):
        self.cameraNameReal = cameraName

        self.camera = PhotonCamera(cameraName)
        kRobotToCam = wpimath.geometry.Transform3d(
            wpimath.geometry.Translation3d(intCamX, intCamY, intCamZ),
            wpimath.geometry.Rotation3d.fromDegrees(0.0, 0.0, camPitch),
        )
        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded),
            PoseStrategy.LOWEST_AMBIGUITY,
            self.camera,
            kRobotToCam,
        )
        self.result = 0
        self.hasTargets = False
        self.target = 0
        self.ambiguity = 1
        self.photonTable = NetworkTableInstance.getDefault().getTable("photon")
        self.robotX = 0
        self.robotY = 0
        self.robotAngle = False
        self.TFID = 0

        # self.photonTable.putNumber(self.cameraNameReal + "Rot :", 0)
        # self.photonTable.putNumber(self.cameraNameReal + "X :", 0)
        # self.photonTable.putNumber(self.cameraNameReal + "Y :", 0)
        # self.photonTable.putNumber(self.cameraNameReal + "ambiguity :", 1)

    def update(self):
        # startCameraUpdate = wpilib.getTime()
        self.result = self.camera.getLatestResult()
        # self.photonTable.putNumber(
        #    "result update time", wpilib.getTime() - startCameraUpdate
        # )
        # startCameraUpdate = wpilib.getTime()
        self.hasTargets = self.result.hasTargets()
        # self.photonTable.putNumber(
        #     "hasTargests update time", wpilib.getTime() - startCameraUpdate
        # )
        # startCameraUpdate = wpilib.getTime()
        self.target = self.result.getTargets()
        # self.photonTable.putNumber("getTargets", wpilib.getTime() - startCameraUpdate)
        if self.hasTargets:
            self.fiducialId = self.target[0].getFiducialId()
            self.ambiguity = self.target[0].getPoseAmbiguity()
            self.photonTable.putNumber("ambiguity", self.ambiguity)
            if self.ambiguity < 0.04:
                self.camEstPose = self.camPoseEst.update()
                self.TFID = self.fiducialId
                # self.photonTable.putNumber("FiducialID", self.fiducialId)
                if self.camEstPose != None:
                    self.robotX = self.camEstPose.estimatedPose.X()
                    # self.photonTable.putNumber(self.cameraNameReal + "X :", self.robotX)
                    self.robotY = self.camEstPose.estimatedPose.Y()
                    # self.photonTable.putNumber(self.cameraNameReal + "Y :", self.robotY)
                    self.robotAngle = self.camEstPose.estimatedPose.rotation().Z()
                    # self.photonTable.putNumber(
                    #     self.cameraNameReal + "Rot :", self.robotAngle
                    # )
            else:
                self.TFID = -1
        else:
            self.ambiguity = 1
            self.fiducialId = -1
        # self.photonTable.putNumber("Setpoint Fid Id", self.TFID)
        # self.photonTable.putNumber(self.cameraNameReal + "ambiguity :", self.ambiguity)

    # def savePos(self):
    #     with open("pyTest.txt", "a") as f:
    #         f.write(self.cameraNameReal + " X = " + f"{self.robotX}" "\n")
    #         f.write(self.cameraNameReal + " Y = " + f"{self.robotY}" "\n")
    #         f.write(self.cameraNameReal + " Angle = " + f"{self.robotAngle}" "\n")
    #     # Test.close()
