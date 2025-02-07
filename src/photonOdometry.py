import photonlibpy

# from photonlibpy import photonCamera, photonPoseEstimator, poseStrategy
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy

from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from ntcore import NetworkTableInstance
import wpimath.geometry
import numpy

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
        self.camera = PhotonCamera(cameraName)
        kRobotToCam = wpimath.geometry.Transform3d(
    wpimath.geometry.Translation3d(intCamX, intCamY, intCamZ),
    wpimath.geometry.Rotation3d.fromDegrees(0.0, camPitch, 0.0),
)
        self.camPoseEst = PhotonPoseEstimator(
        AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape),
            PoseStrategy.LOWEST_AMBIGUITY,
            self.camera,
            kRobotToCam,
        )
        self.result = 0
        self.hasTargets = False

        self.target = 0
        self.ambiguity = 1
        # self.pitch = 0
        # self.yaw = 0
        # self.area = 0
        # self.skew = 0
        #self.pose = 0
        self.transform3d = 0
        self.pX = 0
        self.pY = 0
        self.pH = 0
        self.camX = 0
        self.camY = 0
        self.camAngle = camPitch
        self.crX = intCamX
        self.crY = intCamY
        self.cHyp = 0
        self.angle = False
        self.ca = False
        self.photonTable = NetworkTableInstance.getDefault().getTable("photon")
        self.robotX = 0
        self.robotY = 0
        self.robotAngle = False

    def update(self):

        # self.photonTable.putNumber("estPose", camEstPose.estimatedPose.x)
        self.result = self.camera.getLatestResult()
        self.hasTargets = self.result.hasTargets()
        self.target = self.result.getTargets()
        if self.hasTargets:
            self.fiducialId = self.target[0].getFiducialId()
            # print("this is an idididiidididiididid: ", self.fiducialId)

            # self.pitch = self.target[0].getPitch()
            # self.yaw = self.target[0].getYaw()
            #self.area = self.target[0].getArea()
            #self.skew = self.target[0].getSkew()
            #self.pose = self.target[0].getBestCameraToTarget()
            # self.corners = self.target.getDetectedCorners()
            self.ambiguity = self.target[0].getPoseAmbiguity()
            self.transform3d = self.target[0].getBestCameraToTarget()
            self.cHyp = self.transform3d.X()
            #print ("XXXXX", self.pX)
            self.pY = self.transform3d.Y()
            # self.photonOdometry =
            self.a = self.transform3d.rotation()
            self.angle = self.a.Z()
            #self.angle = self.angle *(180/numpy.pi)
            #print("angleee", self.angle)
        else:
            self.ambiguity = 1
            self.pitch = 0
            self.yaw = 0
            self.area = 0
            self.skew = 0
            self.pose = 0
            self.transform3d = 0
            self.fiducialId = 0
            self.pX = False
            self.pY = False
        if self.ambiguity < 0.16:
            self.camEstPose = self.camPoseEst.update()
            #print("est pose", self.camEstPose.estimatedPose.X)
            self.robotX = self.camEstPose.estimatedPose.x
            self.robotY = self.camEstPose.estimatedPose.y
            self.robotAngle = self.camEstPose.estimatedPose.rotation
            #self.robotAngle = self.robotAngle.
            #print("robotX", self.robotX)
   

    def odometryUpdate(self):

        if not (self.ambiguity > 0.15 and self.fiducialId < 1):
           # self.lsa = 
            self.pAngle = self.angle + numpy.pi
            self.pX = self.cHyp * numpy.sin(self.pAngle)
            if self.pX < 0:
                self.pX * -1
            self.pY = numpy.sqrt((self.cHyp * self.cHyp) - (self.pX * self.pX))
            self.camX = ((aprilTagX[self.fiducialId])/39.37) - self.pX
            self.camY = ((aprilTagY[self.fiducialId])/39.37) - self.pY
            # elif self.angle > -1:
            #     self.pAngle = 180 - self.angle
            #     self.pX = self.cHyp * numpy.sin(self.pAngle)
            #     self.pY = numpy.sqrt((self.cHyp * self.cHyp) - (self.pX * self.pX))
            #     self.camX = ((aprilTagX[self.fiducialId])/39.37) - self.pX
            #     self.camY = ((aprilTagY[self.fiducialId])/39.37) - self.pY
            print("cx", self.pX)
            print("cy", self.pY)
            self.ca = aprilTagDegrees[self.fiducialId] + (self.angle + 180)
            self.ca = self.ca - 180
            if self.ca < -360:
                self.ca + 720
            if self.ca < 0:
                self.ca + 360
            self.photonTable.putNumber("x", self.camX)
            self.photonTable.putNumber("y", self.camY)
            self.photonTable.putNumber("Ambiguity", self.ambiguity)
            self.photonTable.putNumber("angle", self.ca)
            print("x = ", self.camX)
            print("y = ", self.camY)
            print("angle", self.ca)
           
            print("est X =", self.camEstPose.estimatedPose.X)
            
            self.robotX = self.camX - self.crX
            self.robotY = self.camY - self.crY
            self.robotAngle = self.ca + self.camAngle


            