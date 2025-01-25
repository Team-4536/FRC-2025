import photonlibpy
#from photonlibpy import photonCamera, photonPoseEstimator, poseStrategy
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy

from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from ntcore import NetworkTableInstance
import wpimath.geometry

#photonTable = NetworkTableInstance.getDefault()
# kRobotToCam = wpimath.geometry.Transform3d(
#     wpimath.geometry.Translation3d(0.5, 0.0, 0.5),
#     wpimath.geometry.Rotation3d.fromDegrees(0.0, -30.0, 0.0),
# )
aprilTagX = [0, 657.37, 657.37, 455.15, 365.20, 365.20, 530.49, 546.87, 530.49, 497.77, 481.39, 497.77, 33.51, 33.51, 325.68, 325.68, 235.73, 160.39, 144.00, 160.39, 193.10, 209.49, 193.10]
aprilTagY = [0, 25.80, 291.20, 317.15, 241.64, 75.39, 130.17, 158.50, 186.83, 186.83, 158.50, 130.17, 25.80, 291.20, 241.64, 75.39, -0.15, 130.17, 158.50, 186.83, 186.83, 158.50, 130.17]
class photonVision:
    photonTable = NetworkTableInstance.getDefault()
    def __init__(self, cameraName):
        self.camera = PhotonCamera(cameraName)
        
        # self.camPoseEst = PhotonPoseEstimator(
        #AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape),
        #     PoseStrategy.LOWEST_AMBIGUITY,
        #     self.camera,
        #     kRobotToCam,
        # )
        self.result = 0
        self.hasTargets = False
        
        self.target = 0
        self.ambiguity = 1
        self.pitch = 0
        self.yaw = 0
        self.area = 0
        self.skew = 0
        self.pose = 0
        self.transform3d = 0
        self.pX = 0
        self.pY = 0
        self.pH = 0
        self.fX = 0
        self.fY = 0
        self.photonTable = NetworkTableInstance.getDefault().getTable("photon")



        
    def update(self):

        #camEstPose = self.camPoseEst.update()
        #self.photonTable.putNumber("estPose", camEstPose.estimatedPose.x)
        self.result = self.camera.getLatestResult()
        self.hasTargets = self.result.hasTargets()
        self.target = self.result.getTargets()
        if self.hasTargets:
            self.fiducialId = self.target[0].getFiducialId()
            #print("this is an idididiidididiididid: ", self.fiducialId)

            self.pitch = self.target[0].getPitch()
            self.yaw = self.target[0].getYaw()
            self.area = self.target[0].getArea()
            self.skew = self.target[0].getSkew()
            self.pose = self.target[0].getBestCameraToTarget()
            #self.corners = self.target.getDetectedCorners()
            self.ambiguity = self.target[0].getPoseAmbiguity()
            self.transform3d = self.target[0].getBestCameraToTarget()
            self.pX = self.transform3d.X()
            self.pY = self.transform3d.Y()
            #self.photonOdometry = 
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
        self.photonTable.putNumber("Pitch", self.pitch)
        self.photonTable.putNumber("Yaw", self.yaw)
        self.photonTable.putNumber("Area", self.area)
        self.photonTable.putNumber("Skew", self.skew)
     
        self.photonTable.putNumber("Ambiguity", self.ambiguity)
        self.photonTable.putNumber("FiducialId", self.fiducialId)
        self.photonTable.putBoolean("Has Targets", self.hasTargets)


    # def odometryUpdate(self):
        
    #     if not(self.ambiguity > 0.15 and self.fiducialId < 1):
            # self.pH = (self.pX ** 2) + (self.pY ** 2)
            # self.pH = self.pH ** (1/2)
            #print("fiducial id: ", self.fiducialId)
            # self.fX = (aprilTagX[self.fiducialId]) - self.pX
            # self.fY = (aprilTagY[self.fiducialId]) - self.pY
            # self.photonTable.putNumber("x", self.fX)
            #self.photonTable.putNumber("y", self.fY)
            
