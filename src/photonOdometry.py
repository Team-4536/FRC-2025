import photonlibpy
from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from ntcore import NetworkTableInstance
import wpimath.geometry

#photonTable = NetworkTableInstance.getDefault()
kRobotToCam = wpimath.geometry.Transform3d(
    wpimath.geometry.Translation3d(0.5, 0.0, 0.5),
    wpimath.geometry.Rotation3d.fromDegrees(0.0, -30.0, 0.0),
)
class phtonVision:
    photonTable = NetworkTableInstance.getDefault()
    def __init__(self, cameraName):
        self.camera = photonCamera.PhotonCamera("cameraName")
        
        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
            PoseStrategy.LOWEST_AMBIGUITY,
            self.cam,
            kRobotToCam,
        )
        # self.result = 0
        # self.hasTargets = False
        # self.target = 0
        # self.ambiguity = 1
        # self.pitch = 0
        # self.yaw = 0
        # self.area = 0
        # self.skew = 0
        # self.pose = 0
        # self.transform3d = 0
        # self.pX = 0
        # self.pY = 0
        # self.pH = 0
        self.photonTable = NetworkTableInstance.getDefault().getTable("photon")



        
    def update(self):
        self.result = self.camera.getLatestResult()
        self.hasTargets = self.result.hasTargets()
        self.target = self.result.getTargets()

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
        self.photonTable.putNumber("Pitch", self.pitch)
        self.photonTable.putNumber("Yaw", self.yaw)
        self.photonTable.putNumber("Area", self.area)
        self.photonTable.putNumber("Skew", self.skew)
        self.photonTable.putNumber("Pose", self.pose)
        self.photonTable.putNumber("Ambiguity", self.ambiguity)
        self.photonTable.putBoolean("Has Targets", self.hasTargets)

    def odometryUpdate(self):
        
        if not(self.ambiguity > 0.15):
            self.pH = (self.pX ** 2) + (self.pY ** 2)
            self.pH = self.pH ** (1/2)
            