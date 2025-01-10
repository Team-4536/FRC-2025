import photonlibpy
from photonlibpy import photonCamera
from ntcore import NetworkTableInstance

#photonTable = NetworkTableInstance.getDefault()
class phtonVision:
    photonTable = NetworkTableInstance.getDefault()
    def __init__(self, cameraName):
        self.camera = photonCamera.PhotonCamera("cameraName")
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
        self.photonTable = NetworkTableInstance.getDefault().getTable("photon")


        
    def update(self):
        self.result = self.camera.getLatestResult()
        self.hasTargets = self.result.hasTargets()
        self.target = self.result.getTargets()

        self.pitch = self.target[0].getPitch()
        self.yaw = self.target[1].getYaw()
        self.area = self.target[2].getArea()
        self.skew = self.target[3].getSkew()
        self.pose = self.target[4].getBestCameraToTarget()
        #self.corners = self.target.getDetectedCorners()
        self.ambiguity = self.target[5].getPoseAmbiguity()
        self.transform3d = self.target[6].getBestCameraToTarget()
        #self.photonOdometry = 
        self.photonTable.putNumber("Pitch", self.pitch)
        self.photonTable.putNumber("Yaw", self.yaw)
        self.photonTable.putNumber("Area", self.area)
        self.photonTable.putNumber("Skew", self.skew)
        self.photonTable.putNumber("Pose", self.pose)
        self.photonTable.putNumber("Ambiguity", self.ambiguity)
        self.photonTable.putBoolean("Has Targets", self.hasTargets)