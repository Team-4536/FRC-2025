import photonlibpy
from photonlibpy import photonCamera
from ntcore import NetworkTableInstance
camera = photonCamera.PhotonCamera("Camera1")
photonTable = NetworkTableInstance.getDefault()
#class phtonVision:
    
    
result = camera.getLatestResult()
hasTargets = result.hasTargets()
target = result.getTargets()

pitch = target[0].getPitch()
yaw = target[1].getYaw()
area = target[2].getArea()
skew = target[3].getSkew()
pose = target[4].getBestCameraToTarget()
#corners = target.getDetectedCorners()
ambiguity = target[5].getPoseAmbiguity()
transform3d = target[6].getBestCameraToTarget()
#photonOdometry = 
photonTable.putNumber("pitch", 0)