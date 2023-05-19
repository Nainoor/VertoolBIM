import hppfcl.hppfcl
import typing

__all__ = [
    "AABB",
    "AngleAxis",
    "BVHBuildState",
    "BVHModelBase",
    "BVHModelOBB",
    "BVHModelOBBRSS",
    "BVHModelType",
    "BVH_BUILD_STATE_BEGUN",
    "BVH_BUILD_STATE_EMPTY",
    "BVH_BUILD_STATE_PROCESSED",
    "BVH_BUILD_STATE_REPLACE_BEGUN",
    "BVH_BUILD_STATE_UPDATED",
    "BVH_BUILD_STATE_UPDATE_BEGUN",
    "BVH_MODEL_POINTCLOUD",
    "BVH_MODEL_TRIANGLES",
    "BVH_MODEL_UNKNOWN",
    "BV_AABB",
    "BV_KDOP16",
    "BV_KDOP18",
    "BV_KDOP24",
    "BV_OBB",
    "BV_OBBRSS",
    "BV_RSS",
    "BV_UNKNOWN",
    "BV_kIOS",
    "Box",
    "CONTACT",
    "CPUTimes",
    "CachedMeshLoader",
    "Capsule",
    "CollisionGeometry",
    "CollisionObject",
    "CollisionRequest",
    "CollisionRequestFlag",
    "CollisionResult",
    "ComputeCollision",
    "ComputeDistance",
    "Cone",
    "Contact",
    "Convex",
    "ConvexBase",
    "Cylinder",
    "DISTANCE_LOWER_BOUND",
    "DistanceRequest",
    "DistanceResult",
    "Exception",
    "Failed",
    "GEOM_BOX",
    "GEOM_CAPSULE",
    "GEOM_CONE",
    "GEOM_CONVEX",
    "GEOM_CYLINDER",
    "GEOM_HALFSPACE",
    "GEOM_OCTREE",
    "GEOM_PLANE",
    "GEOM_SPHERE",
    "GEOM_TRIANGLE",
    "GJK",
    "GJKStatus",
    "HF_AABB",
    "HF_OBBRSS",
    "HPP_FCL_MAJOR_VERSION",
    "HPP_FCL_MINOR_VERSION",
    "HPP_FCL_PATCH_VERSION",
    "Halfspace",
    "HeightFieldAABB",
    "HeightFieldOBBRSS",
    "Inside",
    "MeshLoader",
    "MinkowskiDiff",
    "NODE_TYPE",
    "NO_REQUEST",
    "OBJECT_TYPE",
    "OT_BVH",
    "OT_GEOM",
    "OT_HFIELD",
    "OT_OCTREE",
    "OT_UNKNOWN",
    "OcTree",
    "Plane",
    "Quaternion",
    "QueryRequest",
    "QueryResult",
    "ShapeBase",
    "Sphere",
    "StdVec_CollisionRequest",
    "StdVec_CollisionResult",
    "StdVec_Contact",
    "StdVec_DistanceRequest",
    "StdVec_DistanceResult",
    "StdVec_Triangle",
    "StdVec_Vec3f",
    "Transform3f",
    "Triangle",
    "TriangleP",
    "Valid",
    "checkVersionAtLeast",
    "checkVersionAtMost",
    "collide",
    "computeMemoryFootprint",
    "distance",
    "getNumpyType",
    "makeOctree",
    "rotate",
    "seed",
    "setNumpyType",
    "sharedMemory",
    "switchToNumpyArray",
    "switchToNumpyMatrix",
    "translate"
]

class AABB():
    pass

class AngleAxis():
    pass

class CollisionGeometry():
    pass

class BVHModelBase():
    pass

class BVHModelOBBRSS():
    pass

class ShapeBase():
    pass

class CPUTimes():
    pass

class MeshLoader():
    pass

class Capsule():
    pass

class BVHModelOBB():
    pass

class CollisionObject():
    pass

class QueryRequest():
    pass

class QueryResult():
    pass

class ComputeCollision():
    pass

class ComputeDistance():
    pass

class Cone():
    pass

class Contact():
    pass

class ConvexBase():
    pass

class Convex():
    pass

class Cylinder():
    pass

class DistanceRequest():
    pass

class DistanceResult():
    pass

class Exception():
    pass

class GJK():
    pass

class Halfspace():
    pass

class HeightFieldAABB():
    pass

class HeightFieldOBBRSS():
    pass

class CachedMeshLoader():
    pass

class MinkowskiDiff():
    pass

class OcTree():
    pass

class Plane():
    pass

class Quaternion():
    pass

class CollisionRequest():
    pass

class CollisionResult():
    pass

class Box():
    pass

class Sphere():
    pass

class StdVec_CollisionRequest():
    pass

class StdVec_CollisionResult():
    pass

class StdVec_Contact():
    pass

class StdVec_DistanceRequest():
    pass

class StdVec_DistanceResult():
    pass

class StdVec_Triangle():
    pass

class StdVec_Vec3f():
    pass

class Transform3f():
    pass

class Triangle():
    pass

class TriangleP():
    pass

def checkVersionAtLeast(major: int, minor: int, patch: int) -> bool:
    """
    checkVersionAtLeast( (int)major, (int)minor, (int)patch) -> bool :
        Checks if the current version of hpp-fcl is at least the version provided by the input arguments.

        C++ signature :
            bool checkVersionAtLeast(unsigned int,unsigned int,unsigned int)
    """

def checkVersionAtMost(major: int, minor: int, patch: int) -> bool:
    """
    checkVersionAtMost( (int)major, (int)minor, (int)patch) -> bool :
        Checks if the current version of hpp-fcl is at most the version provided by the input arguments.

        C++ signature :
            bool checkVersionAtMost(unsigned int,unsigned int,unsigned int)
    """

@typing.overload
def collide(arg1: CollisionGeometry, arg2: Transform3f, arg3: CollisionGeometry, arg4: Transform3f, arg5: CollisionRequest, arg6: CollisionResult) -> int:
    """
    collide( (CollisionObject)arg1, (CollisionObject)arg2, (CollisionRequest)arg3, (CollisionResult)arg4) -> int :

        C++ signature :
            unsigned __int64 collide(class hpp::fcl::CollisionObject const * __ptr64,class hpp::fcl::CollisionObject const * __ptr64,struct hpp::fcl::CollisionRequest,struct hpp::fcl::CollisionResult {lvalue})

        C++ signature :
            unsigned __int64 collide(class hpp::fcl::CollisionGeometry const * __ptr64,class hpp::fcl::Transform3f,class hpp::fcl::CollisionGeometry const * __ptr64,class hpp::fcl::Transform3f,struct hpp::fcl::CollisionRequest {lvalue},struct hpp::fcl::CollisionResult {lvalue})
    """
@typing.overload
def collide(arg1: CollisionObject, arg2: CollisionObject, arg3: CollisionRequest, arg4: CollisionResult) -> int:
    pass

@typing.overload
def computeMemoryFootprint(arg1: BVHModelOBB) -> int:
    """
    computeMemoryFootprint( (Sphere)arg1) -> int :

        C++ signature :
            unsigned __int64 computeMemoryFootprint(class hpp::fcl::Sphere)

        C++ signature :
            unsigned __int64 computeMemoryFootprint(class hpp::fcl::Cone)

        C++ signature :
            unsigned __int64 computeMemoryFootprint(class hpp::fcl::Capsule)

        C++ signature :
            unsigned __int64 computeMemoryFootprint(class hpp::fcl::Cylinder)

        C++ signature :
            unsigned __int64 computeMemoryFootprint(class hpp::fcl::Box)

        C++ signature :
            unsigned __int64 computeMemoryFootprint(class hpp::fcl::Plane)

        C++ signature :
            unsigned __int64 computeMemoryFootprint(class hpp::fcl::Halfspace)

        C++ signature :
            unsigned __int64 computeMemoryFootprint(class hpp::fcl::TriangleP)

        C++ signature :
            unsigned __int64 computeMemoryFootprint(class hpp::fcl::BVHModel<struct hpp::fcl::OBB>)

        C++ signature :
            unsigned __int64 computeMemoryFootprint(class hpp::fcl::BVHModel<struct hpp::fcl::RSS>)

        C++ signature :
            unsigned __int64 computeMemoryFootprint(class hpp::fcl::BVHModel<struct hpp::fcl::OBBRSS>)
    """
@typing.overload
def computeMemoryFootprint(arg1: BVHModelOBBRSS) -> int:
    pass
@typing.overload
def computeMemoryFootprint(arg1: Box) -> int:
    pass
@typing.overload
def computeMemoryFootprint(arg1: Capsule) -> int:
    pass
@typing.overload
def computeMemoryFootprint(arg1: Cone) -> int:
    pass
@typing.overload
def computeMemoryFootprint(arg1: Cylinder) -> int:
    pass
@typing.overload
def computeMemoryFootprint(arg1: Halfspace) -> int:
    pass
@typing.overload
def computeMemoryFootprint(arg1: Plane) -> int:
    pass
@typing.overload
def computeMemoryFootprint(arg1: Sphere) -> int:
    pass
@typing.overload
def computeMemoryFootprint(arg1: TriangleP) -> int:
    pass
@typing.overload
def computeMemoryFootprint(arg1: object) -> int:
    pass

@typing.overload
def distance(arg1: CollisionGeometry, arg2: Transform3f, arg3: CollisionGeometry, arg4: Transform3f, arg5: DistanceRequest, arg6: DistanceResult) -> float:
    """
    distance( (CollisionObject)arg1, (CollisionObject)arg2, (DistanceRequest)arg3, (DistanceResult)arg4) -> float :

        C++ signature :
            double distance(class hpp::fcl::CollisionObject const * __ptr64,class hpp::fcl::CollisionObject const * __ptr64,struct hpp::fcl::DistanceRequest,struct hpp::fcl::DistanceResult {lvalue})

        C++ signature :
            double distance(class hpp::fcl::CollisionGeometry const * __ptr64,class hpp::fcl::Transform3f,class hpp::fcl::CollisionGeometry const * __ptr64,class hpp::fcl::Transform3f,struct hpp::fcl::DistanceRequest {lvalue},struct hpp::fcl::DistanceResult {lvalue})
    """
@typing.overload
def distance(arg1: CollisionObject, arg2: CollisionObject, arg3: DistanceRequest, arg4: DistanceResult) -> float:
    pass

def getNumpyType() -> object:
    """
    getNumpyType() -> object :
        Get the Numpy type returned by the converters from an Eigen object.

        C++ signature :
            class boost::python::api::object getNumpyType()
    """

def makeOctree(arg1: object, arg2: float) -> OcTree:
    """
    makeOctree( (object)arg1, (float)arg2) -> OcTree :

        C++ signature :
            class boost::shared_ptr<class hpp::fcl::OcTree> makeOctree(class Eigen::Matrix<double,-1,3,0,-1,3>,double)
    """

def rotate(aabb: AABB, R: object) -> AABB:
    """
    rotate( (AABB)aabb, (object)R) -> AABB :
        Rotate the AABB object by R

        C++ signature :
            class hpp::fcl::AABB rotate(class hpp::fcl::AABB,class Eigen::Matrix<double,3,3,0,3,3>)
    """

def seed(seed_value: int) -> None:
    """
    seed( (int)seed_value) -> None :
        Initialize the pseudo-random number generator with the argument seed_value.

        C++ signature :
            void seed(unsigned int)
    """

def setNumpyType(numpy_type: object) -> None:
    """
    setNumpyType( (object)numpy_type) -> None :
        Change the Numpy type returned by the converters from an Eigen object.

        C++ signature :
            void setNumpyType(class boost::python::api::object {lvalue})
    """

@typing.overload
def sharedMemory() -> bool:
    """
    sharedMemory( (bool)value) -> None :
        Share the memory when converting from Eigen to Numpy.

        C++ signature :
            void sharedMemory(bool)

        C++ signature :
            bool sharedMemory()
    """
@typing.overload
def sharedMemory(value: bool) -> None:
    pass

def switchToNumpyArray() -> None:
    """
    switchToNumpyArray() -> None :
        Set the conversion from Eigen::Matrix to numpy.ndarray.

        C++ signature :
            void switchToNumpyArray()
    """

def switchToNumpyMatrix() -> None:
    """
    switchToNumpyMatrix() -> None :
        Set the conversion from Eigen::Matrix to numpy.matrix.

        C++ signature :
            void switchToNumpyMatrix()
    """

def translate(aabb: AABB, t: object) -> AABB:
    """
    translate( (AABB)aabb, (object)t) -> AABB :
        Translate the center of AABB by t

        C++ signature :
            class hpp::fcl::AABB translate(class hpp::fcl::AABB,class Eigen::Matrix<double,3,1,0,3,1>)
    """

BVH_BUILD_STATE_BEGUN = hppfcl.hppfcl.BVHBuildState.BVH_BUILD_STATE_BEGUN
BVH_BUILD_STATE_EMPTY = hppfcl.hppfcl.BVHBuildState.BVH_BUILD_STATE_EMPTY
BVH_BUILD_STATE_PROCESSED = hppfcl.hppfcl.BVHBuildState.BVH_BUILD_STATE_PROCESSED
BVH_BUILD_STATE_REPLACE_BEGUN = hppfcl.hppfcl.BVHBuildState.BVH_BUILD_STATE_REPLACE_BEGUN
BVH_BUILD_STATE_UPDATED = hppfcl.hppfcl.BVHBuildState.BVH_BUILD_STATE_UPDATED
BVH_BUILD_STATE_UPDATE_BEGUN = hppfcl.hppfcl.BVHBuildState.BVH_BUILD_STATE_UPDATE_BEGUN
BVH_MODEL_POINTCLOUD = hppfcl.hppfcl.BVHModelType.BVH_MODEL_POINTCLOUD
BVH_MODEL_TRIANGLES = hppfcl.hppfcl.BVHModelType.BVH_MODEL_TRIANGLES
BVH_MODEL_UNKNOWN = hppfcl.hppfcl.BVHModelType.BVH_MODEL_UNKNOWN
BV_AABB = hppfcl.hppfcl.NODE_TYPE.BV_AABB
BV_KDOP16 = hppfcl.hppfcl.NODE_TYPE.BV_KDOP16
BV_KDOP18 = hppfcl.hppfcl.NODE_TYPE.BV_KDOP18
BV_KDOP24 = hppfcl.hppfcl.NODE_TYPE.BV_KDOP24
BV_OBB = hppfcl.hppfcl.NODE_TYPE.BV_OBB
BV_OBBRSS = hppfcl.hppfcl.NODE_TYPE.BV_OBBRSS
BV_RSS = hppfcl.hppfcl.NODE_TYPE.BV_RSS
BV_UNKNOWN = hppfcl.hppfcl.NODE_TYPE.BV_UNKNOWN
BV_kIOS = hppfcl.hppfcl.NODE_TYPE.BV_kIOS
CONTACT = hppfcl.hppfcl.CollisionRequestFlag.CONTACT
DISTANCE_LOWER_BOUND = hppfcl.hppfcl.CollisionRequestFlag.DISTANCE_LOWER_BOUND
Failed = hppfcl.hppfcl.GJKStatus.Failed
GEOM_BOX = hppfcl.hppfcl.NODE_TYPE.GEOM_BOX
GEOM_CAPSULE = hppfcl.hppfcl.NODE_TYPE.GEOM_CAPSULE
GEOM_CONE = hppfcl.hppfcl.NODE_TYPE.GEOM_CONE
GEOM_CONVEX = hppfcl.hppfcl.NODE_TYPE.GEOM_CONVEX
GEOM_CYLINDER = hppfcl.hppfcl.NODE_TYPE.GEOM_CYLINDER
GEOM_HALFSPACE = hppfcl.hppfcl.NODE_TYPE.GEOM_HALFSPACE
GEOM_OCTREE = hppfcl.hppfcl.NODE_TYPE.GEOM_OCTREE
GEOM_PLANE = hppfcl.hppfcl.NODE_TYPE.GEOM_PLANE
GEOM_SPHERE = hppfcl.hppfcl.NODE_TYPE.GEOM_SPHERE
GEOM_TRIANGLE = hppfcl.hppfcl.NODE_TYPE.GEOM_TRIANGLE
HF_AABB = hppfcl.hppfcl.NODE_TYPE.HF_AABB
HF_OBBRSS = hppfcl.hppfcl.NODE_TYPE.HF_OBBRSS
HPP_FCL_MAJOR_VERSION = 1
HPP_FCL_MINOR_VERSION = 8
HPP_FCL_PATCH_VERSION = 0
Inside = hppfcl.hppfcl.GJKStatus.Inside
NO_REQUEST = hppfcl.hppfcl.CollisionRequestFlag.NO_REQUEST
OT_BVH = hppfcl.hppfcl.OBJECT_TYPE.OT_BVH
OT_GEOM = hppfcl.hppfcl.OBJECT_TYPE.OT_GEOM
OT_HFIELD = hppfcl.hppfcl.OBJECT_TYPE.OT_HFIELD
OT_OCTREE = hppfcl.hppfcl.OBJECT_TYPE.OT_OCTREE
OT_UNKNOWN = hppfcl.hppfcl.OBJECT_TYPE.OT_UNKNOWN
Valid = hppfcl.hppfcl.GJKStatus.Valid
__raw_version__ = '1.8.0'
__version__ = '1.8.0'

