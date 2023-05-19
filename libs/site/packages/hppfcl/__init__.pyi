import hppfcl
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
    "hppfcl",
    "makeOctree",
    "rotate",
    "seed",
    "setNumpyType",
    "sharedMemory",
    "switchToNumpyArray",
    "switchToNumpyMatrix",
    "translate"
]

class AABB(Boost.Python.instance):
    """
    A class describing the AABB collision structure, which is a box in 3D space determined by two diagonal points
    """
    @staticmethod
    def __add__(arg1: AABB, arg2: AABB) -> object: 
        """
        __add__( (AABB)arg1, (AABB)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __add__(class hpp::fcl::AABB {lvalue},class hpp::fcl::AABB)
        """
    @staticmethod
    @typing.overload
    def __iadd__(arg1: object, arg2: AABB) -> object: 
        """
        __iadd__( (object)arg1, (AABB)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __iadd__(struct boost::python::back_reference<class hpp::fcl::AABB & __ptr64>,class hpp::fcl::AABB)

            C++ signature :
                struct _object * __ptr64 __iadd__(struct boost::python::back_reference<class hpp::fcl::AABB & __ptr64>,class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @staticmethod
    @typing.overload
    def __iadd__(arg1: object, arg2: object) -> object: ...
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None :
            Default constructor

            C++ signature :
                void __init__(struct _object * __ptr64)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Matrix<double,3,1,0,3,1>)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<double,3,1,0,3,1>)

            C++ signature :
                void __init__(struct _object * __ptr64,class hpp::fcl::AABB,class Eigen::Matrix<double,3,1,0,3,1>)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @typing.overload
    def __init__(self, a: object, b: object) -> None: ...
    @typing.overload
    def __init__(self, a: object, b: object, c: object) -> None: ...
    @typing.overload
    def __init__(self, core: AABB, delta: object) -> None: ...
    @typing.overload
    def __init__(self, v: object) -> None: ...
    def center(self) -> object: 
        """
        center( (AABB)self) -> object :
            Center of the AABB.

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> center(class hpp::fcl::AABB {lvalue})
        """
    @typing.overload
    def contain(self, other: AABB) -> bool: 
        """
        contain( (AABB)self, (object)p) -> bool :
            Check whether the AABB contains a point p.

            C++ signature :
                bool contain(class hpp::fcl::AABB {lvalue},class Eigen::Matrix<double,3,1,0,3,1>)

            C++ signature :
                bool contain(class hpp::fcl::AABB {lvalue},class hpp::fcl::AABB)
        """
    @typing.overload
    def contain(self, p: object) -> bool: ...
    def depth(self) -> float: 
        """
        depth( (AABB)self) -> float :
            Depth of the AABB.

            C++ signature :
                double depth(class hpp::fcl::AABB {lvalue})
        """
    def distance(self, other: AABB) -> float: 
        """
        distance( (AABB)self, (AABB)other) -> float :
            Distance between two AABBs.

            C++ signature :
                double distance(class hpp::fcl::AABB {lvalue},class hpp::fcl::AABB)
        """
    @typing.overload
    def expand(self, core: AABB, ratio: float) -> AABB: 
        """
        expand( (AABB)self, (AABB)core, (float)ratio) -> AABB :
            Expand the AABB by increase the thickness of the plate by a ratio.

            C++ signature :
                class hpp::fcl::AABB {lvalue} expand(class hpp::fcl::AABB {lvalue},class hpp::fcl::AABB,double)

            C++ signature :
                class hpp::fcl::AABB {lvalue} expand(class hpp::fcl::AABB {lvalue},class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @typing.overload
    def expand(self, delta: object) -> AABB: ...
    def height(self) -> float: 
        """
        height( (AABB)self) -> float :
            Height of the AABB.

            C++ signature :
                double height(class hpp::fcl::AABB {lvalue})
        """
    @typing.overload
    def overlap(self, other: AABB) -> bool: 
        """
        overlap( (AABB)self, (AABB)other) -> bool :
            Check whether two AABB are overlap.

            C++ signature :
                bool overlap(class hpp::fcl::AABB {lvalue},class hpp::fcl::AABB)

            C++ signature :
                bool overlap(class hpp::fcl::AABB {lvalue},class hpp::fcl::AABB,class hpp::fcl::AABB {lvalue})
        """
    @typing.overload
    def overlap(self, other: AABB, overlapping_part: AABB) -> bool: ...
    def size(self) -> float: 
        """
        size( (AABB)self) -> float :
            Size of the AABB.

            C++ signature :
                double size(class hpp::fcl::AABB {lvalue})
        """
    def volume(self) -> float: 
        """
        volume( (AABB)self) -> float :
            Volume of the AABB.

            C++ signature :
                double volume(class hpp::fcl::AABB {lvalue})
        """
    def width(self) -> float: 
        """
        width( (AABB)self) -> float :
            Width of the AABB.

            C++ signature :
                double width(class hpp::fcl::AABB {lvalue})
        """
    pass

class AngleAxis(Boost.Python.instance):
    """
    AngleAxis representation of a rotation.
    """
    @staticmethod
    def __eq__(arg1: AngleAxis, arg2: AngleAxis) -> bool: 
        """
        __eq__( (AngleAxis)arg1, (AngleAxis)arg2) -> bool :

            C++ signature :
                bool __eq__(class Eigen::AngleAxis<double>,class Eigen::AngleAxis<double>)
        """
    @typing.overload
    def __init__(self) -> None: 
        """
        __init__( (object)self) -> None :
            Default constructor

            C++ signature :
                void __init__(struct _object * __ptr64)

            C++ signature :
                void __init__(struct _object * __ptr64,double,class Eigen::Matrix<double,3,1,0,3,1>)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Matrix<double,3,3,0,3,3>)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Quaternion<double,0>)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::AngleAxis<double>)
        """
    @typing.overload
    def __init__(self, angle: float, axis: object) -> None: ...
    @typing.overload
    def __init__(self, copy: AngleAxis) -> None: ...
    @typing.overload
    def __init__(self, quaternion: Quaternion) -> None: ...
    @typing.overload
    def __init__(self, rotation_matrix: object) -> None: ...
    @staticmethod
    @typing.overload
    def __mul__(arg1: AngleAxis, arg2: AngleAxis) -> object: 
        """
        __mul__( (AngleAxis)arg1, (object)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __mul__(class Eigen::AngleAxis<double> {lvalue},class Eigen::Matrix<double,3,1,0,3,1>)

            C++ signature :
                struct _object * __ptr64 __mul__(class Eigen::AngleAxis<double> {lvalue},class Eigen::Quaternion<double,0>)

            C++ signature :
                struct _object * __ptr64 __mul__(class Eigen::AngleAxis<double> {lvalue},class Eigen::AngleAxis<double>)
        """
    @staticmethod
    @typing.overload
    def __mul__(arg1: AngleAxis, arg2: Quaternion) -> object: ...
    @staticmethod
    @typing.overload
    def __mul__(arg1: AngleAxis, arg2: object) -> object: ...
    @staticmethod
    def __ne__(arg1: AngleAxis, arg2: AngleAxis) -> bool: 
        """
        __ne__( (AngleAxis)arg1, (AngleAxis)arg2) -> bool :

            C++ signature :
                bool __ne__(class Eigen::AngleAxis<double>,class Eigen::AngleAxis<double>)
        """
    @staticmethod
    def __repr__(arg1: AngleAxis) -> str: 
        """
        __repr__( (AngleAxis)arg1) -> str :

            C++ signature :
                class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> > __repr__(class Eigen::AngleAxis<double>)
        """
    @staticmethod
    def __str__(arg1: AngleAxis) -> str: 
        """
        __str__( (AngleAxis)arg1) -> str :

            C++ signature :
                class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> > __str__(class Eigen::AngleAxis<double>)
        """
    def fromRotationMatrix(self, rotation_matrix: object) -> AngleAxis: 
        """
        fromRotationMatrix( (AngleAxis)self, (object)rotation matrix) -> AngleAxis :
            Sets *this from a 3x3 rotation matrix

            C++ signature :
                class Eigen::AngleAxis<double> {lvalue} fromRotationMatrix(class Eigen::AngleAxis<double> {lvalue},class Eigen::MatrixBase<class Eigen::Matrix<double,3,3,0,3,3> >)
        """
    def inverse(self) -> AngleAxis: 
        """
        inverse( (AngleAxis)self) -> AngleAxis :
            Return the inverse rotation.

            C++ signature :
                class Eigen::AngleAxis<double> inverse(class Eigen::AngleAxis<double> {lvalue})
        """
    def isApprox(self, other_: AngleAxis, prec: float) -> bool: 
        """
        isApprox( (AngleAxis)self, (AngleAxis)other [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to other, within the precision determined by prec.

            C++ signature :
                bool isApprox(class Eigen::AngleAxis<double>,class Eigen::AngleAxis<double> [,double])
        """
    def matrix(self) -> object: 
        """
        matrix( (AngleAxis)self) -> object :
            Returns an equivalent rotation matrix.

            C++ signature :
                class Eigen::Matrix<double,3,3,0,3,3> matrix(class Eigen::AngleAxis<double> {lvalue})
        """
    @staticmethod
    def toRotationMatrix(arg1: AngleAxis) -> object: 
        """
        toRotationMatrix( (AngleAxis)arg1) -> object :
            Constructs and returns an equivalent 3x3 rotation matrix.

            C++ signature :
                class Eigen::Matrix<double,3,3,0,3,3> toRotationMatrix(class Eigen::AngleAxis<double> {lvalue})
        """
    @property
    def angle(self) -> None:
        """
        The rotation angle.

        :type: None
        """
    @property
    def axis(self) -> None:
        """
        The rotation axis.

        :type: None
        """
    pass

class CollisionGeometry(Boost.Python.instance):
    @staticmethod
    def computeCOM(arg1: CollisionGeometry) -> object: 
        """
        computeCOM( (CollisionGeometry)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> computeCOM(class hpp::fcl::CollisionGeometry {lvalue})
        """
    @staticmethod
    def computeLocalAABB(arg1: CollisionGeometry) -> None: 
        """
        computeLocalAABB( (CollisionGeometry)arg1) -> None :

            C++ signature :
                void computeLocalAABB(class hpp::fcl::CollisionGeometry {lvalue})
        """
    @staticmethod
    def computeMomentofInertia(arg1: CollisionGeometry) -> object: 
        """
        computeMomentofInertia( (CollisionGeometry)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,3,0,3,3> computeMomentofInertia(class hpp::fcl::CollisionGeometry {lvalue})
        """
    @staticmethod
    def computeMomentofInertiaRelatedToCOM(arg1: CollisionGeometry) -> object: 
        """
        computeMomentofInertiaRelatedToCOM( (CollisionGeometry)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,3,0,3,3> computeMomentofInertiaRelatedToCOM(class hpp::fcl::CollisionGeometry {lvalue})
        """
    @staticmethod
    def computeVolume(arg1: CollisionGeometry) -> float: 
        """
        computeVolume( (CollisionGeometry)arg1) -> float :

            C++ signature :
                double computeVolume(class hpp::fcl::CollisionGeometry {lvalue})
        """
    @staticmethod
    def getNodeType(arg1: CollisionGeometry) -> NODE_TYPE: 
        """
        getNodeType( (CollisionGeometry)arg1) -> NODE_TYPE :

            C++ signature :
                enum hpp::fcl::NODE_TYPE getNodeType(class hpp::fcl::CollisionGeometry {lvalue})
        """
    @staticmethod
    def getObjectType(arg1: CollisionGeometry) -> OBJECT_TYPE: 
        """
        getObjectType( (CollisionGeometry)arg1) -> OBJECT_TYPE :

            C++ signature :
                enum hpp::fcl::OBJECT_TYPE getObjectType(class hpp::fcl::CollisionGeometry {lvalue})
        """
    def isFree(self) -> bool: 
        """
        isFree( (CollisionGeometry)self) -> bool :
            Whether the object is completely free.

            C++ signature :
                bool isFree(class hpp::fcl::CollisionGeometry {lvalue})
        """
    def isOccupied(self) -> bool: 
        """
        isOccupied( (CollisionGeometry)self) -> bool :
            Whether the object is completely occupied.

            C++ signature :
                bool isOccupied(class hpp::fcl::CollisionGeometry {lvalue})
        """
    def isUncertain(self) -> bool: 
        """
        isUncertain( (CollisionGeometry)self) -> bool :
            Whether the object has some uncertainty.

            C++ signature :
                bool isUncertain(class hpp::fcl::CollisionGeometry {lvalue})
        """
    @property
    def aabb_center(self) -> None:
        """
        AABB center in local coordinate.

        :type: None
        """
    @property
    def aabb_local(self) -> None:
        """
        AABB in local coordinate, used for tight AABB when only translation transform.

        :type: None
        """
    @property
    def aabb_radius(self) -> None:
        """
        AABB radius.

        :type: None
        """
    @property
    def cost_density(self) -> None:
        """
        Collision cost for unit volume.

        :type: None
        """
    @property
    def threshold_free(self) -> None:
        """
        Threshold for free (<= is free).

        :type: None
        """
    @property
    def threshold_occupied(self) -> None:
        """
        Threshold for occupied ( >= is occupied).

        :type: None
        """
    pass

class BVHModelBase(CollisionGeometry, Boost.Python.instance):
    @staticmethod
    def addSubModel(arg1: BVHModelBase, arg2_: StdVec_Vec3f, arg3: StdVec_Triangle) -> int: 
        """
        addSubModel( (BVHModelBase)arg1, (StdVec_Vec3f)arg2 [, (StdVec_Triangle)arg3]) -> int :

            C++ signature :
                int addSubModel(class hpp::fcl::BVHModelBase {lvalue},class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > > [,class std::vector<class hpp::fcl::Triangle,class std::allocator<class hpp::fcl::Triangle> >])
        """
    @staticmethod
    def addTriangle(arg1: BVHModelBase, arg2: object, arg3: object, arg4: object) -> int: 
        """
        addTriangle( (BVHModelBase)arg1, (object)arg2, (object)arg3, (object)arg4) -> int :

            C++ signature :
                int addTriangle(class hpp::fcl::BVHModelBase {lvalue},class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @staticmethod
    def addTriangles(arg1: BVHModelBase, arg2: object) -> int: 
        """
        addTriangles( (BVHModelBase)arg1, (object)arg2) -> int :

            C++ signature :
                int addTriangles(class hpp::fcl::BVHModelBase {lvalue},class Eigen::Matrix<__int64,-1,3,0,-1,3>)
        """
    @staticmethod
    def addVertex(arg1: BVHModelBase, arg2: object) -> int: 
        """
        addVertex( (BVHModelBase)arg1, (object)arg2) -> int :

            C++ signature :
                int addVertex(class hpp::fcl::BVHModelBase {lvalue},class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @staticmethod
    def addVertices(arg1: BVHModelBase, arg2: object) -> int: 
        """
        addVertices( (BVHModelBase)arg1, (object)arg2) -> int :

            C++ signature :
                int addVertices(class hpp::fcl::BVHModelBase {lvalue},class Eigen::Matrix<double,-1,3,0,-1,3>)
        """
    @staticmethod
    def beginModel(arg1: BVHModelBase, arg2: int, arg3: int) -> int: 
        """
        beginModel( (BVHModelBase)arg1, (int)arg2, (int)arg3) -> int :

            C++ signature :
                int beginModel(class hpp::fcl::BVHModelBase {lvalue},unsigned int,unsigned int)
        """
    @staticmethod
    def beginReplaceModel(arg1: BVHModelBase) -> int: 
        """
        beginReplaceModel( (BVHModelBase)arg1) -> int :

            C++ signature :
                int beginReplaceModel(class hpp::fcl::BVHModelBase {lvalue})
        """
    @staticmethod
    def buildConvexHull(arg1: BVHModelBase, arg2: bool, arg3: str) -> bool: 
        """
        buildConvexHull( (BVHModelBase)arg1, (bool)arg2, (str)arg3) -> bool :

            C++ signature :
                bool buildConvexHull(class hpp::fcl::BVHModelBase {lvalue},bool,char const * __ptr64)
        """
    @staticmethod
    def buildConvexRepresentation(arg1: BVHModelBase, arg2: bool) -> None: 
        """
        buildConvexRepresentation( (BVHModelBase)arg1, (bool)arg2) -> None :

            C++ signature :
                void buildConvexRepresentation(class hpp::fcl::BVHModelBase {lvalue},bool)
        """
    @staticmethod
    def endModel(arg1: BVHModelBase) -> int: 
        """
        endModel( (BVHModelBase)arg1) -> int :

            C++ signature :
                int endModel(class hpp::fcl::BVHModelBase {lvalue})
        """
    @staticmethod
    def endReplaceModel(arg1: BVHModelBase, arg2: bool, arg3: bool) -> int: 
        """
        endReplaceModel( (BVHModelBase)arg1, (bool)arg2, (bool)arg3) -> int :

            C++ signature :
                int endReplaceModel(class hpp::fcl::BVHModelBase {lvalue},bool,bool)
        """
    @staticmethod
    def getModelType(arg1: BVHModelBase) -> BVHModelType: 
        """
        getModelType( (BVHModelBase)arg1) -> BVHModelType :

            C++ signature :
                enum hpp::fcl::BVHModelType getModelType(class hpp::fcl::BVHModelBase {lvalue})
        """
    @staticmethod
    def replaceSubModel(arg1: BVHModelBase, arg2: StdVec_Vec3f) -> int: 
        """
        replaceSubModel( (BVHModelBase)arg1, (StdVec_Vec3f)arg2) -> int :

            C++ signature :
                int replaceSubModel(class hpp::fcl::BVHModelBase {lvalue},class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > >)
        """
    @staticmethod
    def replaceTriangle(arg1: BVHModelBase, arg2: object, arg3: object, arg4: object) -> int: 
        """
        replaceTriangle( (BVHModelBase)arg1, (object)arg2, (object)arg3, (object)arg4) -> int :

            C++ signature :
                int replaceTriangle(class hpp::fcl::BVHModelBase {lvalue},class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @staticmethod
    def replaceVertex(arg1: BVHModelBase, arg2: object) -> int: 
        """
        replaceVertex( (BVHModelBase)arg1, (object)arg2) -> int :

            C++ signature :
                int replaceVertex(class hpp::fcl::BVHModelBase {lvalue},class Eigen::Matrix<double,3,1,0,3,1>)
        """
    def tri_indices(self, index: int) -> Triangle: 
        """
        tri_indices( (BVHModelBase)self, (int)index) -> Triangle :
            Retrieve the triangle given by its index.

            C++ signature :
                class hpp::fcl::Triangle tri_indices(class hpp::fcl::BVHModelBase,unsigned int)
        """
    def vertex(self, index: int) -> object: 
        """
        vertex( (BVHModelBase)self, (int)index) -> object :
            Retrieve the vertex given by its index.

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> {lvalue} vertex(class hpp::fcl::BVHModelBase {lvalue},unsigned int)
        """
    @typing.overload
    def vertices(self) -> object: 
        """
        vertices( (BVHModelBase)self, (int)index) -> object :
            Retrieve the vertex given by its index.

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> {lvalue} vertices(class hpp::fcl::BVHModelBase {lvalue},unsigned int)

            C++ signature :
                class Eigen::Ref<class Eigen::Matrix<double,-1,3,1,-1,3>,0,class Eigen::OuterStride<-1> > vertices(class hpp::fcl::BVHModelBase {lvalue})
        """
    @typing.overload
    def vertices(self, index: int) -> object: ...
    @property
    def build_state(self) -> None:
        """
        :type: None
        """
    @property
    def convex(self) -> None:
        """
        :type: None
        """
    @property
    def num_tris(self) -> None:
        """
        :type: None
        """
    @property
    def num_vertices(self) -> None:
        """
        :type: None
        """
    pass

class BVHModelOBBRSS(BVHModelBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)

            C++ signature :
                void __init__(struct _object * __ptr64,class hpp::fcl::BVHModel<struct hpp::fcl::OBBRSS>)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: BVHModelOBBRSS) -> None: ...
    @staticmethod
    def clone(arg1: BVHModelOBBRSS) -> BVHModelOBBRSS: 
        """
        clone( (BVHModelOBBRSS)arg1) -> BVHModelOBBRSS :

            C++ signature :
                class hpp::fcl::BVHModel<struct hpp::fcl::OBBRSS> * __ptr64 clone(class hpp::fcl::BVHModel<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def getNumBVs(arg1: BVHModelOBBRSS) -> int: 
        """
        getNumBVs( (BVHModelOBBRSS)arg1) -> int :

            C++ signature :
                unsigned int getNumBVs(class hpp::fcl::BVHModel<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def makeParentRelative(arg1: BVHModelOBBRSS) -> None: 
        """
        makeParentRelative( (BVHModelOBBRSS)arg1) -> None :

            C++ signature :
                void makeParentRelative(class hpp::fcl::BVHModel<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def memUsage(arg1: BVHModelOBBRSS, arg2: bool) -> int: 
        """
        memUsage( (BVHModelOBBRSS)arg1, (bool)arg2) -> int :

            C++ signature :
                int memUsage(class hpp::fcl::BVHModel<struct hpp::fcl::OBBRSS> {lvalue},bool)
        """
    pass

class ShapeBase(CollisionGeometry, Boost.Python.instance):
    pass

class CPUTimes(Boost.Python.instance):
    def clear(self) -> None: 
        """
        clear( (CPUTimes)self) -> None :
            Reset the time values.

            C++ signature :
                void clear(struct hpp::fcl::CPUTimes {lvalue})
        """
    @property
    def system(self) -> None:
        """
        system time in micro seconds (us)

        :type: None
        """
    @property
    def user(self) -> None:
        """
        user time in micro seconds (us)

        :type: None
        """
    @property
    def wall(self) -> None:
        """
        wall time in micro seconds (us)

        :type: None
        """
    pass

class MeshLoader(Boost.Python.instance):
    def __init__(self, node_type: NODE_TYPE) -> None: 
        """
        __init__( (object)self [, (NODE_TYPE)node_type]) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64 [,enum hpp::fcl::NODE_TYPE])
        """
    def load(self, filename_: str, scale: object) -> BVHModelBase: 
        """
        load( (MeshLoader)self, (str)filename [, (object)scale]) -> BVHModelBase :

            C++ signature :
                class boost::shared_ptr<class hpp::fcl::BVHModelBase> load(class hpp::fcl::MeshLoader {lvalue},class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> > [,class Eigen::Matrix<double,3,1,0,3,1>])
        """
    @staticmethod
    def loadOctree(arg1: MeshLoader, arg2: str) -> CollisionGeometry: 
        """
        loadOctree( (MeshLoader)arg1, (str)arg2) -> CollisionGeometry :

            C++ signature :
                class boost::shared_ptr<class hpp::fcl::CollisionGeometry> loadOctree(class hpp::fcl::MeshLoader {lvalue},class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >)
        """
    __instance_size__ = 32
    pass

class Capsule(ShapeBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object, arg2: float, arg3: float) -> None: 
        """
        __init__( (object)arg1, (float)arg2, (float)arg3) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,double,double)
        """
    @staticmethod
    def clone(arg1: Capsule) -> Capsule: 
        """
        clone( (Capsule)arg1) -> Capsule :

            C++ signature :
                class hpp::fcl::Capsule * __ptr64 clone(class hpp::fcl::Capsule {lvalue})
        """
    @property
    def halfLength(self) -> None:
        """
        :type: None
        """
    @property
    def radius(self) -> None:
        """
        :type: None
        """
    pass

class BVHModelOBB(BVHModelBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)

            C++ signature :
                void __init__(struct _object * __ptr64,class hpp::fcl::BVHModel<struct hpp::fcl::OBB>)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: BVHModelOBB) -> None: ...
    @staticmethod
    def clone(arg1: BVHModelOBB) -> BVHModelOBB: 
        """
        clone( (BVHModelOBB)arg1) -> BVHModelOBB :

            C++ signature :
                class hpp::fcl::BVHModel<struct hpp::fcl::OBB> * __ptr64 clone(class hpp::fcl::BVHModel<struct hpp::fcl::OBB> {lvalue})
        """
    @staticmethod
    def getNumBVs(arg1: BVHModelOBB) -> int: 
        """
        getNumBVs( (BVHModelOBB)arg1) -> int :

            C++ signature :
                unsigned int getNumBVs(class hpp::fcl::BVHModel<struct hpp::fcl::OBB> {lvalue})
        """
    @staticmethod
    def makeParentRelative(arg1: BVHModelOBB) -> None: 
        """
        makeParentRelative( (BVHModelOBB)arg1) -> None :

            C++ signature :
                void makeParentRelative(class hpp::fcl::BVHModel<struct hpp::fcl::OBB> {lvalue})
        """
    @staticmethod
    def memUsage(arg1: BVHModelOBB, arg2: bool) -> int: 
        """
        memUsage( (BVHModelOBB)arg1, (bool)arg2) -> int :

            C++ signature :
                int memUsage(class hpp::fcl::BVHModel<struct hpp::fcl::OBB> {lvalue},bool)
        """
    pass

class CollisionObject(Boost.Python.instance):
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: CollisionGeometry) -> None: 
        """
        __init__( (object)arg1, (CollisionGeometry)arg2) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,class boost::shared_ptr<class hpp::fcl::CollisionGeometry>)

            C++ signature :
                void __init__(struct _object * __ptr64,class boost::shared_ptr<class hpp::fcl::CollisionGeometry>,class hpp::fcl::Transform3f)

            C++ signature :
                void __init__(struct _object * __ptr64,class boost::shared_ptr<class hpp::fcl::CollisionGeometry>,class Eigen::Matrix<double,3,3,0,3,3>,class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: CollisionGeometry, arg3: Transform3f) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: CollisionGeometry, arg3: object, arg4: object) -> None: ...
    @staticmethod
    def collisionGeometry(arg1: CollisionObject) -> CollisionGeometry: 
        """
        collisionGeometry( (CollisionObject)arg1) -> CollisionGeometry :

            C++ signature :
                class boost::shared_ptr<class hpp::fcl::CollisionGeometry> collisionGeometry(class hpp::fcl::CollisionObject {lvalue})
        """
    @staticmethod
    def computeAABB(arg1: CollisionObject) -> None: 
        """
        computeAABB( (CollisionObject)arg1) -> None :

            C++ signature :
                void computeAABB(class hpp::fcl::CollisionObject {lvalue})
        """
    @staticmethod
    def getAABB(arg1: CollisionObject) -> AABB: 
        """
        getAABB( (CollisionObject)arg1) -> AABB :

            C++ signature :
                class hpp::fcl::AABB getAABB(class hpp::fcl::CollisionObject {lvalue})
        """
    @staticmethod
    def getNodeType(arg1: CollisionObject) -> NODE_TYPE: 
        """
        getNodeType( (CollisionObject)arg1) -> NODE_TYPE :

            C++ signature :
                enum hpp::fcl::NODE_TYPE getNodeType(class hpp::fcl::CollisionObject {lvalue})
        """
    @staticmethod
    def getObjectType(arg1: CollisionObject) -> OBJECT_TYPE: 
        """
        getObjectType( (CollisionObject)arg1) -> OBJECT_TYPE :

            C++ signature :
                enum hpp::fcl::OBJECT_TYPE getObjectType(class hpp::fcl::CollisionObject {lvalue})
        """
    @staticmethod
    def getRotation(arg1: CollisionObject) -> object: 
        """
        getRotation( (CollisionObject)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,3,0,3,3> getRotation(class hpp::fcl::CollisionObject {lvalue})
        """
    @staticmethod
    def getTransform(arg1: CollisionObject) -> Transform3f: 
        """
        getTransform( (CollisionObject)arg1) -> Transform3f :

            C++ signature :
                class hpp::fcl::Transform3f getTransform(class hpp::fcl::CollisionObject {lvalue})
        """
    @staticmethod
    def getTranslation(arg1: CollisionObject) -> object: 
        """
        getTranslation( (CollisionObject)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> getTranslation(class hpp::fcl::CollisionObject {lvalue})
        """
    @staticmethod
    def isIdentityTransform(arg1: CollisionObject) -> bool: 
        """
        isIdentityTransform( (CollisionObject)arg1) -> bool :

            C++ signature :
                bool isIdentityTransform(class hpp::fcl::CollisionObject {lvalue})
        """
    @staticmethod
    def setIdentityTransform(arg1: CollisionObject) -> None: 
        """
        setIdentityTransform( (CollisionObject)arg1) -> None :

            C++ signature :
                void setIdentityTransform(class hpp::fcl::CollisionObject {lvalue})
        """
    @staticmethod
    def setRotation(arg1: CollisionObject, arg2: object) -> None: 
        """
        setRotation( (CollisionObject)arg1, (object)arg2) -> None :

            C++ signature :
                void setRotation(class hpp::fcl::CollisionObject {lvalue},class Eigen::Matrix<double,3,3,0,3,3>)
        """
    @staticmethod
    def setTransform(arg1: CollisionObject, arg2: Transform3f) -> None: 
        """
        setTransform( (CollisionObject)arg1, (Transform3f)arg2) -> None :

            C++ signature :
                void setTransform(class hpp::fcl::CollisionObject {lvalue},class hpp::fcl::Transform3f)
        """
    @staticmethod
    def setTranslation(arg1: CollisionObject, arg2: object) -> None: 
        """
        setTranslation( (CollisionObject)arg1, (object)arg2) -> None :

            C++ signature :
                void setTranslation(class hpp::fcl::CollisionObject {lvalue},class Eigen::Matrix<double,3,1,0,3,1>)
        """
    pass

class QueryRequest(Boost.Python.instance):
    @staticmethod
    def updateGuess(arg1: QueryRequest, arg2: QueryResult) -> None: 
        """
        updateGuess( (QueryRequest)arg1, (QueryResult)arg2) -> None :

            C++ signature :
                void updateGuess(struct hpp::fcl::QueryRequest {lvalue},struct hpp::fcl::QueryResult)
        """
    @property
    def cached_gjk_guess(self) -> None:
        """
        :type: None
        """
    @property
    def cached_support_func_guess(self) -> None:
        """
        :type: None
        """
    @property
    def enable_cached_gjk_guess(self) -> None:
        """
        :type: None
        """
    @property
    def enable_timings(self) -> None:
        """
        :type: None
        """
    pass

class QueryResult(Boost.Python.instance):
    @property
    def cached_gjk_guess(self) -> None:
        """
        :type: None
        """
    @property
    def cached_support_func_guess(self) -> None:
        """
        :type: None
        """
    @property
    def timings(self) -> None:
        """
        :type: None
        """
    pass

class ComputeCollision(Boost.Python.instance):
    @staticmethod
    def __call__(arg1: ComputeCollision, arg2: Transform3f, arg3: Transform3f, arg4: CollisionRequest, arg5: CollisionResult) -> int: 
        """
        __call__( (ComputeCollision)arg1, (Transform3f)arg2, (Transform3f)arg3, (CollisionRequest)arg4, (CollisionResult)arg5) -> int :

            C++ signature :
                unsigned __int64 __call__(class hpp::fcl::ComputeCollision {lvalue},class hpp::fcl::Transform3f,class hpp::fcl::Transform3f,struct hpp::fcl::CollisionRequest {lvalue},struct hpp::fcl::CollisionResult {lvalue})
        """
    @staticmethod
    def __init__(arg1: object, arg2: CollisionGeometry, arg3: CollisionGeometry) -> None: 
        """
        __init__( (object)arg1, (CollisionGeometry)arg2, (CollisionGeometry)arg3) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,class hpp::fcl::CollisionGeometry const * __ptr64,class hpp::fcl::CollisionGeometry const * __ptr64)
        """
    pass

class ComputeDistance(Boost.Python.instance):
    @staticmethod
    def __call__(arg1: ComputeDistance, arg2: Transform3f, arg3: Transform3f, arg4: DistanceRequest, arg5: DistanceResult) -> float: 
        """
        __call__( (ComputeDistance)arg1, (Transform3f)arg2, (Transform3f)arg3, (DistanceRequest)arg4, (DistanceResult)arg5) -> float :

            C++ signature :
                double __call__(class hpp::fcl::ComputeDistance {lvalue},class hpp::fcl::Transform3f,class hpp::fcl::Transform3f,struct hpp::fcl::DistanceRequest {lvalue},struct hpp::fcl::DistanceResult {lvalue})
        """
    @staticmethod
    def __init__(arg1: object, arg2: CollisionGeometry, arg3: CollisionGeometry) -> None: 
        """
        __init__( (object)arg1, (CollisionGeometry)arg2, (CollisionGeometry)arg3) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,class hpp::fcl::CollisionGeometry const * __ptr64,class hpp::fcl::CollisionGeometry const * __ptr64)
        """
    pass

class Cone(ShapeBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object, arg2: float, arg3: float) -> None: 
        """
        __init__( (object)arg1, (float)arg2, (float)arg3) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,double,double)
        """
    @staticmethod
    def clone(arg1: Cone) -> Cone: 
        """
        clone( (Cone)arg1) -> Cone :

            C++ signature :
                class hpp::fcl::Cone * __ptr64 clone(class hpp::fcl::Cone {lvalue})
        """
    @property
    def halfLength(self) -> None:
        """
        :type: None
        """
    @property
    def radius(self) -> None:
        """
        :type: None
        """
    pass

class Contact(Boost.Python.instance):
    @staticmethod
    def __eq__(arg1: Contact, arg2: Contact) -> object: 
        """
        __eq__( (Contact)arg1, (Contact)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __eq__(struct hpp::fcl::Contact {lvalue},struct hpp::fcl::Contact)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: CollisionGeometry, arg3: CollisionGeometry, arg4: int, arg5: int) -> None: 
        """
        __init__( (object)self) -> None :
            Default constructor

            C++ signature :
                void __init__(struct _object * __ptr64)

            C++ signature :
                void __init__(struct _object * __ptr64,class hpp::fcl::CollisionGeometry const * __ptr64,class hpp::fcl::CollisionGeometry const * __ptr64,int,int)

            C++ signature :
                void __init__(struct _object * __ptr64,class hpp::fcl::CollisionGeometry const * __ptr64,class hpp::fcl::CollisionGeometry const * __ptr64,int,int,class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<double,3,1,0,3,1>,double)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: CollisionGeometry, arg3: CollisionGeometry, arg4: int, arg5: int, arg6: object, arg7: object, arg8: float) -> None: ...
    @typing.overload
    def __init__(self) -> None: ...
    @staticmethod
    def __ne__(arg1: Contact, arg2: Contact) -> object: 
        """
        __ne__( (Contact)arg1, (Contact)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __ne__(struct hpp::fcl::Contact {lvalue},struct hpp::fcl::Contact)
        """
    @property
    def b1(self) -> None:
        """
        :type: None
        """
    @property
    def b2(self) -> None:
        """
        :type: None
        """
    @property
    def normal(self) -> None:
        """
        :type: None
        """
    @property
    def o1(self) -> None:
        """
        :type: None
        """
    @property
    def o2(self) -> None:
        """
        :type: None
        """
    @property
    def penetration_depth(self) -> None:
        """
        :type: None
        """
    @property
    def pos(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 96
    pass

class ConvexBase(ShapeBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    def clone(arg1: ConvexBase) -> ConvexBase: 
        """
        clone( (ConvexBase)arg1) -> ConvexBase :

            C++ signature :
                class hpp::fcl::ConvexBase * __ptr64 clone(class hpp::fcl::ConvexBase {lvalue})
        """
    @staticmethod
    def convexHull(arg1: StdVec_Vec3f, arg2: bool, arg3: str) -> ConvexBase: 
        """
        convexHull( (StdVec_Vec3f)arg1, (bool)arg2, (str)arg3) -> ConvexBase :

            C++ signature :
                class hpp::fcl::ConvexBase * __ptr64 convexHull(class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > >,bool,char const * __ptr64)
        """
    @staticmethod
    def neighbors(arg1: ConvexBase, arg2: int) -> list: 
        """
        neighbors( (ConvexBase)arg1, (int)arg2) -> list :

            C++ signature :
                class boost::python::list neighbors(class hpp::fcl::ConvexBase,unsigned int)
        """
    def point(self, index: int) -> object: 
        """
        point( (ConvexBase)self, (int)index) -> object :
            Retrieve the point given by its index.

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> {lvalue} point(class hpp::fcl::ConvexBase,unsigned int)
        """
    @typing.overload
    def points(self) -> object: 
        """
        points( (ConvexBase)self, (int)index) -> object :
            Retrieve the point given by its index.

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> {lvalue} points(class hpp::fcl::ConvexBase,unsigned int)

            C++ signature :
                class Eigen::Ref<class Eigen::Matrix<double,-1,3,1,-1,3>,0,class Eigen::OuterStride<-1> > points(class hpp::fcl::ConvexBase)
        """
    @typing.overload
    def points(self, index: int) -> object: ...
    @property
    def center(self) -> None:
        """
        :type: None
        """
    @property
    def num_points(self) -> None:
        """
        :type: None
        """
    pass

class Convex(ConvexBase, ShapeBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object, arg2: StdVec_Vec3f, arg3: StdVec_Triangle) -> object: 
        """
        __init__( (object)arg1, (StdVec_Vec3f)arg2, (StdVec_Triangle)arg3) -> object :

            C++ signature :
                void * __ptr64 __init__(class boost::python::api::object,class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > >,class std::vector<class hpp::fcl::Triangle,class std::allocator<class hpp::fcl::Triangle> >)
        """
    @staticmethod
    def polygons(arg1: Convex, arg2: int) -> Triangle: 
        """
        polygons( (Convex)arg1, (int)arg2) -> Triangle :

            C++ signature :
                class hpp::fcl::Triangle polygons(class hpp::fcl::Convex<class hpp::fcl::Triangle>,unsigned int)
        """
    @property
    def num_polygons(self) -> None:
        """
        :type: None
        """
    pass

class Cylinder(ShapeBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object, arg2: float, arg3: float) -> None: 
        """
        __init__( (object)arg1, (float)arg2, (float)arg3) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,double,double)
        """
    @staticmethod
    def clone(arg1: Cylinder) -> Cylinder: 
        """
        clone( (Cylinder)arg1) -> Cylinder :

            C++ signature :
                class hpp::fcl::Cylinder * __ptr64 clone(class hpp::fcl::Cylinder {lvalue})
        """
    @property
    def halfLength(self) -> None:
        """
        :type: None
        """
    @property
    def radius(self) -> None:
        """
        :type: None
        """
    pass

class DistanceRequest(QueryRequest, Boost.Python.instance):
    def __init__(self, enable_nearest_points_: bool, rel_err_: float, abs_err: float) -> None: 
        """
        __init__( (object)self [, (bool)enable_nearest_points [, (float)rel_err [, (float)abs_err]]]) -> None :
            Constructor

            C++ signature :
                void __init__(struct _object * __ptr64 [,bool [,double [,double]]])
        """
    @property
    def abs_err(self) -> None:
        """
        :type: None
        """
    @property
    def enable_nearest_points(self) -> None:
        """
        :type: None
        """
    @property
    def rel_err(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 88
    pass

class DistanceResult(QueryResult, Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    def clear(arg1: DistanceResult) -> None: 
        """
        clear( (DistanceResult)arg1) -> None :

            C++ signature :
                void clear(struct hpp::fcl::DistanceResult {lvalue})
        """
    @staticmethod
    def getNearestPoint1(arg1: DistanceResult) -> object: 
        """
        getNearestPoint1( (DistanceResult)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> getNearestPoint1(struct hpp::fcl::DistanceResult)
        """
    @staticmethod
    def getNearestPoint2(arg1: DistanceResult) -> object: 
        """
        getNearestPoint2( (DistanceResult)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> getNearestPoint2(struct hpp::fcl::DistanceResult)
        """
    @property
    def b1(self) -> None:
        """
        :type: None
        """
    @property
    def b2(self) -> None:
        """
        :type: None
        """
    @property
    def min_distance(self) -> None:
        """
        :type: None
        """
    @property
    def normal(self) -> None:
        """
        :type: None
        """
    @property
    def o1(self) -> None:
        """
        :type: None
        """
    @property
    def o2(self) -> None:
        """
        :type: None
        """
    pass

class Exception(Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object, arg2: str) -> None: 
        """
        __init__( (object)arg1, (str)arg2) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >)
        """
    @property
    def message(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 72
    pass

class GJK(Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object, arg2: int, arg3: float) -> None: 
        """
        __init__( (object)arg1, (int)arg2, (float)arg3) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,unsigned int,double)
        """
    @staticmethod
    def evaluate(arg1: GJK, arg2: MinkowskiDiff, arg3: object, arg4: object) -> GJKStatus: 
        """
        evaluate( (GJK)arg1, (MinkowskiDiff)arg2, (object)arg3, (object)arg4) -> GJKStatus :

            C++ signature :
                enum hpp::fcl::details::GJK::Status evaluate(struct hpp::fcl::details::GJK {lvalue},struct hpp::fcl::details::MinkowskiDiff,class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<int,2,1,0,2,1>)
        """
    @staticmethod
    def getClosestPoints(arg1: GJK, arg2: MinkowskiDiff, arg3: object, arg4: object) -> bool: 
        """
        getClosestPoints( (GJK)arg1, (MinkowskiDiff)arg2, (object)arg3, (object)arg4) -> bool :

            C++ signature :
                bool getClosestPoints(struct hpp::fcl::details::GJK {lvalue},struct hpp::fcl::details::MinkowskiDiff,class Eigen::Matrix<double,3,1,0,3,1> {lvalue},class Eigen::Matrix<double,3,1,0,3,1> {lvalue})
        """
    @staticmethod
    def getGuessFromSimplex(arg1: GJK) -> object: 
        """
        getGuessFromSimplex( (GJK)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> getGuessFromSimplex(struct hpp::fcl::details::GJK {lvalue})
        """
    @staticmethod
    def hasClosestPoints(arg1: GJK) -> bool: 
        """
        hasClosestPoints( (GJK)arg1) -> bool :

            C++ signature :
                bool hasClosestPoints(struct hpp::fcl::details::GJK {lvalue})
        """
    @staticmethod
    def hasPenetrationInformation(arg1: GJK, arg2: MinkowskiDiff) -> bool: 
        """
        hasPenetrationInformation( (GJK)arg1, (MinkowskiDiff)arg2) -> bool :

            C++ signature :
                bool hasPenetrationInformation(struct hpp::fcl::details::GJK {lvalue},struct hpp::fcl::details::MinkowskiDiff)
        """
    @staticmethod
    def setDistanceEarlyBreak(arg1: GJK, arg2: float) -> None: 
        """
        setDistanceEarlyBreak( (GJK)arg1, (float)arg2) -> None :

            C++ signature :
                void setDistanceEarlyBreak(struct hpp::fcl::details::GJK {lvalue},double)
        """
    @property
    def distance(self) -> None:
        """
        :type: None
        """
    @property
    def ray(self) -> None:
        """
        :type: None
        """
    @property
    def support_hint(self) -> None:
        """
        :type: None
        """
    pass

class Halfspace(ShapeBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1, (object)arg2, (float)arg3) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Matrix<double,3,1,0,3,1>,double)

            C++ signature :
                void __init__(struct _object * __ptr64,double,double,double,double)

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: float, arg3: float, arg4: float, arg5: float) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: object, arg3: float) -> None: ...
    @staticmethod
    def clone(arg1: Halfspace) -> Halfspace: 
        """
        clone( (Halfspace)arg1) -> Halfspace :

            C++ signature :
                class hpp::fcl::Halfspace * __ptr64 clone(class hpp::fcl::Halfspace {lvalue})
        """
    @property
    def d(self) -> None:
        """
        :type: None
        """
    @property
    def n(self) -> None:
        """
        :type: None
        """
    pass

class HeightFieldAABB(CollisionGeometry, Boost.Python.instance):
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)

            C++ signature :
                void __init__(struct _object * __ptr64,class hpp::fcl::HeightField<class hpp::fcl::AABB>)

            C++ signature :
                void __init__(struct _object * __ptr64,double,double,class Eigen::Matrix<double,-1,-1,0,-1,-1> [,double])
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: HeightFieldAABB) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: float, arg3: float, arg4_: object, arg5: float) -> None: ...
    @staticmethod
    def clone(arg1: HeightFieldAABB) -> HeightFieldAABB: 
        """
        clone( (HeightFieldAABB)arg1) -> HeightFieldAABB :

            C++ signature :
                class hpp::fcl::HeightField<class hpp::fcl::AABB> * __ptr64 clone(class hpp::fcl::HeightField<class hpp::fcl::AABB> {lvalue})
        """
    @staticmethod
    def getBV(arg1: HeightFieldAABB, arg2: int) -> object: 
        """
        getBV( (HeightFieldAABB)arg1, (int)arg2) -> object :

            C++ signature :
                struct hpp::fcl::HFNode<class hpp::fcl::AABB> {lvalue} getBV(class hpp::fcl::HeightField<class hpp::fcl::AABB> {lvalue},unsigned int)
        """
    @staticmethod
    def getHeights(arg1: HeightFieldAABB) -> object: 
        """
        getHeights( (HeightFieldAABB)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,-1,-1,0,-1,-1> getHeights(class hpp::fcl::HeightField<class hpp::fcl::AABB> {lvalue})
        """
    @staticmethod
    def getMaxHeight(arg1: HeightFieldAABB) -> float: 
        """
        getMaxHeight( (HeightFieldAABB)arg1) -> float :

            C++ signature :
                double getMaxHeight(class hpp::fcl::HeightField<class hpp::fcl::AABB> {lvalue})
        """
    @staticmethod
    def getMinHeight(arg1: HeightFieldAABB) -> float: 
        """
        getMinHeight( (HeightFieldAABB)arg1) -> float :

            C++ signature :
                double getMinHeight(class hpp::fcl::HeightField<class hpp::fcl::AABB> {lvalue})
        """
    @staticmethod
    def getNodeType(arg1: HeightFieldAABB) -> NODE_TYPE: 
        """
        getNodeType( (HeightFieldAABB)arg1) -> NODE_TYPE :

            C++ signature :
                enum hpp::fcl::NODE_TYPE getNodeType(class hpp::fcl::HeightField<class hpp::fcl::AABB> {lvalue})
        """
    @staticmethod
    def getXDim(arg1: HeightFieldAABB) -> float: 
        """
        getXDim( (HeightFieldAABB)arg1) -> float :

            C++ signature :
                double getXDim(class hpp::fcl::HeightField<class hpp::fcl::AABB> {lvalue})
        """
    @staticmethod
    def getXGrid(arg1: HeightFieldAABB) -> object: 
        """
        getXGrid( (HeightFieldAABB)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,-1,1,0,-1,1> getXGrid(class hpp::fcl::HeightField<class hpp::fcl::AABB> {lvalue})
        """
    @staticmethod
    def getYDim(arg1: HeightFieldAABB) -> float: 
        """
        getYDim( (HeightFieldAABB)arg1) -> float :

            C++ signature :
                double getYDim(class hpp::fcl::HeightField<class hpp::fcl::AABB> {lvalue})
        """
    @staticmethod
    def getYGrid(arg1: HeightFieldAABB) -> object: 
        """
        getYGrid( (HeightFieldAABB)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,-1,1,0,-1,1> getYGrid(class hpp::fcl::HeightField<class hpp::fcl::AABB> {lvalue})
        """
    @staticmethod
    def updateHeights(arg1: HeightFieldAABB, arg2: object) -> None: 
        """
        updateHeights( (HeightFieldAABB)arg1, (object)arg2) -> None :

            C++ signature :
                void updateHeights(class hpp::fcl::HeightField<class hpp::fcl::AABB> {lvalue},class Eigen::Matrix<double,-1,-1,0,-1,-1>)
        """
    pass

class HeightFieldOBBRSS(CollisionGeometry, Boost.Python.instance):
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)

            C++ signature :
                void __init__(struct _object * __ptr64,class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS>)

            C++ signature :
                void __init__(struct _object * __ptr64,double,double,class Eigen::Matrix<double,-1,-1,0,-1,-1> [,double])
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: HeightFieldOBBRSS) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: float, arg3: float, arg4_: object, arg5: float) -> None: ...
    @staticmethod
    def clone(arg1: HeightFieldOBBRSS) -> HeightFieldOBBRSS: 
        """
        clone( (HeightFieldOBBRSS)arg1) -> HeightFieldOBBRSS :

            C++ signature :
                class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> * __ptr64 clone(class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def getBV(arg1: HeightFieldOBBRSS, arg2: int) -> object: 
        """
        getBV( (HeightFieldOBBRSS)arg1, (int)arg2) -> object :

            C++ signature :
                struct hpp::fcl::HFNode<struct hpp::fcl::OBBRSS> {lvalue} getBV(class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> {lvalue},unsigned int)
        """
    @staticmethod
    def getHeights(arg1: HeightFieldOBBRSS) -> object: 
        """
        getHeights( (HeightFieldOBBRSS)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,-1,-1,0,-1,-1> getHeights(class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def getMaxHeight(arg1: HeightFieldOBBRSS) -> float: 
        """
        getMaxHeight( (HeightFieldOBBRSS)arg1) -> float :

            C++ signature :
                double getMaxHeight(class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def getMinHeight(arg1: HeightFieldOBBRSS) -> float: 
        """
        getMinHeight( (HeightFieldOBBRSS)arg1) -> float :

            C++ signature :
                double getMinHeight(class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def getNodeType(arg1: HeightFieldOBBRSS) -> NODE_TYPE: 
        """
        getNodeType( (HeightFieldOBBRSS)arg1) -> NODE_TYPE :

            C++ signature :
                enum hpp::fcl::NODE_TYPE getNodeType(class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def getXDim(arg1: HeightFieldOBBRSS) -> float: 
        """
        getXDim( (HeightFieldOBBRSS)arg1) -> float :

            C++ signature :
                double getXDim(class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def getXGrid(arg1: HeightFieldOBBRSS) -> object: 
        """
        getXGrid( (HeightFieldOBBRSS)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,-1,1,0,-1,1> getXGrid(class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def getYDim(arg1: HeightFieldOBBRSS) -> float: 
        """
        getYDim( (HeightFieldOBBRSS)arg1) -> float :

            C++ signature :
                double getYDim(class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def getYGrid(arg1: HeightFieldOBBRSS) -> object: 
        """
        getYGrid( (HeightFieldOBBRSS)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,-1,1,0,-1,1> getYGrid(class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> {lvalue})
        """
    @staticmethod
    def updateHeights(arg1: HeightFieldOBBRSS, arg2: object) -> None: 
        """
        updateHeights( (HeightFieldOBBRSS)arg1, (object)arg2) -> None :

            C++ signature :
                void updateHeights(class hpp::fcl::HeightField<struct hpp::fcl::OBBRSS> {lvalue},class Eigen::Matrix<double,-1,-1,0,-1,-1>)
        """
    pass

class CachedMeshLoader(MeshLoader, Boost.Python.instance):
    def __init__(self, node_type: NODE_TYPE) -> None: 
        """
        __init__( (object)self [, (NODE_TYPE)node_type]) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64 [,enum hpp::fcl::NODE_TYPE])
        """
    __instance_size__ = 32
    pass

class MinkowskiDiff(Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    @typing.overload
    def set(arg1: MinkowskiDiff, arg2: ShapeBase, arg3: ShapeBase) -> None: 
        """
        set( (MinkowskiDiff)arg1, (ShapeBase)arg2, (ShapeBase)arg3) -> None :

            C++ signature :
                void set(struct hpp::fcl::details::MinkowskiDiff {lvalue},class hpp::fcl::ShapeBase const * __ptr64,class hpp::fcl::ShapeBase const * __ptr64)

            C++ signature :
                void set(struct hpp::fcl::details::MinkowskiDiff {lvalue},class hpp::fcl::ShapeBase const * __ptr64,class hpp::fcl::ShapeBase const * __ptr64,class hpp::fcl::Transform3f,class hpp::fcl::Transform3f)
        """
    @staticmethod
    @typing.overload
    def set(arg1: MinkowskiDiff, arg2: ShapeBase, arg3: ShapeBase, arg4: Transform3f, arg5: Transform3f) -> None: ...
    @staticmethod
    def support(arg1: MinkowskiDiff, arg2: object, arg3: bool, arg4: object, arg5: object, arg6: object) -> None: 
        """
        support( (MinkowskiDiff)arg1, (object)arg2, (bool)arg3, (object)arg4, (object)arg5, (object)arg6) -> None :

            C++ signature :
                void support(struct hpp::fcl::details::MinkowskiDiff {lvalue},class Eigen::Matrix<double,3,1,0,3,1>,bool,class Eigen::Matrix<double,3,1,0,3,1> {lvalue},class Eigen::Matrix<double,3,1,0,3,1> {lvalue},class Eigen::Matrix<int,2,1,0,2,1> {lvalue})
        """
    @staticmethod
    def support0(arg1: MinkowskiDiff, arg2: object, arg3: bool, arg4: int) -> object: 
        """
        support0( (MinkowskiDiff)arg1, (object)arg2, (bool)arg3, (int)arg4) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> support0(struct hpp::fcl::details::MinkowskiDiff {lvalue},class Eigen::Matrix<double,3,1,0,3,1>,bool,int {lvalue})
        """
    @staticmethod
    def support1(arg1: MinkowskiDiff, arg2: object, arg3: bool, arg4: int) -> object: 
        """
        support1( (MinkowskiDiff)arg1, (object)arg2, (bool)arg3, (int)arg4) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> support1(struct hpp::fcl::details::MinkowskiDiff {lvalue},class Eigen::Matrix<double,3,1,0,3,1>,bool,int {lvalue})
        """
    @property
    def inflation(self) -> None:
        """
        :type: None
        """
    pass

class OcTree(CollisionGeometry, Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object, arg2: float) -> None: 
        """
        __init__( (object)arg1, (float)arg2) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,double)
        """
    @staticmethod
    def getDefaultOccupancy(arg1: OcTree) -> float: 
        """
        getDefaultOccupancy( (OcTree)arg1) -> float :

            C++ signature :
                double getDefaultOccupancy(class hpp::fcl::OcTree {lvalue})
        """
    @staticmethod
    def getFreeThres(arg1: OcTree) -> float: 
        """
        getFreeThres( (OcTree)arg1) -> float :

            C++ signature :
                double getFreeThres(class hpp::fcl::OcTree {lvalue})
        """
    @staticmethod
    def getOccupancyThres(arg1: OcTree) -> float: 
        """
        getOccupancyThres( (OcTree)arg1) -> float :

            C++ signature :
                double getOccupancyThres(class hpp::fcl::OcTree {lvalue})
        """
    @staticmethod
    def getRootBV(arg1: OcTree) -> AABB: 
        """
        getRootBV( (OcTree)arg1) -> AABB :

            C++ signature :
                class hpp::fcl::AABB getRootBV(class hpp::fcl::OcTree {lvalue})
        """
    @staticmethod
    def getTreeDepth(arg1: OcTree) -> int: 
        """
        getTreeDepth( (OcTree)arg1) -> int :

            C++ signature :
                unsigned int getTreeDepth(class hpp::fcl::OcTree {lvalue})
        """
    @staticmethod
    def setCellDefaultOccupancy(arg1: OcTree, arg2: float) -> None: 
        """
        setCellDefaultOccupancy( (OcTree)arg1, (float)arg2) -> None :

            C++ signature :
                void setCellDefaultOccupancy(class hpp::fcl::OcTree {lvalue},double)
        """
    @staticmethod
    def setFreeThres(arg1: OcTree, arg2: float) -> None: 
        """
        setFreeThres( (OcTree)arg1, (float)arg2) -> None :

            C++ signature :
                void setFreeThres(class hpp::fcl::OcTree {lvalue},double)
        """
    @staticmethod
    def setOccupancyThres(arg1: OcTree, arg2: float) -> None: 
        """
        setOccupancyThres( (OcTree)arg1, (float)arg2) -> None :

            C++ signature :
                void setOccupancyThres(class hpp::fcl::OcTree {lvalue},double)
        """
    pass

class Plane(ShapeBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1, (object)arg2, (float)arg3) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Matrix<double,3,1,0,3,1>,double)

            C++ signature :
                void __init__(struct _object * __ptr64,double,double,double,double)

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: float, arg3: float, arg4: float, arg5: float) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: object, arg3: float) -> None: ...
    @staticmethod
    def clone(arg1: Plane) -> Plane: 
        """
        clone( (Plane)arg1) -> Plane :

            C++ signature :
                class hpp::fcl::Plane * __ptr64 clone(class hpp::fcl::Plane {lvalue})
        """
    @property
    def d(self) -> None:
        """
        :type: None
        """
    @property
    def n(self) -> None:
        """
        :type: None
        """
    pass

class Quaternion(Boost.Python.instance):
    """
    Quaternion representing rotation.

    Supported operations ('q is a Quaternion, 'v' is a Vector3): 'q*q' (rotation composition), 'q*=q', 'q*v' (rotating 'v' by 'q'), 'q==q', 'q!=q', 'q[0..3]'.
    """
    @staticmethod
    def FromTwoVectors(a: object, b: object) -> Quaternion: 
        """
        FromTwoVectors( (object)a, (object)b) -> Quaternion :
            Returns the quaternion which transforms a into b through a rotation.

            C++ signature :
                class Eigen::Quaternion<double,0> * __ptr64 FromTwoVectors(class Eigen::Ref<class Eigen::Matrix<double,3,1,0,3,1>,0,class Eigen::InnerStride<1> >,class Eigen::Ref<class Eigen::Matrix<double,3,1,0,3,1>,0,class Eigen::InnerStride<1> >)
        """
    @staticmethod
    def Identity() -> Quaternion: 
        """
        Identity() -> Quaternion :
            Returns a quaternion representing an identity rotation.

            C++ signature :
                class Eigen::Quaternion<double,0> * __ptr64 Identity()
        """
    @staticmethod
    def __abs__(arg1: Quaternion) -> float: 
        """
        __abs__( (Quaternion)arg1) -> float :

            C++ signature :
                double __abs__(class Eigen::Quaternion<double,0> {lvalue})
        """
    @staticmethod
    def __eq__(arg1: Quaternion, arg2: Quaternion) -> bool: 
        """
        __eq__( (Quaternion)arg1, (Quaternion)arg2) -> bool :

            C++ signature :
                bool __eq__(class Eigen::Quaternion<double,0>,class Eigen::Quaternion<double,0>)
        """
    @staticmethod
    def __getitem__(arg1: Quaternion, arg2: int) -> float: 
        """
        __getitem__( (Quaternion)arg1, (int)arg2) -> float :

            C++ signature :
                double __getitem__(class Eigen::Quaternion<double,0>,int)
        """
    @staticmethod
    def __imul__(arg1: object, arg2: Quaternion) -> object: 
        """
        __imul__( (object)arg1, (Quaternion)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __imul__(struct boost::python::back_reference<class Eigen::Quaternion<double,0> & __ptr64>,class Eigen::Quaternion<double,0>)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> object: 
        """
        __init__( (object)arg1, (object)R) -> object :
            Initialize from rotation matrix.
            	R : a rotation matrix 3x3.

            C++ signature :
                void * __ptr64 __init__(class boost::python::api::object,class Eigen::Ref<class Eigen::Matrix<double,3,3,0,3,3>,0,class Eigen::OuterStride<-1> >)

            C++ signature :
                void * __ptr64 __init__(class boost::python::api::object,class Eigen::AngleAxis<double>)

            C++ signature :
                void * __ptr64 __init__(class boost::python::api::object,class Eigen::Quaternion<double,0>)

            C++ signature :
                void * __ptr64 __init__(class boost::python::api::object,class Eigen::Ref<class Eigen::Matrix<double,3,1,0,3,1>,0,class Eigen::InnerStride<1> >,class Eigen::Ref<class Eigen::Matrix<double,3,1,0,3,1>,0,class Eigen::InnerStride<1> >)

            C++ signature :
                void * __ptr64 __init__(class boost::python::api::object,class Eigen::Ref<class Eigen::Matrix<double,4,1,0,4,1>,0,class Eigen::InnerStride<1> >)

            C++ signature :
                void * __ptr64 __init__(class boost::python::api::object)

            C++ signature :
                void * __ptr64 __init__(class boost::python::api::object,double,double,double,double)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, R: object) -> object: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, aa: AngleAxis) -> object: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, quat: Quaternion) -> object: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, u: object, v: object) -> object: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, vec4: object) -> object: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, w: float, x: float, y: float, z: float) -> object: ...
    @staticmethod
    def __len__() -> int: 
        """
        __len__() -> int :

            C++ signature :
                int __len__()
        """
    @staticmethod
    @typing.overload
    def __mul__(arg1: Quaternion, arg2: Quaternion) -> object: 
        """
        __mul__( (Quaternion)arg1, (Quaternion)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __mul__(class Eigen::Quaternion<double,0> {lvalue},class Eigen::Quaternion<double,0>)

            C++ signature :
                struct _object * __ptr64 __mul__(class Eigen::Quaternion<double,0> {lvalue},class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @staticmethod
    @typing.overload
    def __mul__(arg1: Quaternion, arg2: object) -> object: ...
    @staticmethod
    def __ne__(arg1: Quaternion, arg2: Quaternion) -> bool: 
        """
        __ne__( (Quaternion)arg1, (Quaternion)arg2) -> bool :

            C++ signature :
                bool __ne__(class Eigen::Quaternion<double,0>,class Eigen::Quaternion<double,0>)
        """
    @staticmethod
    def __repr__(arg1: Quaternion) -> str: 
        """
        __repr__( (Quaternion)arg1) -> str :

            C++ signature :
                class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> > __repr__(class Eigen::Quaternion<double,0>)
        """
    @staticmethod
    def __setitem__(arg1: Quaternion, arg2: int, arg3: float) -> None: 
        """
        __setitem__( (Quaternion)arg1, (int)arg2, (float)arg3) -> None :

            C++ signature :
                void __setitem__(class Eigen::Quaternion<double,0> {lvalue},int,double)
        """
    @staticmethod
    def __str__(arg1: Quaternion) -> str: 
        """
        __str__( (Quaternion)arg1) -> str :

            C++ signature :
                class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> > __str__(class Eigen::Quaternion<double,0>)
        """
    def _transformVector(self, vector: object) -> object: 
        """
        _transformVector( (Quaternion)self, (object)vector) -> object :
            Rotation of a vector by a quaternion.

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> _transformVector(class Eigen::Quaternion<double,0> {lvalue},class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @staticmethod
    def angularDistance(arg1: Quaternion, arg2: Quaternion) -> float: 
        """
        angularDistance( (Quaternion)arg1, (Quaternion)arg2) -> float :
            Returns the angle (in radian) between two rotations.

            C++ signature :
                double angularDistance(class Eigen::Quaternion<double,0> {lvalue},class Eigen::QuaternionBase<class Eigen::Quaternion<double,0> >)
        """
    @typing.overload
    def assign(self, aa: AngleAxis) -> Quaternion: 
        """
        assign( (Quaternion)self, (Quaternion)quat) -> Quaternion :
            Set *this from an quaternion quat and returns a reference to *this.

            C++ signature :
                class Eigen::Quaternion<double,0> {lvalue} assign(class Eigen::Quaternion<double,0> {lvalue},class Eigen::Quaternion<double,0>)

            C++ signature :
                class Eigen::Quaternion<double,0> {lvalue} assign(class Eigen::Quaternion<double,0> {lvalue},class Eigen::AngleAxis<double>)
        """
    @typing.overload
    def assign(self, quat: Quaternion) -> Quaternion: ...
    def coeffs(self) -> object: 
        """
        coeffs( (Quaternion)self) -> object :
            Returns a vector of the coefficients (x,y,z,w)

            C++ signature :
                class Eigen::Matrix<double,4,1,0,4,1> coeffs(class Eigen::Quaternion<double,0> {lvalue})
        """
    def conjugate(self) -> Quaternion: 
        """
        conjugate( (Quaternion)self) -> Quaternion :
            Returns the conjugated quaternion.
            The conjugate of a quaternion represents the opposite rotation.

            C++ signature :
                class Eigen::Quaternion<double,0> conjugate(class Eigen::Quaternion<double,0> {lvalue})
        """
    def dot(self, other: Quaternion) -> float: 
        """
        dot( (Quaternion)self, (Quaternion)other) -> float :
            Returns the dot product of *this with an other Quaternion.
            Geometrically speaking, the dot product of two unit quaternions corresponds to the cosine of half the angle between the two rotations.

            C++ signature :
                double dot(class Eigen::Quaternion<double,0> {lvalue},class Eigen::QuaternionBase<class Eigen::Quaternion<double,0> >)
        """
    def inverse(self) -> Quaternion: 
        """
        inverse( (Quaternion)self) -> Quaternion :
            Returns the quaternion describing the inverse rotation.

            C++ signature :
                class Eigen::Quaternion<double,0> inverse(class Eigen::Quaternion<double,0> {lvalue})
        """
    def isApprox(self, other_: Quaternion, prec: float) -> bool: 
        """
        isApprox( (Quaternion)self, (Quaternion)other [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to other, within the precision determined by prec.

            C++ signature :
                bool isApprox(class Eigen::Quaternion<double,0>,class Eigen::Quaternion<double,0> [,double])
        """
    def matrix(self) -> object: 
        """
        matrix( (Quaternion)self) -> object :
            Returns an equivalent 3x3 rotation matrix. Similar to toRotationMatrix.

            C++ signature :
                class Eigen::Matrix<double,3,3,0,3,3> matrix(class Eigen::Quaternion<double,0> {lvalue})
        """
    def norm(self) -> float: 
        """
        norm( (Quaternion)self) -> float :
            Returns the norm of the quaternion's coefficients.

            C++ signature :
                double norm(class Eigen::Quaternion<double,0> {lvalue})
        """
    def normalize(self) -> Quaternion: 
        """
        normalize( (Quaternion)self) -> Quaternion :
            Normalizes the quaternion *this.

            C++ signature :
                class Eigen::Quaternion<double,0> {lvalue} normalize(class Eigen::Quaternion<double,0> {lvalue})
        """
    def normalized(self) -> Quaternion: 
        """
        normalized( (Quaternion)self) -> Quaternion :
            Returns a normalized copy of *this.

            C++ signature :
                class Eigen::Quaternion<double,0> * __ptr64 normalized(class Eigen::Quaternion<double,0>)
        """
    def setFromTwoVectors(self, a: object, b: object) -> Quaternion: 
        """
        setFromTwoVectors( (Quaternion)self, (object)a, (object)b) -> Quaternion :
            Set *this to be the quaternion which transforms a into b through a rotation.

            C++ signature :
                class Eigen::Quaternion<double,0> {lvalue} setFromTwoVectors(class Eigen::Quaternion<double,0> {lvalue},class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<double,3,1,0,3,1>)
        """
    def setIdentity(self) -> Quaternion: 
        """
        setIdentity( (Quaternion)self) -> Quaternion :
            Set *this to the identity rotation.

            C++ signature :
                class Eigen::Quaternion<double,0> {lvalue} setIdentity(class Eigen::Quaternion<double,0> {lvalue})
        """
    def slerp(self, t: float, other: Quaternion) -> Quaternion: 
        """
        slerp( (Quaternion)self, (float)t, (Quaternion)other) -> Quaternion :
            Returns the spherical linear interpolation between the two quaternions *this and other at the parameter t in [0;1].

            C++ signature :
                class Eigen::Quaternion<double,0> slerp(class Eigen::Quaternion<double,0>,double,class Eigen::Quaternion<double,0>)
        """
    def squaredNorm(self) -> float: 
        """
        squaredNorm( (Quaternion)self) -> float :
            Returns the squared norm of the quaternion's coefficients.

            C++ signature :
                double squaredNorm(class Eigen::Quaternion<double,0> {lvalue})
        """
    @staticmethod
    def toRotationMatrix(arg1: Quaternion) -> object: 
        """
        toRotationMatrix( (Quaternion)arg1) -> object :
            Returns an equivalent 3x3 rotation matrix.

            C++ signature :
                class Eigen::Matrix<double,3,3,0,3,3> toRotationMatrix(class Eigen::Quaternion<double,0> {lvalue})
        """
    def vec(self) -> object: 
        """
        vec( (Quaternion)self) -> object :
            Returns a vector expression of the imaginary part (x,y,z).

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> vec(class Eigen::Quaternion<double,0>)
        """
    @property
    def w(self) -> None:
        """
        The w coefficient.

        :type: None
        """
    @property
    def x(self) -> None:
        """
        The x coefficient.

        :type: None
        """
    @property
    def y(self) -> None:
        """
        The y coefficient.

        :type: None
        """
    @property
    def z(self) -> None:
        """
        The z coefficient.

        :type: None
        """
    pass

class CollisionRequest(QueryRequest, Boost.Python.instance):
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)

            C++ signature :
                void __init__(struct _object * __ptr64,enum hpp::fcl::CollisionRequestFlag,unsigned __int64)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: CollisionRequestFlag, arg3: int) -> None: ...
    @property
    def break_distance(self) -> None:
        """
        :type: None
        """
    @property
    def distance_upper_bound(self) -> None:
        """
        :type: None
        """
    @property
    def enable_contact(self) -> None:
        """
        :type: None
        """
    @property
    def enable_distance_lower_bound(self) -> None:
        """
        :type: None
        """
    @property
    def num_max_contacts(self) -> None:
        """
        :type: None
        """
    @property
    def security_margin(self) -> None:
        """
        :type: None
        """
    pass

class CollisionResult(QueryResult, Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    def addContact(arg1: CollisionResult, arg2: Contact) -> None: 
        """
        addContact( (CollisionResult)arg1, (Contact)arg2) -> None :

            C++ signature :
                void addContact(struct hpp::fcl::CollisionResult {lvalue},struct hpp::fcl::Contact)
        """
    @staticmethod
    def clear(arg1: CollisionResult) -> None: 
        """
        clear( (CollisionResult)arg1) -> None :

            C++ signature :
                void clear(struct hpp::fcl::CollisionResult {lvalue})
        """
    @staticmethod
    def getContact(arg1: CollisionResult, arg2: int) -> Contact: 
        """
        getContact( (CollisionResult)arg1, (int)arg2) -> Contact :

            C++ signature :
                struct hpp::fcl::Contact getContact(struct hpp::fcl::CollisionResult {lvalue},unsigned __int64)
        """
    @staticmethod
    @typing.overload
    def getContacts(arg1: CollisionResult) -> StdVec_Contact: 
        """
        getContacts( (CollisionResult)arg1, (StdVec_Contact)arg2) -> None :

            C++ signature :
                void getContacts(struct hpp::fcl::CollisionResult {lvalue},class std::vector<struct hpp::fcl::Contact,class std::allocator<struct hpp::fcl::Contact> > {lvalue})

            C++ signature :
                class std::vector<struct hpp::fcl::Contact,class std::allocator<struct hpp::fcl::Contact> > getContacts(struct hpp::fcl::CollisionResult {lvalue})
        """
    @staticmethod
    @typing.overload
    def getContacts(arg1: CollisionResult, arg2: StdVec_Contact) -> None: ...
    @staticmethod
    def isCollision(arg1: CollisionResult) -> bool: 
        """
        isCollision( (CollisionResult)arg1) -> bool :

            C++ signature :
                bool isCollision(struct hpp::fcl::CollisionResult {lvalue})
        """
    @staticmethod
    def numContacts(arg1: CollisionResult) -> int: 
        """
        numContacts( (CollisionResult)arg1) -> int :

            C++ signature :
                unsigned __int64 numContacts(struct hpp::fcl::CollisionResult {lvalue})
        """
    @property
    def distance_lower_bound(self) -> None:
        """
        :type: None
        """
    pass

class Box(ShapeBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)

            C++ signature :
                void __init__(struct _object * __ptr64,double,double,double)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: float, arg3: float, arg4: float) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: object) -> None: ...
    @staticmethod
    def clone(arg1: Box) -> Box: 
        """
        clone( (Box)arg1) -> Box :

            C++ signature :
                class hpp::fcl::Box * __ptr64 clone(class hpp::fcl::Box {lvalue})
        """
    @property
    def halfSide(self) -> None:
        """
        :type: None
        """
    pass

class Sphere(ShapeBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object, arg2: float) -> None: 
        """
        __init__( (object)arg1, (float)arg2) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,double)
        """
    @staticmethod
    def clone(arg1: Sphere) -> Sphere: 
        """
        clone( (Sphere)arg1) -> Sphere :

            C++ signature :
                class hpp::fcl::Sphere * __ptr64 clone(class hpp::fcl::Sphere {lvalue})
        """
    @property
    def radius(self) -> None:
        """
        :type: None
        """
    pass

class StdVec_CollisionRequest(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_CollisionRequest, arg2: object) -> bool: 
        """
        __contains__( (StdVec_CollisionRequest)arg1, (object)arg2) -> bool :

            C++ signature :
                bool __contains__(class std::vector<struct hpp::fcl::CollisionRequest,class std::allocator<struct hpp::fcl::CollisionRequest> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __delitem__(arg1: StdVec_CollisionRequest, arg2: object) -> None: 
        """
        __delitem__( (StdVec_CollisionRequest)arg1, (object)arg2) -> None :

            C++ signature :
                void __delitem__(class std::vector<struct hpp::fcl::CollisionRequest,class std::allocator<struct hpp::fcl::CollisionRequest> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object :

            C++ signature :
                class boost::python::api::object __getitem__(struct boost::python::back_reference<class std::vector<struct hpp::fcl::CollisionRequest,class std::allocator<struct hpp::fcl::CollisionRequest> > & __ptr64>,struct _object * __ptr64)
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object :

            C++ signature :
                struct boost::python::objects::iterator_range<struct boost::python::return_internal_reference<1,struct boost::python::default_call_policies>,class std::_Vector_iterator<class std::_Vector_val<struct std::_Simple_types<struct hpp::fcl::CollisionRequest> > > > __iter__(struct boost::python::back_reference<class std::vector<struct hpp::fcl::CollisionRequest,class std::allocator<struct hpp::fcl::CollisionRequest> > & __ptr64>)
        """
    @staticmethod
    def __len__(arg1: StdVec_CollisionRequest) -> int: 
        """
        __len__( (StdVec_CollisionRequest)arg1) -> int :

            C++ signature :
                unsigned __int64 __len__(class std::vector<struct hpp::fcl::CollisionRequest,class std::allocator<struct hpp::fcl::CollisionRequest> > {lvalue})
        """
    @staticmethod
    def __setitem__(arg1: StdVec_CollisionRequest, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_CollisionRequest)arg1, (object)arg2, (object)arg3) -> None :

            C++ signature :
                void __setitem__(class std::vector<struct hpp::fcl::CollisionRequest,class std::allocator<struct hpp::fcl::CollisionRequest> > {lvalue},struct _object * __ptr64,struct _object * __ptr64)
        """
    @staticmethod
    def append(arg1: StdVec_CollisionRequest, arg2: object) -> None: 
        """
        append( (StdVec_CollisionRequest)arg1, (object)arg2) -> None :

            C++ signature :
                void append(class std::vector<struct hpp::fcl::CollisionRequest,class std::allocator<struct hpp::fcl::CollisionRequest> > {lvalue},class boost::python::api::object)
        """
    @staticmethod
    def extend(arg1: StdVec_CollisionRequest, arg2: object) -> None: 
        """
        extend( (StdVec_CollisionRequest)arg1, (object)arg2) -> None :

            C++ signature :
                void extend(class std::vector<struct hpp::fcl::CollisionRequest,class std::allocator<struct hpp::fcl::CollisionRequest> > {lvalue},class boost::python::api::object)
        """
    __instance_size__ = 40
    pass

class StdVec_CollisionResult(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_CollisionResult, arg2: object) -> bool: 
        """
        __contains__( (StdVec_CollisionResult)arg1, (object)arg2) -> bool :

            C++ signature :
                bool __contains__(class std::vector<struct hpp::fcl::CollisionResult,class std::allocator<struct hpp::fcl::CollisionResult> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __delitem__(arg1: StdVec_CollisionResult, arg2: object) -> None: 
        """
        __delitem__( (StdVec_CollisionResult)arg1, (object)arg2) -> None :

            C++ signature :
                void __delitem__(class std::vector<struct hpp::fcl::CollisionResult,class std::allocator<struct hpp::fcl::CollisionResult> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object :

            C++ signature :
                class boost::python::api::object __getitem__(struct boost::python::back_reference<class std::vector<struct hpp::fcl::CollisionResult,class std::allocator<struct hpp::fcl::CollisionResult> > & __ptr64>,struct _object * __ptr64)
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object :

            C++ signature :
                struct boost::python::objects::iterator_range<struct boost::python::return_internal_reference<1,struct boost::python::default_call_policies>,class std::_Vector_iterator<class std::_Vector_val<struct std::_Simple_types<struct hpp::fcl::CollisionResult> > > > __iter__(struct boost::python::back_reference<class std::vector<struct hpp::fcl::CollisionResult,class std::allocator<struct hpp::fcl::CollisionResult> > & __ptr64>)
        """
    @staticmethod
    def __len__(arg1: StdVec_CollisionResult) -> int: 
        """
        __len__( (StdVec_CollisionResult)arg1) -> int :

            C++ signature :
                unsigned __int64 __len__(class std::vector<struct hpp::fcl::CollisionResult,class std::allocator<struct hpp::fcl::CollisionResult> > {lvalue})
        """
    @staticmethod
    def __setitem__(arg1: StdVec_CollisionResult, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_CollisionResult)arg1, (object)arg2, (object)arg3) -> None :

            C++ signature :
                void __setitem__(class std::vector<struct hpp::fcl::CollisionResult,class std::allocator<struct hpp::fcl::CollisionResult> > {lvalue},struct _object * __ptr64,struct _object * __ptr64)
        """
    @staticmethod
    def append(arg1: StdVec_CollisionResult, arg2: object) -> None: 
        """
        append( (StdVec_CollisionResult)arg1, (object)arg2) -> None :

            C++ signature :
                void append(class std::vector<struct hpp::fcl::CollisionResult,class std::allocator<struct hpp::fcl::CollisionResult> > {lvalue},class boost::python::api::object)
        """
    @staticmethod
    def extend(arg1: StdVec_CollisionResult, arg2: object) -> None: 
        """
        extend( (StdVec_CollisionResult)arg1, (object)arg2) -> None :

            C++ signature :
                void extend(class std::vector<struct hpp::fcl::CollisionResult,class std::allocator<struct hpp::fcl::CollisionResult> > {lvalue},class boost::python::api::object)
        """
    __instance_size__ = 40
    pass

class StdVec_Contact(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Contact, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Contact)arg1, (object)arg2) -> bool :

            C++ signature :
                bool __contains__(class std::vector<struct hpp::fcl::Contact,class std::allocator<struct hpp::fcl::Contact> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Contact, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Contact)arg1, (object)arg2) -> None :

            C++ signature :
                void __delitem__(class std::vector<struct hpp::fcl::Contact,class std::allocator<struct hpp::fcl::Contact> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object :

            C++ signature :
                class boost::python::api::object __getitem__(struct boost::python::back_reference<class std::vector<struct hpp::fcl::Contact,class std::allocator<struct hpp::fcl::Contact> > & __ptr64>,struct _object * __ptr64)
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object :

            C++ signature :
                struct boost::python::objects::iterator_range<struct boost::python::return_internal_reference<1,struct boost::python::default_call_policies>,class std::_Vector_iterator<class std::_Vector_val<struct std::_Simple_types<struct hpp::fcl::Contact> > > > __iter__(struct boost::python::back_reference<class std::vector<struct hpp::fcl::Contact,class std::allocator<struct hpp::fcl::Contact> > & __ptr64>)
        """
    @staticmethod
    def __len__(arg1: StdVec_Contact) -> int: 
        """
        __len__( (StdVec_Contact)arg1) -> int :

            C++ signature :
                unsigned __int64 __len__(class std::vector<struct hpp::fcl::Contact,class std::allocator<struct hpp::fcl::Contact> > {lvalue})
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Contact, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Contact)arg1, (object)arg2, (object)arg3) -> None :

            C++ signature :
                void __setitem__(class std::vector<struct hpp::fcl::Contact,class std::allocator<struct hpp::fcl::Contact> > {lvalue},struct _object * __ptr64,struct _object * __ptr64)
        """
    @staticmethod
    def append(arg1: StdVec_Contact, arg2: object) -> None: 
        """
        append( (StdVec_Contact)arg1, (object)arg2) -> None :

            C++ signature :
                void append(class std::vector<struct hpp::fcl::Contact,class std::allocator<struct hpp::fcl::Contact> > {lvalue},class boost::python::api::object)
        """
    @staticmethod
    def extend(arg1: StdVec_Contact, arg2: object) -> None: 
        """
        extend( (StdVec_Contact)arg1, (object)arg2) -> None :

            C++ signature :
                void extend(class std::vector<struct hpp::fcl::Contact,class std::allocator<struct hpp::fcl::Contact> > {lvalue},class boost::python::api::object)
        """
    __instance_size__ = 40
    pass

class StdVec_DistanceRequest(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_DistanceRequest, arg2: object) -> bool: 
        """
        __contains__( (StdVec_DistanceRequest)arg1, (object)arg2) -> bool :

            C++ signature :
                bool __contains__(class std::vector<struct hpp::fcl::DistanceRequest,class std::allocator<struct hpp::fcl::DistanceRequest> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __delitem__(arg1: StdVec_DistanceRequest, arg2: object) -> None: 
        """
        __delitem__( (StdVec_DistanceRequest)arg1, (object)arg2) -> None :

            C++ signature :
                void __delitem__(class std::vector<struct hpp::fcl::DistanceRequest,class std::allocator<struct hpp::fcl::DistanceRequest> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object :

            C++ signature :
                class boost::python::api::object __getitem__(struct boost::python::back_reference<class std::vector<struct hpp::fcl::DistanceRequest,class std::allocator<struct hpp::fcl::DistanceRequest> > & __ptr64>,struct _object * __ptr64)
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object :

            C++ signature :
                struct boost::python::objects::iterator_range<struct boost::python::return_internal_reference<1,struct boost::python::default_call_policies>,class std::_Vector_iterator<class std::_Vector_val<struct std::_Simple_types<struct hpp::fcl::DistanceRequest> > > > __iter__(struct boost::python::back_reference<class std::vector<struct hpp::fcl::DistanceRequest,class std::allocator<struct hpp::fcl::DistanceRequest> > & __ptr64>)
        """
    @staticmethod
    def __len__(arg1: StdVec_DistanceRequest) -> int: 
        """
        __len__( (StdVec_DistanceRequest)arg1) -> int :

            C++ signature :
                unsigned __int64 __len__(class std::vector<struct hpp::fcl::DistanceRequest,class std::allocator<struct hpp::fcl::DistanceRequest> > {lvalue})
        """
    @staticmethod
    def __setitem__(arg1: StdVec_DistanceRequest, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_DistanceRequest)arg1, (object)arg2, (object)arg3) -> None :

            C++ signature :
                void __setitem__(class std::vector<struct hpp::fcl::DistanceRequest,class std::allocator<struct hpp::fcl::DistanceRequest> > {lvalue},struct _object * __ptr64,struct _object * __ptr64)
        """
    @staticmethod
    def append(arg1: StdVec_DistanceRequest, arg2: object) -> None: 
        """
        append( (StdVec_DistanceRequest)arg1, (object)arg2) -> None :

            C++ signature :
                void append(class std::vector<struct hpp::fcl::DistanceRequest,class std::allocator<struct hpp::fcl::DistanceRequest> > {lvalue},class boost::python::api::object)
        """
    @staticmethod
    def extend(arg1: StdVec_DistanceRequest, arg2: object) -> None: 
        """
        extend( (StdVec_DistanceRequest)arg1, (object)arg2) -> None :

            C++ signature :
                void extend(class std::vector<struct hpp::fcl::DistanceRequest,class std::allocator<struct hpp::fcl::DistanceRequest> > {lvalue},class boost::python::api::object)
        """
    __instance_size__ = 40
    pass

class StdVec_DistanceResult(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_DistanceResult, arg2: object) -> bool: 
        """
        __contains__( (StdVec_DistanceResult)arg1, (object)arg2) -> bool :

            C++ signature :
                bool __contains__(class std::vector<struct hpp::fcl::DistanceResult,class std::allocator<struct hpp::fcl::DistanceResult> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __delitem__(arg1: StdVec_DistanceResult, arg2: object) -> None: 
        """
        __delitem__( (StdVec_DistanceResult)arg1, (object)arg2) -> None :

            C++ signature :
                void __delitem__(class std::vector<struct hpp::fcl::DistanceResult,class std::allocator<struct hpp::fcl::DistanceResult> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object :

            C++ signature :
                class boost::python::api::object __getitem__(struct boost::python::back_reference<class std::vector<struct hpp::fcl::DistanceResult,class std::allocator<struct hpp::fcl::DistanceResult> > & __ptr64>,struct _object * __ptr64)
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object :

            C++ signature :
                struct boost::python::objects::iterator_range<struct boost::python::return_internal_reference<1,struct boost::python::default_call_policies>,class std::_Vector_iterator<class std::_Vector_val<struct std::_Simple_types<struct hpp::fcl::DistanceResult> > > > __iter__(struct boost::python::back_reference<class std::vector<struct hpp::fcl::DistanceResult,class std::allocator<struct hpp::fcl::DistanceResult> > & __ptr64>)
        """
    @staticmethod
    def __len__(arg1: StdVec_DistanceResult) -> int: 
        """
        __len__( (StdVec_DistanceResult)arg1) -> int :

            C++ signature :
                unsigned __int64 __len__(class std::vector<struct hpp::fcl::DistanceResult,class std::allocator<struct hpp::fcl::DistanceResult> > {lvalue})
        """
    @staticmethod
    def __setitem__(arg1: StdVec_DistanceResult, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_DistanceResult)arg1, (object)arg2, (object)arg3) -> None :

            C++ signature :
                void __setitem__(class std::vector<struct hpp::fcl::DistanceResult,class std::allocator<struct hpp::fcl::DistanceResult> > {lvalue},struct _object * __ptr64,struct _object * __ptr64)
        """
    @staticmethod
    def append(arg1: StdVec_DistanceResult, arg2: object) -> None: 
        """
        append( (StdVec_DistanceResult)arg1, (object)arg2) -> None :

            C++ signature :
                void append(class std::vector<struct hpp::fcl::DistanceResult,class std::allocator<struct hpp::fcl::DistanceResult> > {lvalue},class boost::python::api::object)
        """
    @staticmethod
    def extend(arg1: StdVec_DistanceResult, arg2: object) -> None: 
        """
        extend( (StdVec_DistanceResult)arg1, (object)arg2) -> None :

            C++ signature :
                void extend(class std::vector<struct hpp::fcl::DistanceResult,class std::allocator<struct hpp::fcl::DistanceResult> > {lvalue},class boost::python::api::object)
        """
    __instance_size__ = 40
    pass

class StdVec_Triangle(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Triangle, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Triangle)arg1, (object)arg2) -> bool :

            C++ signature :
                bool __contains__(class std::vector<class hpp::fcl::Triangle,class std::allocator<class hpp::fcl::Triangle> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Triangle, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Triangle)arg1, (object)arg2) -> None :

            C++ signature :
                void __delitem__(class std::vector<class hpp::fcl::Triangle,class std::allocator<class hpp::fcl::Triangle> > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object :

            C++ signature :
                class boost::python::api::object __getitem__(struct boost::python::back_reference<class std::vector<class hpp::fcl::Triangle,class std::allocator<class hpp::fcl::Triangle> > & __ptr64>,struct _object * __ptr64)
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object :

            C++ signature :
                struct boost::python::objects::iterator_range<struct boost::python::return_internal_reference<1,struct boost::python::default_call_policies>,class std::_Vector_iterator<class std::_Vector_val<struct std::_Simple_types<class hpp::fcl::Triangle> > > > __iter__(struct boost::python::back_reference<class std::vector<class hpp::fcl::Triangle,class std::allocator<class hpp::fcl::Triangle> > & __ptr64>)
        """
    @staticmethod
    def __len__(arg1: StdVec_Triangle) -> int: 
        """
        __len__( (StdVec_Triangle)arg1) -> int :

            C++ signature :
                unsigned __int64 __len__(class std::vector<class hpp::fcl::Triangle,class std::allocator<class hpp::fcl::Triangle> > {lvalue})
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Triangle, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Triangle)arg1, (object)arg2, (object)arg3) -> None :

            C++ signature :
                void __setitem__(class std::vector<class hpp::fcl::Triangle,class std::allocator<class hpp::fcl::Triangle> > {lvalue},struct _object * __ptr64,struct _object * __ptr64)
        """
    @staticmethod
    def append(arg1: StdVec_Triangle, arg2: object) -> None: 
        """
        append( (StdVec_Triangle)arg1, (object)arg2) -> None :

            C++ signature :
                void append(class std::vector<class hpp::fcl::Triangle,class std::allocator<class hpp::fcl::Triangle> > {lvalue},class boost::python::api::object)
        """
    @staticmethod
    def extend(arg1: StdVec_Triangle, arg2: object) -> None: 
        """
        extend( (StdVec_Triangle)arg1, (object)arg2) -> None :

            C++ signature :
                void extend(class std::vector<class hpp::fcl::Triangle,class std::allocator<class hpp::fcl::Triangle> > {lvalue},class boost::python::api::object)
        """
    __instance_size__ = 40
    pass

class StdVec_Vec3f(Boost.Python.instance):
    @staticmethod
    def __contains__(arg1: StdVec_Vec3f, arg2: object) -> bool: 
        """
        __contains__( (StdVec_Vec3f)arg1, (object)arg2) -> bool :

            C++ signature :
                bool __contains__(class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __delitem__(arg1: StdVec_Vec3f, arg2: object) -> None: 
        """
        __delitem__( (StdVec_Vec3f)arg1, (object)arg2) -> None :

            C++ signature :
                void __delitem__(class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > > {lvalue},struct _object * __ptr64)
        """
    @staticmethod
    def __getitem__(arg1: object, arg2: object) -> object: 
        """
        __getitem__( (object)arg1, (object)arg2) -> object :

            C++ signature :
                class boost::python::api::object __getitem__(struct boost::python::back_reference<class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > > & __ptr64>,struct _object * __ptr64)
        """
    @staticmethod
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)
        """
    @staticmethod
    def __iter__(arg1: object) -> object: 
        """
        __iter__( (object)arg1) -> object :

            C++ signature :
                struct boost::python::objects::iterator_range<struct boost::python::return_internal_reference<1,struct boost::python::default_call_policies>,class std::_Vector_iterator<class std::_Vector_val<struct std::_Simple_types<class Eigen::Matrix<double,3,1,0,3,1> > > > > __iter__(struct boost::python::back_reference<class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > > & __ptr64>)
        """
    @staticmethod
    def __len__(arg1: StdVec_Vec3f) -> int: 
        """
        __len__( (StdVec_Vec3f)arg1) -> int :

            C++ signature :
                unsigned __int64 __len__(class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > > {lvalue})
        """
    @staticmethod
    def __setitem__(arg1: StdVec_Vec3f, arg2: object, arg3: object) -> None: 
        """
        __setitem__( (StdVec_Vec3f)arg1, (object)arg2, (object)arg3) -> None :

            C++ signature :
                void __setitem__(class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > > {lvalue},struct _object * __ptr64,struct _object * __ptr64)
        """
    @staticmethod
    def append(arg1: StdVec_Vec3f, arg2: object) -> None: 
        """
        append( (StdVec_Vec3f)arg1, (object)arg2) -> None :

            C++ signature :
                void append(class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > > {lvalue},class boost::python::api::object)
        """
    @staticmethod
    def extend(arg1: StdVec_Vec3f, arg2: object) -> None: 
        """
        extend( (StdVec_Vec3f)arg1, (object)arg2) -> None :

            C++ signature :
                void extend(class std::vector<class Eigen::Matrix<double,3,1,0,3,1>,class std::allocator<class Eigen::Matrix<double,3,1,0,3,1> > > {lvalue},class boost::python::api::object)
        """
    __instance_size__ = 40
    pass

class Transform3f(Boost.Python.instance):
    @staticmethod
    def __eq__(arg1: Transform3f, arg2: Transform3f) -> object: 
        """
        __eq__( (Transform3f)arg1, (Transform3f)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __eq__(class hpp::fcl::Transform3f {lvalue},class hpp::fcl::Transform3f)
        """
    @staticmethod
    def __imul__(arg1: object, arg2: Transform3f) -> object: 
        """
        __imul__( (object)arg1, (Transform3f)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __imul__(struct boost::python::back_reference<class hpp::fcl::Transform3f & __ptr64>,class hpp::fcl::Transform3f)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::MatrixBase<class Eigen::Matrix<double,3,3,0,3,3> >,class Eigen::MatrixBase<class Eigen::Matrix<double,3,1,0,3,1> >)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Quaternion<double,0>,class Eigen::MatrixBase<class Eigen::Matrix<double,3,1,0,3,1> >)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Matrix<double,3,3,0,3,3>)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Quaternion<double,0>)

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Matrix<double,3,1,0,3,1>)

            C++ signature :
                void __init__(struct _object * __ptr64,class hpp::fcl::Transform3f)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: Quaternion) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: Quaternion, arg3: object) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: Transform3f) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: object) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: object, arg3: object) -> None: ...
    @staticmethod
    def __mul__(arg1: Transform3f, arg2: Transform3f) -> object: 
        """
        __mul__( (Transform3f)arg1, (Transform3f)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __mul__(class hpp::fcl::Transform3f {lvalue},class hpp::fcl::Transform3f)
        """
    @staticmethod
    def __ne__(arg1: Transform3f, arg2: Transform3f) -> object: 
        """
        __ne__( (Transform3f)arg1, (Transform3f)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __ne__(class hpp::fcl::Transform3f {lvalue},class hpp::fcl::Transform3f)
        """
    @staticmethod
    def getQuatRotation(arg1: Transform3f) -> Quaternion: 
        """
        getQuatRotation( (Transform3f)arg1) -> Quaternion :

            C++ signature :
                class Eigen::Quaternion<double,0> getQuatRotation(class hpp::fcl::Transform3f {lvalue})
        """
    @staticmethod
    def getRotation(arg1: Transform3f) -> object: 
        """
        getRotation( (Transform3f)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,3,0,3,3> getRotation(class hpp::fcl::Transform3f {lvalue})
        """
    @staticmethod
    def getTranslation(arg1: Transform3f) -> object: 
        """
        getTranslation( (Transform3f)arg1) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> getTranslation(class hpp::fcl::Transform3f {lvalue})
        """
    @staticmethod
    def inverse(arg1: Transform3f) -> Transform3f: 
        """
        inverse( (Transform3f)arg1) -> Transform3f :

            C++ signature :
                class hpp::fcl::Transform3f inverse(class hpp::fcl::Transform3f {lvalue})
        """
    @staticmethod
    def inverseInPlace(arg1: Transform3f) -> Transform3f: 
        """
        inverseInPlace( (Transform3f)arg1) -> Transform3f :

            C++ signature :
                class hpp::fcl::Transform3f {lvalue} inverseInPlace(class hpp::fcl::Transform3f {lvalue})
        """
    @staticmethod
    def inverseTimes(arg1: Transform3f, arg2: Transform3f) -> Transform3f: 
        """
        inverseTimes( (Transform3f)arg1, (Transform3f)arg2) -> Transform3f :

            C++ signature :
                class hpp::fcl::Transform3f inverseTimes(class hpp::fcl::Transform3f {lvalue},class hpp::fcl::Transform3f)
        """
    @staticmethod
    def isIdentity(arg1: Transform3f) -> bool: 
        """
        isIdentity( (Transform3f)arg1) -> bool :

            C++ signature :
                bool isIdentity(class hpp::fcl::Transform3f {lvalue})
        """
    @staticmethod
    def setIdentity(arg1: Transform3f) -> None: 
        """
        setIdentity( (Transform3f)arg1) -> None :

            C++ signature :
                void setIdentity(class hpp::fcl::Transform3f {lvalue})
        """
    @staticmethod
    def setQuatRotation(arg1: Transform3f, arg2: Quaternion) -> None: 
        """
        setQuatRotation( (Transform3f)arg1, (Quaternion)arg2) -> None :

            C++ signature :
                void setQuatRotation(class hpp::fcl::Transform3f {lvalue},class Eigen::Quaternion<double,0>)
        """
    @staticmethod
    def setRotation(arg1: Transform3f, arg2: object) -> None: 
        """
        setRotation( (Transform3f)arg1, (object)arg2) -> None :

            C++ signature :
                void setRotation(class hpp::fcl::Transform3f {lvalue},class Eigen::MatrixBase<class Eigen::Matrix<double,3,3,0,3,3> >)
        """
    @staticmethod
    @typing.overload
    def setTransform(arg1: Transform3f, arg2: Quaternion, arg3: object) -> None: 
        """
        setTransform( (Transform3f)arg1, (object)arg2, (object)arg3) -> None :

            C++ signature :
                void setTransform(class hpp::fcl::Transform3f {lvalue},class Eigen::MatrixBase<class Eigen::Matrix<double,3,3,0,3,3> >,class Eigen::MatrixBase<class Eigen::Matrix<double,3,1,0,3,1> >)

            C++ signature :
                void setTransform(class hpp::fcl::Transform3f {lvalue},class Eigen::Quaternion<double,0>,class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @staticmethod
    @typing.overload
    def setTransform(arg1: Transform3f, arg2: object, arg3: object) -> None: ...
    @staticmethod
    def setTranslation(arg1: Transform3f, arg2: object) -> None: 
        """
        setTranslation( (Transform3f)arg1, (object)arg2) -> None :

            C++ signature :
                void setTranslation(class hpp::fcl::Transform3f {lvalue},class Eigen::MatrixBase<class Eigen::Matrix<double,3,1,0,3,1> >)
        """
    @staticmethod
    def transform(arg1: Transform3f, arg2: object) -> object: 
        """
        transform( (Transform3f)arg1, (object)arg2) -> object :

            C++ signature :
                class Eigen::Matrix<double,3,1,0,3,1> transform(class hpp::fcl::Transform3f {lvalue},class Eigen::MatrixBase<class Eigen::Matrix<double,3,1,0,3,1> >)
        """
    pass

class Triangle(Boost.Python.instance):
    @staticmethod
    def __eq__(arg1: Triangle, arg2: Triangle) -> object: 
        """
        __eq__( (Triangle)arg1, (Triangle)arg2) -> object :

            C++ signature :
                struct _object * __ptr64 __eq__(class hpp::fcl::Triangle {lvalue},class hpp::fcl::Triangle)
        """
    @staticmethod
    def __getitem__(arg1: Triangle, arg2: int) -> int: 
        """
        __getitem__( (Triangle)arg1, (int)arg2) -> int :

            C++ signature :
                unsigned __int64 __getitem__(class hpp::fcl::Triangle,int)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object) -> None: 
        """
        __init__( (object)arg1) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64)

            C++ signature :
                void __init__(struct _object * __ptr64,unsigned __int64,unsigned __int64,unsigned __int64)
        """
    @staticmethod
    @typing.overload
    def __init__(arg1: object, arg2: int, arg3: int, arg4: int) -> None: ...
    @staticmethod
    def __setitem__(arg1: Triangle, arg2: int, arg3: int) -> None: 
        """
        __setitem__( (Triangle)arg1, (int)arg2, (int)arg3) -> None :

            C++ signature :
                void __setitem__(class hpp::fcl::Triangle {lvalue},int,unsigned __int64)
        """
    @staticmethod
    def set(arg1: Triangle, arg2: int, arg3: int, arg4: int) -> None: 
        """
        set( (Triangle)arg1, (int)arg2, (int)arg3, (int)arg4) -> None :

            C++ signature :
                void set(class hpp::fcl::Triangle {lvalue},unsigned __int64,unsigned __int64,unsigned __int64)
        """
    @staticmethod
    def size() -> int: 
        """
        size() -> int :

            C++ signature :
                int size()
        """
    pass

class TriangleP(ShapeBase, CollisionGeometry, Boost.Python.instance):
    @staticmethod
    def __init__(arg1: object, arg2: object, arg3: object, arg4: object) -> None: 
        """
        __init__( (object)arg1, (object)arg2, (object)arg3, (object)arg4) -> None :

            C++ signature :
                void __init__(struct _object * __ptr64,class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<double,3,1,0,3,1>,class Eigen::Matrix<double,3,1,0,3,1>)
        """
    @staticmethod
    def clone(arg1: TriangleP) -> TriangleP: 
        """
        clone( (TriangleP)arg1) -> TriangleP :

            C++ signature :
                class hpp::fcl::TriangleP * __ptr64 clone(class hpp::fcl::TriangleP {lvalue})
        """
    @property
    def a(self) -> None:
        """
        :type: None
        """
    @property
    def b(self) -> None:
        """
        :type: None
        """
    @property
    def c(self) -> None:
        """
        :type: None
        """
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
import Boost.Python
import hppfcl.hppfcl

