// http://www.boost.org/doc/libs/1_53_0/libs/python/doc/tutorial/doc/html/python/exposing.html

#include <boost/python.hpp>

#include <qmat/qvec.h>
#include <qmat/qmat.h>

#include <innermodel/innermodel.h>
#include <innermodel/innermodeldraw.h>

using namespace RMat;
using namespace boost::python;

// using namespace InnerModel;

/*
BOOST_PYTHON_MODULE(librobocomp_innermodel)
{

	class_<InnerModel>("InnerModel", init<>())
	 .def(init<std::string>())
	 .def(init<InnerModel &>())
	 .def("updateTransformValues", &InnerModel::updateTransformValuesS,
	     (
	         arg("transformId"),
	         arg("tx"), arg("ty"), arg("tz"),
	         arg("rx"), arg("ry"), arg("rz"),
	         arg("parentId")=""
	     )
	 )
	 .def("transform", &InnerModel::transformS)
	 .def("getTransformationMatrix", &InnerModel::getTransformationMatrixS)
	 .def("getParentIdentifier", &InnerModel::getParentIdentifierS)
    ;
	*/

/*
	class_<InnerModelDraw>("OsgView", init<>());
	
	class_<InnerModelDraw>("InnerModelDraw", init<>())
	 .def("createViewer", &InnerModelDraw::createViewer)
	 .def("addPlane_ignoreExisting", &InnerModelDraw::addPlane_ignoreExisting,
	     (
	        arg("innerViewer"),
	        arg("id"),
	        arg("root"),
	        arg("pose"),
	        arg("normal"),
	        arg("color"),
	        arg("size")
	     )
	 )
	;*/


// 	class_<InnerModelTransform>("InnerModelTransform", init<>())
// 	 .def("getParentId", &InnerModel::getParentId)
// 	 .def("getParentTransform", &InnerModelTransform::getParentTransform)
//     ;



/*
    	void updateTransformValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId="");
	void updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId="");
	void updateRotationValues(QString transformId, float rx, float ry, float rz,QString parentId="");
	void updateJointValue(QString jointId, float angle);
	void updatePrismaticJointPosition(QString jointId, float position);
	void updatePlaneValues(QString planeId, float nx, float ny, float nz, float px, float py, float pz);

	InnerModelTransform *newTransform(QString id, QString engine, InnerModelNode *parent, float tx=0, float ty=0, float tz=0, float rx=0, float ry=0, float rz=0, float mass=0);
	InnerModelJoint *newJoint(QString id, InnerModelTransform* parent, float lx = 0, float ly = 0, float lz = 0, float hx = 0, float hy = 0, float hz = 0,  float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, float min=-INFINITY, float max=INFINITY, uint32_t port = 0, std::string axis = "z", float home=0);
	InnerModelTouchSensor *newTouchSensor(QString id, InnerModelTransform* parent, QString type, float nx = 0, float ny = 0, float nz = 0, float min=0, float max=INFINITY, uint32_t port=0);
	InnerModelPrismaticJoint *newPrismaticJoint(QString id, InnerModelTransform* parent, float min=-INFINITY, float max=INFINITY, float value=0, float offset=0, uint32_t port = 0, std::string axis = "z", float home=0);
	InnerModelDifferentialRobot *newDifferentialRobot(QString id, InnerModelTransform* parent, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, uint32_t port = 0, float noise=0.);
	InnerModelOmniRobot *newOmniRobot(QString id, InnerModelTransform* parent, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, uint32_t port = 0, float noise=0.);
	InnerModelCamera *newCamera(QString id, InnerModelNode *parent, float width, float height, float focal);
	InnerModelRGBD *newRGBD(QString id, InnerModelNode *parent, float width, float height, float focal, float noise, uint32_t port = 0, QString ifconfig="");
	InnerModelIMU *newIMU(QString id, InnerModelNode *parent, uint32_t port = 0);
	InnerModelLaser *newLaser(QString id, InnerModelNode *parent, uint32_t port = 0, uint32_t min=0, uint32_t max=30000, float angle = M_PIl, uint32_t measures = 360, QString ifconfig="");
	InnerModelPlane *newPlane(QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx=0, float ny=0, float nz=0, float px=0, float py=0, float pz=0, bool collidable=0);
	InnerModelMesh *newMesh(QString id, InnerModelNode *parent, QString path, float scale, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable=0);
	InnerModelMesh *newMesh(QString id, InnerModelNode *parent, QString path, float scalex, float scaley, float scalez, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable=0);
	InnerModelPointCloud *newPointCloud(QString id, InnerModelNode *parent);

	QVec transform(const QString & destId, const QVec &origVec, const QString & origId);
	QVec transform(const QString & destId, const QString & origId) { return transform(destId, QVec::vec3(0,0,0), origId); }
	QVec rotationAngles(const QString & destId, const QString & origId);
	QVec project(QString reference, QVec origVec, QString cameraId);
	QVec backProject(const QString &cameraId, const QVec &coord) ;//const;
	void imageCoordToAngles(const QString &cameraId, QVec coord, float &pan, float &tilt, const QString & anglesRefS);
	QVec anglesToImageCoord(const QString &cameraId, float pan, float tilt, const QString & anglesRefS);
	QVec imageCoordPlusDepthTo(QString cameraId, QVec coord, float depth, QString to);
	QVec projectFromCameraToPlane(const QString &to, const QVec &coord, const QString &cameraId, const QVec &vPlane, const float &dist);
	QVec horizonLine(QString planeId, QString cameraId, float heightOffset=0.);

	/// Matrix transformation retrieval methods
	RTMat getTransformationMatrix(const QString &destId, const QString &origId);
	QMat getRotationMatrixTo(const QString &to, const QString &from);
	QVec getTranslationVectorTo(const QString &to, const QString &from);
	QMat getHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
	QMat getAffineHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
	QMat getPlaneProjectionMatrix(QString virtualCamera, QString plane, QString sourceCamera);
*/


// }


