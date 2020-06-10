#ifndef INNER_API
#define INNER_API

#include <qmat/QMatAll>
#include "topics/DSRGraphPubSubTypes.h"

namespace CRDT
{
    class CRDTGraph;
    class VEdge;

    class InnerAPI
    {
        using VEdgePtr = std::shared_ptr<VEdge>;
        using Lists = std::tuple<std::list<RMat::RTMat>, std::list<RMat::RTMat> >; 
        public:
            InnerAPI(CRDTGraph *G_);
            
            /////////////////////////////////////////////////
            /// Kinematic transformation methods
            ////////////////////////////////////////////////
            std::optional<QVec> transform(const QString & destId, const QVec &origVec, const QString & origId);
            std::optional<QVec> transform( const QString &destId, const QString & origId);
            std::optional<QVec> transformS( const std::string &destId, const QVec &origVec, const std::string & origId);
            std::optional<QVec> transformS( const std::string &destId, const std::string &origId);
            std::optional<QVec> transform6D(const QString &destId, const QVec &origVec, const QString & origId); 
            std::optional<QVec> transform6D(const QString &destId, const QString & origId);
            std::optional<QVec> transformS6D(const std::string &destId, const std::string & origId);
            std::optional<QVec> transformS6D(const std::string &destId, const QVec &origVec, const std::string & origId);
                
            ////////////////////////////////////////////////
            /// Transformation matrix retrieval methods
            ////////////////////////////////////////////////
            std::optional<RTMat> getTransformationMatrix(const QString &destId, const QString &origId);
            std::optional<RTMat> getTransformationMatrixS(const std::string &destId, const std::string &origId);
            QMat getRotationMatrixTo(const QString &to, const QString &from);
            QVec getTranslationVectorTo(const QString &to, const QString &from);
            QVec rotationAngles(const QString & destId, const QString & origId);
        
            ////////////////////////////////////////////////
            /// Update transform methods
            ////////////////////////////////////////////////
            void updateTransformValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId="");
            void updateTransformValues(QString transformId, QVec v, QString parentId="");
            void updateTransformValuesS(std::string transformId, float tx, float ty, float tz, float rx, float ry, float rz, std::string parentId="");
            void updateTransformValuesS(std::string transformId, QVec v, std::string parentId="");
            void updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId="");
            void updateRotationValues(QString transformId, float rx, float ry, float rz,QString parentId="");

        private:
            CRDT::CRDTGraph *G;
            std::optional<InnerAPI::Lists> setLists(const std::string &origId, const std::string &destId);
        
    };
};

#endif