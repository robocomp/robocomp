/*
 * Copyright 2016 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

 #ifndef INNERMODELDISPLAY_H
 #define INNERMODELDISPLAY_H

 #include <innermodel/innermodelnode.h>
 #include <osg/TriangleFunctor>
 #include <osg/io_utils>
 #include <osg/Geode>
 #include <osg/MatrixTransform>

 class InnerModelDisplay : public InnerModelNode
 {
 	public:
    InnerModelDisplay(QString id_, uint32_t port_, QString texture_, float width_, float height_,float depth_, int repeat_, float nx_, float ny_, float nz_, float px_, float py_, float pz_, bool collidable_, InnerModelNode *parent_=NULL);
    void updateTexture(QString texture_);
    void print(bool verbose);
    void save(QTextStream &out, int tabs);
    void setUpdatePointers(float *nx_, float *ny_, float *nz_, float *px_, float *py_, float *pz_);
    void update();
    void update(float nx_, float ny_, float nz_, float px_, float py_, float pz_);
    virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

    QVec normal, point;
    QString texture;
    uint32_t port;
    int repeat;
    float width, height, depth;
    float *nx, *ny, *nz;
    float *px, *py, *pz;
    bool collidable;

};

#endif // INNERMODELDISPLAY_H
