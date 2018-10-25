	///////////////////////////////////
	/// Kinematic transformation methods
	////////////////////////////////////
  
	QVec transform(  const QString & destId, const QVec &origVec, const QString & origId);
	QVec transform(  const QString &destId, const QString & origId) { return transform(destId, QVec::vec3(0,0,0), origId); };
	QVec transformS( const std::string &destId, const QVec &origVec, const std::string & origId);
	QVec transformS( const std::string &destId, const std::string &origId) { return transform(QString::fromStdString(destId), QVec::vec3(0,0,0), QString::fromStdString(origId)); }

	QVec transform6D(const QString &destId, const QVec &origVec, const QString & origId) { Q_ASSERT(origVec.size() == 6); return transform(destId, origVec, origId); }
	QVec transform6D(const QString &destId, const QString & origId) { return transform(destId, QVec::vec6(0,0,0,0,0,0), origId); }
	QVec transformS6D(const std::string &destId, const std::string & origId) 
		{ return transform(QString::fromStdString(destId), QVec::vec6(0,0,0,0,0,0), QString::fromStdString(origId)); }
	QVec transformS6D(const std::string &destId, const QVec &origVec, const std::string & origId) 
		{ return transform(QString::fromStdString(destId), origVec, QString::fromStdString(origId)); }
	
	////////////////////////////////////////////
	/// Transformation matrix retrieval methods
	///////////////////////////////////////////
	RTMat getTransformationMatrix(const QString &destId, const QString &origId);
	RTMat getTransformationMatrixS(const std::string &destId, const std::string &origId);
	QMat getRotationMatrixTo(const QString &to, const QString &from);
	QVec getTranslationVectorTo(const QString &to, const QString &from);
	QVec rotationAngles(const QString & destId, const QString & origId);

	/////////////////////////////////////////////
	/// Graoh editing methods
	/////////////////////////////////////////////
	QList<QString> getIDKeys() {return hash.keys(); }
	InnerModelNode *getNode(const QString & id) const { /*QMutexLocker ml(mutex); */if (hash.contains(id)) return hash[id]; else return NULL;}
	template <class N> N* getNode(const QString &id) const
	{
		N* r = dynamic_cast<N *>(getNode(id));
		if (not r)
		{
			QString error;
			if (not hash[id])
				error.sprintf("No such joint %s", id.toStdString().c_str());
			else
				error.sprintf("%s doesn't seem to be a joint", id.toStdString().c_str());
			throw error;
		}
		return r;
	}

	void removeSubTree(InnerModelNode *item, QStringList *l);
	void removeNode(const QString & id);
	void moveSubTree(InnerModelNode *nodeSrc, InnerModelNode *nodeDst);
	void getSubTree(InnerModelNode *node, QStringList *l);
	void getSubTree(InnerModelNode *node, QList<InnerModelNode *> *l);
	void computeLevels(InnerModelNode *node);
	InnerModelNode *getRoot() { return root; }
	QString getParentIdentifier(QString id);
	std::string getParentIdentifierS(std::string id);

	/////////////////////
	/// Set debug level
	/////////////////////
	int debugLevel(int level=-1) { static int debug_level=0; if (level>-1) debug_level=level; return debug_level; }

	////////////////////////////
	// FCL related
	////////////////////////////
	bool collidable(const QString &a);
	bool collide(const QString &a, const QString &b);
	float distance(const QString &a, const QString &b);

#if FCL_SUPPORT==1
	bool collide(const QString &a, const fcl::CollisionObject *obj);
#endif

	/**
		* @brief Computes the jacobian of a list of joints at a given configuration point given by motores
		*
		* @param listaJoints list of names of joints in InnerModel that conform the open kinematic chain
		* @param motores value of motro joints where the jacobian will be evaluated
		* @param endEffector name of end effector of the kin. chain. It can be an element further away than the last in listaJoint.
		* @return RMat::QMat Jacobian as MxN matrix of evaluated partial derivatives. M=joints, N=6 (pose cartesian coordinates of the endEffector) (CHECK ORDER)
		*/
	QMat jacobian(QStringList &listaJoints, const QVec &motores, const QString &endEffector);
	QMat jacobianS(std::vector<std::string> &listaJoints, const QVec &motores, const std::string &endEffector)
	{
		QStringList listaJointQ/* = QStringList::fromStdList(listaJoints)*/;
		for (auto e : listaJoints)
		{
			listaJointQ.push_back(QString::fromStdString(e));
		}
		return jacobian(listaJointQ, motores, QString::fromStdString(endEffector));
	}

#ifdef PYTHON_BINDINGS_SUPPORT
	QMat jacobianSPython(const  boost::python::list &listaJointsP, const QVec &motores, const std::string &endEffector)
	{
		std::vector<std::string> listaJoint = std::vector<std::string>(boost::python::stl_input_iterator<std::string>(listaJointsP), boost::python::stl_input_iterator<std::string>( ) );
		return jacobianS(listaJoint, motores, endEffector);
	}
#endif


