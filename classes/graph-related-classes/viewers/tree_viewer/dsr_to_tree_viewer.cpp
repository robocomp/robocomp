#include "../../CRDT_graphviewer.h"
#include <cppitertools/range.hpp>
#include <qmat/QMatAll>
#include <QTreeWidget>
#include "../../CRDT_graphnode.h"
#include "../../CRDT_graphedge.h"
#include "dsr_to_tree_viewer.h"

using namespace DSR ;

DSRtoTreeViewer::DSRtoTreeViewer(std::shared_ptr<CRDT::CRDTGraph> G_, QWidget *parent) :  QTreeWidget(parent)
{
    qRegisterMetaType<std::int32_t>("std::int32_t");
    qRegisterMetaType<std::string>("std::string");
    G = G_;
    this->setMinimumSize(400, 400);
    //this->setFrameShape(NoFrame);
	//this->adjustSize();
 	setMouseTracking(true);
//	setHeaderHidden(true);
    createGraph();

    connect(G.get(), &CRDT::CRDTGraph::update_node_signal, this,
			[=]( std::int32_t id, std::string type ) {DSRtoTreeViewer::add_or_assign_node_SLOT(id, type);});
    setColumnCount(2);
	QStringList horzHeaders;
	horzHeaders <<"Attribute"<< "Value";

	this->setHeaderLabels( horzHeaders );
	this->header()->setDefaultSectionSize(250);
	//connect(G.get(), &CRDT::CRDTGraph::update_edge_signal, this, &DSRtoGraphViewer::addEdgeSLOT);
//	connect(G.get(), &CRDT::CRDTGraph::del_edge_signal, this, &DSRtoTreeViewer::);
	connect(G.get(), &CRDT::CRDTGraph::del_node_signal, this, &DSRtoTreeViewer::del_node_SLOT);
}

void DSRtoTreeViewer::createGraph()
{
	this->clear();
	qDebug() << __FUNCTION__ << "Reading graph in Tree Viewer";
    try
    {
        auto map = G->getCopy();
		for(const auto &[k, node] : map)
		       add_or_assign_node_SLOT(k,  node);
    }
	catch(const std::exception &e) { std::cout << e.what() << " Error accessing "<< __FUNCTION__<<":"<<__LINE__<< std::endl;}
}

//////////////////////////////////////////////////////////////////////////////////////
///// SLOTS
//////////////////////////////////////////////////////////////////////////////////////
void DSRtoTreeViewer::add_or_assign_node_SLOT(int id, const std::string &type,  const std::string &name)
{
	QTreeWidgetItem* item;
	try {
		if (tree_map.count(id)==0)
		{
			QTreeWidgetItem* symbol_widget = nullptr;
			if (types_map.count(type)) {
				symbol_widget = types_map[type];
			}
			else {
				symbol_widget = new QTreeWidgetItem(this->invisibleRootItem());
				QCheckBox* check_box = new QCheckBox(QString("Node type: "+QString::fromUtf8(type.c_str())));
				check_box->setChecked(true);
				connect(check_box, &QCheckBox::stateChanged, this,
						[=](int value) { DSRtoTreeViewer::category_change_SLOT(value, symbol_widget); });
				this->setItemWidget(symbol_widget, 0, check_box);
				types_map[type] = symbol_widget;

			}

			item = new QTreeWidgetItem(symbol_widget);
			tree_map[id] = item;
		}
		else
		{
			item = tree_map[id];
			// TODO: Look for a better way than removing and adding all the attributes
			foreach(auto i, item->takeChildren()) delete i;
		}
		auto node = G->get_node(id);
		for (auto[key, value] : node->attrs()) {
			QTreeWidgetItem* q_attr = new QTreeWidgetItem(item);
			q_attr->setText(0, QString::fromStdString(key));
			switch (value.value()._d()) {
			case 0: {
				QLineEdit* ledit = new QLineEdit(QString::fromStdString(value.value().str()));
				ledit->setReadOnly(true);
				this->setItemWidget(q_attr, 1, ledit);
			}
				break;
			case 1: {
				QSpinBox* spin = new QSpinBox();
				spin->setReadOnly(true);
				spin->setMinimum(-10000);
				spin->setMaximum(10000);
				spin->setValue(value.value().dec());
				this->setItemWidget(q_attr, 1, spin);

			}
				break;
			case 2: {
				QDoubleSpinBox* spin = new QDoubleSpinBox();
				spin->setReadOnly(true);
				spin->setMinimum(-10000);
				spin->setMaximum(10000);
				spin->setValue(std::round(static_cast<double>(value.value().fl())*1000000)/1000000);
				this->setItemWidget(q_attr, 1, spin);
			}
				break;
			case 3: {
				QWidget* widget = new QWidget();
				QHBoxLayout* layout = new QHBoxLayout;
				widget->setLayout(layout);
				if (!value.value().float_vec().empty()) {
					for (std::size_t i = 0; i<value.value().float_vec().size(); ++i) {
						QDoubleSpinBox* spin = new QDoubleSpinBox();
						spin->setReadOnly(true);
						spin->setMinimum(-10000);
						spin->setMaximum(10000);
						spin->setValue(value.value().float_vec()[i]);
						layout->addWidget(spin);
					}
					this->setItemWidget(q_attr, 1, widget);
				}
			}
				break;
			case 4: {
				QComboBox* combo = new QComboBox();
				combo->setEnabled(false);
				combo->addItem("true");
				combo->addItem("false");
				if (value.value().bl())
					combo->setCurrentText("true");
				else
					combo->setCurrentText("false");
				this->setItemWidget(q_attr, 1, combo);
			}
				break;
			}
		}
		QCheckBox* check_box = new QCheckBox(
				QString("Node "+QString::fromUtf8(name.c_str())+" ["+QString::number(id))+"]");
		check_box->setChecked(true);
		connect(check_box, &QCheckBox::stateChanged, this,
				[=](int value) { DSRtoTreeViewer::node_change_SLOT(value, id, type, item); });
		this->setItemWidget(item, 0, check_box);
//		symbol_widget->addChild(item);

	}
	catch(const std::exception &e) { std::cout << e.what() << " Error in method "<< __FUNCTION__<<" : " << std::endl;}

}

void DSRtoTreeViewer::add_or_assign_node_SLOT(int id, Node node)
{
	auto type = node.type();
	add_or_assign_node_SLOT(id, type, node.name());

}

void DSRtoTreeViewer::add_or_assign_edge_SLOT(std::int32_t from, std::int32_t to, const std::string &edge_tag)
{
	
}

void DSRtoTreeViewer::del_edge_SLOT(const std::int32_t from, const std::int32_t to, const std::string &edge_tag)
{
    std::cout<<__FUNCTION__<<":"<<__LINE__<< std::endl;

}

void DSRtoTreeViewer::del_node_SLOT(int id)
{

    std::cout<<__FUNCTION__<<":"<<__LINE__<< std::endl;
	while (tree_map.count(id) > 0) {
		auto item = tree_map[id];
		this->invisibleRootItem()->removeChild(item);
		delete item;
		tree_map.erase(id);
	}
}

void DSRtoTreeViewer::category_change_SLOT(int value, QTreeWidgetItem* parent)
{
	QCheckBox* sender = qobject_cast<QCheckBox*>(this->sender());
	qDebug()<<"Category checkbox clicked "<<value<< sender->text();
	for (int i = 0; i < parent->childCount (); i++) {
		QTreeWidgetItem* child = parent->child(i);
		auto child_checkbox = qobject_cast<QCheckBox*>(this->itemWidget(child,0));
		child_checkbox->setCheckState((Qt::CheckState)value);
	}
}

void DSRtoTreeViewer::node_change_SLOT(int value, int id, const std::string &type,  QTreeWidgetItem* parent)
{
	QCheckBox* sender = qobject_cast<QCheckBox*>(this->sender());
	if(sender)
	{
		qDebug()<<"Emitting signal for "<<value<< qobject_cast<QCheckBox*>(this->itemWidget(parent,0))->text();
		emit node_check_state_changed(value, id, type, parent);
	}
}
