#include <dsr/gui/dsr_gui.h>
#include <cppitertools/range.hpp>
#include <qmat/QMatAll>
#include <QTreeWidget>
#include <dsr/gui/viewers/graph_viewer/graph_node.h>
#include <dsr/gui/viewers/graph_viewer/graph_edge.h>
#include <dsr/gui/viewers/tree_viewer/tree_viewer.h>

using namespace DSR ;

TreeViewer::TreeViewer(std::shared_ptr<DSR::DSRGraph> G_, QWidget *parent) :  QTreeWidget(parent)
{
    qRegisterMetaType<std::int32_t>("std::int32_t");
    qRegisterMetaType<std::uint32_t>("std::uint32_t");
    qRegisterMetaType<std::uint64_t>("std::uint64_t");
    qRegisterMetaType<uint64_t>("uint64_t");
    qRegisterMetaType<std::string>("std::string");
    G = G_;
    this->setMinimumSize(400, 400);
    //this->setFrameShape(NoFrame);
	//this->adjustSize();
 	setMouseTracking(true);
//	setHeaderHidden(true);
    createGraph();

    connect(G.get(), &DSR::DSRGraph::update_node_signal, this,
			[=, this]( std::uint64_t id, const std::string& type ) {TreeViewer::add_or_assign_node_SLOT(id, type);}, Qt::QueuedConnection);
    setColumnCount(2);
	QStringList horzHeaders;
	horzHeaders <<"Attribute"<< "Value";

	this->setHeaderLabels( horzHeaders );
	this->header()->setDefaultSectionSize(250);
	//connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &GraphViewer::addEdgeSLOT, Qt::QueuedConnection);
//	connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &TreeViewer::);
	connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &TreeViewer::del_node_SLOT, Qt::QueuedConnection);
}

void TreeViewer::createGraph()
{
	this->clear();
	tree_map.clear();
	types_map.clear();
	attributes_map.clear();
	qDebug() << __FUNCTION__ << "Reading graph in Tree Viewer";
    try
    {
        auto map = G->getCopy();
		for(const auto &[k, node] : map)
		       add_or_assign_node_SLOT(k,  node);
    }
	catch(const std::exception &e) { std::cout << e.what() << " Error accessing "<< __FUNCTION__<<":"<<__LINE__<< std::endl;}
}

void TreeViewer::reload(QWidget* widget) {

	if(qobject_cast<TreeViewer*>(widget) != nullptr)
	{
        std::cout<<"Reloading Tree viewer"<<std::endl;
		createGraph();
	}
}

//////////////////////////////////////////////////////////////////////////////////////
///// SLOTS
//////////////////////////////////////////////////////////////////////////////////////
void TreeViewer::add_or_assign_node_SLOT(uint64_t id, const std::string &type,  const std::string &name)
{
	QTreeWidgetItem* item;
	try {
		auto node = G->get_node(id);
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
						[=, this](int value) { TreeViewer::category_change_SLOT(value, symbol_widget); });
				this->setItemWidget(symbol_widget, 0, check_box);
				types_map[type] = symbol_widget;

			}

			item = new QTreeWidgetItem(symbol_widget);
			tree_map[id] = item;
			create_attribute_widgets(item, &node.value());
		}
		else
		{
			item = tree_map[id];
			// TODO: Look for a better way than removing and adding all the attributes
			update_attribute_widgets(&node.value());
		}


		QCheckBox* check_box = new QCheckBox(
				QString("Node "+QString::fromUtf8(name.c_str())+" ["+QString::number(id))+"]");
		check_box->setChecked(true);
		connect(check_box, &QCheckBox::stateChanged, this,
				[=, this](int value) { TreeViewer::node_change_SLOT(value, id, type, item); });
		this->setItemWidget(item, 0, check_box);
//		symbol_widget->addChild(item);

	}
	catch(const std::exception &e) { std::cout << e.what() << " Error in method "<< __FUNCTION__<<" : " << std::endl;}

}

void TreeViewer::add_or_assign_node_SLOT(uint64_t id, Node node)
{
	auto type = node.type();
	add_or_assign_node_SLOT(id, type, node.name());

}

void TreeViewer::add_or_assign_edge_SLOT(std::uint64_t from, std::uint64_t to, const std::string &edge_tag)
{
	
}

void TreeViewer::del_edge_SLOT(const std::uint64_t from, const std::uint64_t to, const std::string &edge_tag)
{
    qDebug()<<__FUNCTION__<<":"<<__LINE__;

}

void TreeViewer::del_node_SLOT(uint64_t id)
{

    qDebug()<<__FUNCTION__<<":"<<__LINE__;
	while (tree_map.count(id) > 0) {
		auto item = tree_map[id];
		this->invisibleRootItem()->removeChild(item);
		delete item;
		tree_map.erase(id);
	}
}

void TreeViewer::category_change_SLOT(int value, QTreeWidgetItem* parent)
{
	QCheckBox* sender = qobject_cast<QCheckBox*>(this->sender());
	qDebug()<<"Category checkbox clicked "<<value<< sender->text();
	for (int i = 0; i < parent->childCount (); i++) {
		QTreeWidgetItem* child = parent->child(i);
		auto child_checkbox = qobject_cast<QCheckBox*>(this->itemWidget(child,0));
		child_checkbox->setCheckState((Qt::CheckState)value);
	}
}

void TreeViewer::node_change_SLOT(int value, uint64_t id, const std::string &type,  QTreeWidgetItem* parent)
{
	QCheckBox* sender = qobject_cast<QCheckBox*>(this->sender());
	if(sender)
	{
		qDebug()<<"Emitting signal for "<<value<< qobject_cast<QCheckBox*>(this->itemWidget(parent,0))->text();
		emit node_check_state_changed(value, id, type, parent);
	}
}


void TreeViewer::create_attribute_widgets(QTreeWidgetItem* parent, Node* node)
{
	for (auto[key, value] : node->attrs()) {
		// check if attribute widget already exists
		if(attributes_map.count(node->id()) > 0 and attributes_map[node->id()].count(key) > 0)
			continue;
		create_attribute_widget(parent, node, key, value);
	}

}

void TreeViewer::create_attribute_widget(QTreeWidgetItem* parent, Node* node, std::string key, Attribute value)
{
	QTreeWidgetItem* q_attr = new QTreeWidgetItem(parent);
	attributes_map[node->id()][key] = q_attr;
	q_attr->setText(0, QString::fromStdString(key));
	switch (value.selected()) {
	case 0: {
		QLineEdit* ledit = new QLineEdit(QString::fromStdString(value.str()));
		ledit->setReadOnly(true);
		this->setItemWidget(q_attr, 1, ledit);
	}
		break;
	case 1: {
		QSpinBox* spin = new QSpinBox();
		spin->setReadOnly(true);
		spin->setMinimum(-10000);
		spin->setMaximum(10000);
		spin->setValue(value.dec());
		this->setItemWidget(q_attr, 1, spin);

	}
		break;
	case 2: {
		QDoubleSpinBox* spin = new QDoubleSpinBox();
		spin->setReadOnly(true);
		spin->setMinimum(-10000);
		spin->setMaximum(10000);
		spin->setValue(std::round(static_cast<double>(value.fl())*1000000)/1000000);
		this->setItemWidget(q_attr, 1, spin);
	}
		break;
	case 3: {
		QWidget* widget = new QWidget();
		QHBoxLayout* layout = new QHBoxLayout;
		widget->setLayout(layout);
		if (!value.float_vec().empty()) {
			for (std::size_t i = 0; i<value.float_vec().size(); ++i) {
				QDoubleSpinBox* spin = new QDoubleSpinBox();
				spin->setReadOnly(true);
				spin->setMinimum(-10000);
				spin->setMaximum(10000);
				spin->setValue(value.float_vec()[i]);
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
		if (value.bl())
			combo->setCurrentText("true");
		else
			combo->setCurrentText("false");
		this->setItemWidget(q_attr, 1, combo);
	}
		break;
	case 6: {
            QSpinBox* spin = new QSpinBox();
            spin->setReadOnly(true);
            spin->setMinimum(-10000);
            spin->setMaximum(10000);
            spin->setValue(value.uint());
            this->setItemWidget(q_attr, 1, spin);

        }
            break;
	}

}

void TreeViewer::update_attribute_widgets(Node* node)
{
	for (auto[key, value] : node->attrs()) {
		QTreeWidgetItem* q_attr = attributes_map[node->id()][key];
		if(not q_attr) {
			create_attribute_widget(tree_map[node->id()], node, key, value);
			q_attr = attributes_map[node->id()][key];
			if(not q_attr)
				throw std::runtime_error("Problem creating tree widget for node" +node->name()+" attribute "+key);
		}
		switch (value.selected()) {
            case 0: {

                QLineEdit *ledit = qobject_cast<QLineEdit *>(this->itemWidget(q_attr, 1));
                ledit->setText(QString::fromStdString(value.str()));

            }
                break;
            case 1: {
                QSpinBox *spin = qobject_cast<QSpinBox *>(this->itemWidget(q_attr, 1));
                spin->setValue(value.dec());
            }
                break;
            case 2: {
                QDoubleSpinBox *spin = qobject_cast<QDoubleSpinBox *>(this->itemWidget(q_attr, 1));
                spin->setValue(std::round(static_cast<double>(value.fl()) * 1000000) / 1000000);
            }
                break;
            case 3: {
                QWidget *widget = qobject_cast<QWidget *>(this->itemWidget(q_attr, 1));
                int count = 0;
                for (auto spin : widget->findChildren<QDoubleSpinBox *>()) {
                    spin->setValue(value.float_vec()[count]);
                    count++;
                }
            }
                break;
            case 4: {
                QComboBox *combo = qobject_cast<QComboBox *>(this->itemWidget(q_attr, 1));
                if (value.bl())
                    combo->setCurrentText("true");
                else
                    combo->setCurrentText("false");
            }
                break;

            case 6: {
                QSpinBox *spin = qobject_cast<QSpinBox *>(this->itemWidget(q_attr, 1));
                spin->setValue(value.uint());
            }
                break;
        }
	}
}
