/*
 * Copyright 2018 <copyright holder> <email>
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
 */

#ifndef GRAPHEDGE_H
#define GRAPHEDGE_H

#include <QGraphicsItem>
#include <QContextMenuEvent>
#include <QTableWidget>
#include <memory>
#include <QPen>
#include <utility>
#include <cppitertools/zip.hpp>
#include <QLabel>
#include <dsr/gui/dsr_gui.h>
#include <qmat/QMatAll>
#include <QHeaderView>
#include <cppitertools/range.hpp>

using namespace DSR;

class DoTableEdgeStuff : public  QTableWidget
{
    Q_OBJECT
    public:
        DoTableEdgeStuff(std::shared_ptr<DSR::DSRGraph> graph_, const DSR::IDType &from_, const DSR::IDType &to_, const std::string &label_) :
                            graph(std::move(graph_)), from(from_), to(to_), edge_type(label_)
        {
            qRegisterMetaType<std::int32_t>("std::int32_t");
            qRegisterMetaType<std::uint32_t>("std::uint32_t");
            qRegisterMetaType<std::uint64_t>("std::uint64_t");
            qRegisterMetaType<uint64_t>("uint64_t");
            qRegisterMetaType<std::string>("std::string");
            qRegisterMetaType<std::map<std::string, Attribute>>("Attribs");

            //setWindowFlags(Qt::Widget | Qt::FramelessWindowHint);
            std::optional<Node> from_node = graph->get_node(from);
            std::optional<Node> to_node = graph->get_node(to);
            std::optional<Edge> edge = graph->get_edge(from, to, edge_type);
            qDebug()<<__FUNCTION__ <<" DoTableEdgeStuff"<<from_node.has_value()<<to_node.has_value()<<edge.has_value();
            if (edge.has_value() and from_node.has_value() and to_node.has_value())
            {
                setWindowTitle("Edge " + QString::fromStdString(edge.value().type()) + " from [" + QString::number(from_node.value().id()) +
                                "] to [" + QString::number(to_node.value().id()) + "]");
                setColumnCount(2);
                std::map<std::string, DSR::Attribute> attribs;
                attribs = edge.value().attrs();
                // show id type and name as attributes
                //attribs["ID"] = Attribute(ValType(node_id), 0, 0);
                attribs["type"] = Attribute(ValType(edge.value().type()), 0, 0);
                //attribs["name"] = Attribute(ValType(edge.value().name()), 0, 0);
                setHorizontalHeaderLabels(QStringList{"Key", "Value"});
                for (auto &&[k, v] : attribs) {
                 //TODO: check value range and attributes that could be editable
                    insert_attribute(k, std::move(v));
                }
                horizontalHeader()->setStretchLastSection(true);
                resize_widget();
                //TODO: comprobar QObject::connect(graph.get(), &DSR::DSRGraph::update_attrs_signal, this, &DoTableStuff::drawSLOT);
                //QObject::connect(graph.get(), &DSR::DSRGraph::update_node_signal, this, &DoTableStuff::update_node_slot);
                //QObject::connect(graph.get(), &DSR::DSRGraph::update_node_signal, this, &DoTableStuff::update_node_slot);
                QObject::connect(graph.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &DoTableEdgeStuff::update_edge_attr_slot);
                //QObject::connect(graph.get(), &DSR::DSRGraph::update_edge_signal, this, &DoTableStuff::add_or_assign_edge_slot);
                show();
            }
        };

    public slots:
        void update_edge_attr_slot(uint64_t from, uint64_t to, const std::vector<std::string>& att_name)
        {
            if ((from != this->from) and (to != this->to))
                return;
            if( std::optional<Edge> edge = graph->get_edge(from, to, edge_type); (edge.has_value()))
            {
                auto &attrs = edge.value().attrs();
                for (const std::string &attrib_name : att_name )
                {
                    if (auto value = attrs.find(attrib_name); value != attrs.end())
                    {
                        auto &&av = value->second;
                        if (widget_map.count(attrib_name))
                        {
                            update_attribute_value(attrib_name, std::move(av));
                        } else
                        {
                            insert_attribute(attrib_name, std::move(av));
                        }
                    }
                }
            }
        }
        void resizeEvent(QResizeEvent* event)
        {
            const auto &columns = columnCount();
            for(auto &&index : iter::range(columns))
                setColumnWidth(index, (width()-verticalHeader()->width()-4)/columns);
        }

        void closeEvent (QCloseEvent *event)
        {
            disconnect(graph.get(), 0, this, 0);
        };
    private:
        std::shared_ptr<DSR::DSRGraph> graph;
        std::uint64_t node_id;
        uint64_t from, to;
        std::string edge_type;
        std::map<std::string, QWidget*> widget_map;
        void resize_widget()
        {
            resizeRowsToContents();
            resizeColumnsToContents();
            int width = (this->model()->columnCount() - 1) + this->verticalHeader()->width() + 4;
            int height = (this->model()->rowCount() - 1) + this->horizontalHeader()->height() ;
            for(int column = 0; column < this->model()->columnCount(); column++)
                width = width + this->columnWidth(column);
            for(int row = 0; row < this->model()->rowCount(); row++)
                height = height + this->rowHeight(row);
            this->setMinimumWidth(width);
            this->setMinimumHeight(height);
        }
        void update_attribute_value(const std::string &k, DSR::Attribute &&v)
        {
            widget_map[k]->blockSignals(true);
            switch (v.selected()) {
                case 0: {
                    qobject_cast<QLineEdit *>(widget_map[k])->setText(QString::fromStdString(v.str()));
                    break;
                }
                case 1: {
                    qobject_cast<QSpinBox *>(widget_map[k])->setValue(v.dec());
                    break;
                }
                case 2: {
                    qobject_cast<QDoubleSpinBox *>(widget_map[k])->setValue(std::round(static_cast<double>(v.fl()) * 1000000) / 1000000);
                    break;
                }
                case 3: {
                    break;
                }
                case 4: {
                    qobject_cast<QComboBox *>(widget_map[k])->setCurrentText(v.bl() ? "true" : "false");
                    break;
                }
                case 5: {
                    break;
                }
                case 6:{
                    qobject_cast<QSpinBox *>(widget_map[k])->setValue(v.uint());
                    break;
                }
                case 7:{
                    qobject_cast<QLineEdit *>(widget_map[k])->setText(QString::fromStdString(std::to_string(v.uint64())));
                    break;
                }
            }
            widget_map[k]->blockSignals(false);
        }
        void insert_attribute(const std::string &k, DSR::Attribute &&v)
        {
            bool inserted = true;

            int rc = rowCount();
            insertRow( rc );

            switch (v.selected()) {
                case 0: {
                    QLineEdit *ledit = new QLineEdit(QString::fromStdString(v.str()));
                    setCellWidget(rc, 1, ledit);
                    widget_map[k] = ledit;
                    connect(ledit, &QLineEdit::textChanged, this, [this, k](const QString& text){
                        std::optional<Node> n = graph->get_node(node_id);
                        graph->runtime_checked_update_attrib_by_name(n.value(), k, text.toStdString());
                    });
                    break;
                }
                case 1:
                {
                    QSpinBox *spin = new QSpinBox();
                    spin->setMinimum(-100000);
                    spin->setMaximum(100000);
                    spin->setValue(v.dec());
                    setCellWidget(rc, 1, spin);
                    widget_map[k] = spin;
                    connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this, k](int value){
                        std::optional<Node> n = graph->get_node(node_id);
                        graph->runtime_checked_update_attrib_by_name(n.value(), k, value);
                    });
                    break;
                }
                case 2: {
                    QDoubleSpinBox *spin = new QDoubleSpinBox();
                    spin->setMinimum(-100000);
                    spin->setMaximum(100000);
                    spin->setValue(std::round(static_cast<double>(v.fl()) * 1000000) / 1000000);
                    setCellWidget(rc, 1, spin);
                    widget_map[k] = spin;
                    connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, k](double value){
                        std::optional<Node> n = graph->get_node(node_id);
                        graph->runtime_checked_update_attrib_by_name(n.value(), k, (float)value);
                    });
                    break;
                }
                case 3: {
                    QWidget *widget = new QWidget();
                    QHBoxLayout *layout = new QHBoxLayout;
                    widget->setLayout(layout);
                    if (!v.float_vec().empty() and v.float_vec().size() <= 10) {
                        for (float i : v.float_vec()) {
                            QDoubleSpinBox *spin = new QDoubleSpinBox();
                            spin->setMinimum(-100000);
                            spin->setMaximum(100000);
                            spin->setValue(i);
                            layout->addWidget(spin);
                        }
                        setCellWidget(rc, 1, widget);
                        widget_map[k] = widget;
                    } else {
                        inserted = false;
                    }
                    break;
                }
                case 4:{
                    QComboBox *combo = new QComboBox();
                    combo->addItem("true");
                    combo->addItem("false");
                    combo->setCurrentText(v.bl() ? "true" : "false");
                    setCellWidget(rc, 1, combo);
                    widget_map[k] = combo;
                    connect(combo, &QComboBox::currentTextChanged, this, [this, k](const QString& value){
                        std::optional<Node> n = graph->get_node(node_id);
                        bool val = ( value == "true");
                        graph->runtime_checked_update_attrib_by_name(n.value(), k, val);
                    });
                    break;
                }
                case 5:
                {
                    QWidget *widget = new QWidget();
                    QHBoxLayout *layout = new QHBoxLayout;
                    widget->setLayout(layout);
                    if (!v.byte_vec().empty() and v.byte_vec().size() <= 10) {
                        for (unsigned char i : v.byte_vec()) {
                            QSpinBox *spin = new QSpinBox();
                            spin->setMinimum(0);
                            spin->setMaximum(255);
                            spin->setValue(i);
                            layout->addWidget(spin);
                        }
                        setCellWidget(rc, 1, widget);
                        widget_map[k] = widget;
                    } else {
                        inserted = false;
                    }
                    break;
                }
                case 6:
                {
                    QSpinBox *spin = new QSpinBox();
                    spin->setMinimum(0);
                    spin->setMaximum(100000);
                    spin->setValue((int)v.uint());
                    setCellWidget(rc, 1, spin);
                    widget_map[k] = spin;
                    connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this, k](int value){
                        std::optional<Node> n = graph->get_node(node_id);
                        graph->runtime_checked_update_attrib_by_name(n.value(), k, (unsigned int)value);
                    });
                    break;
                }
                case 7:{
                    QLineEdit *ledit = new QLineEdit(QString::fromStdString(std::to_string(v.uint64())));
                    setCellWidget(rc, 1, ledit);
                    widget_map[k] = ledit;
                    break;
                }
            }

            if (inserted)
            {
                auto *item =new QTableWidgetItem(QString::fromStdString(k));
                item->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEnabled);
                setItem(rc, 0, item);

            } else {
                removeRow(rc+1);
            }
        }
};

class DoRTStuff2 : public  QWidget
{
Q_OBJECT
public:
    DoRTStuff2(std::shared_ptr<DSR::DSRGraph> graph_, const DSR::IDType &from_, const DSR::IDType &to_, const std::string &label_) :
            graph(std::move(graph_)), from(from_), to(to_), edge_type(label_)
    {
        qRegisterMetaType<std::int32_t>("std::int32_t");
        qRegisterMetaType<std::uint32_t>("std::uint32_t");
        qRegisterMetaType<std::uint64_t>("std::uint64_t");
        qRegisterMetaType<uint64_t>("uint64_t");
        qRegisterMetaType<std::string>("std::string");
        qRegisterMetaType<std::map<std::string, Attribute>>("Attribs");

        connect(graph.get(), &DSR::DSRGraph::update_edge_signal, this, &DoRTStuff2::add_or_assign_edge_slot);
        //Inner Api
        inner_eigen = graph->get_inner_eigen_api();

        std::optional<Node> from_node = graph->get_node(from);
        std::optional<Node> to_node = graph->get_node(to);
        std::optional<Edge> edge = graph->get_edge(from, to, edge_type);
        qDebug()<<__FUNCTION__ <<from_node.has_value()<<to_node.has_value()<<edge.has_value();
        if (edge.has_value() and from_node.has_value() and to_node.has_value())
        {
            from_string = to_node.value().name();
            to_string = from_node.value().name();

            //TODO: Comprobar esto
            setWindowTitle("RT: " + QString::fromStdString(from_string) + "(" + QString::fromStdString(from_node.value().type()) + ") to " + QString::fromStdString(to_string) + "(" + QString::fromStdString(to_node.value().type()) + ")");
            qDebug()<<__FUNCTION__ <<"RT: " + QString::fromStdString(from_string) + "(" + QString::fromStdString(from_node.value().type()) + ") to " + QString::fromStdString(to_string) + "(" + QString::fromStdString(to_node.value().type()) + ")";
            QVBoxLayout *vbox = new QVBoxLayout(this);
            //Reference combo
            QComboBox *references_cb = new QComboBox();
            QHBoxLayout *reference_layout = new QHBoxLayout();
            references_cb->addItem("world");
            references_cb->addItem("parent");
            connect(references_cb, SIGNAL(currentTextChanged(QString)), this, SLOT(update_combo(QString)));
            reference_layout->addWidget(new QLabel("Reference:"));
            reference_layout->addWidget(references_cb);
            vbox->addLayout(reference_layout);

            //Add attribs to show
            qDebug()<<__FUNCTION__ <<"Add atributes to show";
            int pos=0;
            for(std::vector<std::string> row : attrib_names)
            {
                QHBoxLayout *hlayout = new QHBoxLayout();
                for(std::string name : row)
                {
                    hlayout->addWidget(new QLabel(QString::fromStdString(name)));
                    QLineEdit *ledit = new QLineEdit();
                    ledit->setReadOnly(true);
                    hlayout->addWidget(ledit);
                    attrib_widgets[pos] = ledit;
                    pos++;
                }
                vbox->addLayout(hlayout);
            }
            //TODO: temporary added to check yolo pose estimation
            qDebug()<<__FUNCTION__ <<"Yolo pose";
            if (label_ == "looking-at")
            {
                QHBoxLayout* looking_layout = new QHBoxLayout();
                looking_layout->addWidget(new QLabel("Pose estimation:"));
                vbox->addLayout(looking_layout);
                //Add attribs to show
                int pos=9;
                for(std::vector<std::string> row : attrib_names)
                {
                    QHBoxLayout *hlayout = new QHBoxLayout();
                    for(std::string name : row)
                    {
                        hlayout->addWidget(new QLabel(QString::fromStdString(name)));
                        QLineEdit *ledit = new QLineEdit();
                        ledit->setReadOnly(true);
                        hlayout->addWidget(ledit);
                        attrib_widgets[pos] = ledit;
                        pos++;
                    }
                    vbox->addLayout(hlayout);
                }
                std::cout<<"size"<<pos<<std::endl;
            }
            qDebug()<<__FUNCTION__ <<"update combo";
            update_combo(references_cb->currentText());
            qDebug()<<__FUNCTION__<<"To show";
            show();
        }
    };
    void generate_node_transform_list(const std::string& to, const std::string& from)
    {
        transform_set.clear();
        transform_set.insert(from);
        auto node = graph->get_node(from);
        try{
            std::optional<Node> parent_node;
            do
            {
                parent_node = graph->get_parent_node(node.value());
                if (parent_node.has_value())
                {
                    transform_set.insert(parent_node.value().name());
                    node = parent_node;
                }
            }while(node.value().type() != "world" and node.value().name() != to);
        }catch(...){}
    };
    void closeEvent (QCloseEvent *event) override
    {
        //graph.reset();
        disconnect(graph.get(), 0, this, 0);
    };
public slots:
    void update_combo(const QString& combo_text)
    {
        qDebug()<<__FUNCTION__ <<combo_text;
        this->reference = to_string;
        qDebug()<<__FUNCTION__ <<__LINE__;
        if (combo_text == "world")
        {
            qDebug()<<__FUNCTION__ <<__LINE__;
            auto patata = graph->get_node_root().value().name();
//            qDebug()<<__FUNCTION__ <<__LINE__<<patata;
            this->reference = graph->get_node_root().value().name();
            qDebug()<<__FUNCTION__ <<__LINE__;
        }

        qDebug()<<__FUNCTION__ <<__LINE__;
        generate_node_transform_list(this->reference, this->from_string);
        qDebug()<<__FUNCTION__ <<__LINE__;
        add_or_assign_edge_slot(from, to, edge_type);
        qDebug()<<__FUNCTION__ <<__LINE__;
    };
    void update_values()
    {
        std::optional<Mat::Vector6d> transform = inner_eigen->transform_axis(this->reference, this->from_string);
        if (transform.has_value())
        {
            for(unsigned int pos = 0;pos < 6;pos++)
            {
                const double &value = transform.value()[pos];
                attrib_widgets[pos]->setText(QString::number(value));
            }
            //convert degress
            for(unsigned int pos = 3;pos < 6;pos++)
            {
                const double &value = transform.value()[pos] * 180 / M_PI;
                attrib_widgets[3 + pos]->setText(QString::number(value));
            }
        }
        else
            std::cout<<"Error retriving RT data"<<std::endl;
    };
    void add_or_assign_edge_slot(const std::uint64_t from, const std::uint64_t to, const std::string& edge_type)
    {
        std::cout<<"edge-"<<edge_type<<std::endl;
        if(edge_type == "RT" or edge_type == "looking-at")
        {
            std::optional<Node> from_node = graph->get_node(from);
            std::optional<Node> to_node = graph->get_node(to);
            if(from_node.has_value() and to_node.has_value())
            {
                std::string from_str = from_node.value().name();
                std::string to_str = to_node.value().name();
                //check if any node is involved in actual reference transform
                if(transform_set.find(from_str)!= transform_set.end() or transform_set.find(to_str) != transform_set.end())
                    update_values();
            }
        }
        if(edge_type == "looking-at")
        {
            std::optional<DSR::Edge> edge = graph->get_edge(from, to, edge_type);
            if(edge.has_value())
            {
                std::optional<const std::vector<float>>rotation = graph->get_attrib_by_name<looking_at_rotation_euler_xyz_att>(edge.value());
                std::optional<const std::vector<float>>translation = graph->get_attrib_by_name<looking_at_translation_att>(edge.value());
                if(rotation.has_value() and translation.has_value())
                {
                    for(unsigned int pos = 9;pos < 12;pos++)
                    {
                        const double &value = translation.value()[pos-9];
                        attrib_widgets[pos]->setText(QString::number(value));
                    }
                    for(unsigned int pos = 12;pos < 15;pos++)
                    {
                        const double &value = rotation.value()[pos-12];
                        attrib_widgets[pos]->setText(QString::number(value));
                        //convert degress
                        const double &value_d = value * 180 / M_PI;
                        attrib_widgets[3 + pos]->setText(QString::number(value_d));
                    }
                    std::cout<<"print values"<<rotation.value()<<translation.value()<<std::endl;
                }
            }
        }
    };

private:
    std::shared_ptr<DSR::DSRGraph> graph;
    std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
    uint64_t from, to;
    std::string edge_type;
    std::string from_string;
    std::string to_string;
    std::string reference;
    std::map<int, QLineEdit*> attrib_widgets;
    std::vector<std::vector<std::string>> attrib_names = {{"X", "Y", "Z"}, {"RX (rad)", "RY (rad)", "RZ (rad)"}, {"RX (deg)", "RY (deg)", "RZ (deg)"} };
    std::set<std::string> transform_set;
};



enum ItemFlags
{
    IF_FramelessSelection = 1,
    IF_DeleteAllowed = 2,
    IF_LastFlag = 4
};

enum ConnectionFlags	// extends ItemFlags
{
    CF_Start_Arrow		= IF_LastFlag,
    CF_End_Arrow		= IF_LastFlag << 2,
    CF_Mutual_Arrows	= CF_Start_Arrow | CF_End_Arrow		// start | end
};

class GraphEdge : public QObject, public QGraphicsLineItem, public std::enable_shared_from_this<GraphEdge>
{
Q_OBJECT
Q_PROPERTY(int edge_pen READ _edge_pen WRITE set_edge_pen)
private:
    GraphNode *source, *dest;
    qreal arrowSize;
    QGraphicsSimpleTextItem *tag;
    QColor color;
//    QGraphicsTextItem *rt_values = nullptr;
    QTableWidget *label = nullptr;
    int line_width;
    QPropertyAnimation* animation;
//    QPolygonF tag_polygon;
//    bool graphic_debug;
//    QRectF calculatedBoundingRect;
//    int bend_factor;
//    int m_itemFlags;
//    int m_internalStateFlags;

public:
    GraphEdge(GraphNode *sourceNode, GraphNode *destNode, const QString &edge_name);
    GraphNode *sourceNode() const;
    GraphNode *destNode() const;
    void adjust(GraphNode* node= nullptr, QPointF pos=QPointF());
//    int type() const override { return Type; }
//    QString getTag() const { return tag->text();};
    int _edge_pen();
    void set_edge_pen(const int with);
    void change_detected();
//    bool isValid() const	{ return source != NULL && dest != NULL; }
//    bool isCircled() const	{ return isValid() && source == dest; }

//protected:
//    QPainterPath shape() const override;
    QRectF boundingRect() const override;
//    QRectF calculateBoundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    void keyPressEvent(QKeyEvent *event) override;
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override;
    void update_edge_attr_slot(std::uint64_t from, std::uint64_t to, const std::vector<std::string>& att_name);

protected:
//    QPointF m_controlPoint, m_controlPos;
//    QPainterPath m_shapeCachePath;
//    QPolygon arrow_polygon_cache;
    const int ARROW_SIZE = 6;
    const int NODE_DIAMETER = 20;
    const int NODE_RADIUS = NODE_DIAMETER/2;



//private:
//    /*virtual*/ void drawArrow(QPainter *painter, const QStyleOptionGraphicsItem *option, bool first, const QLineF &direction) const;
//    /*virtual*/ void drawArrow(QPainter *painter, qreal shift, const QLineF &direction) const;
//    QLineF calculateArrowLine(const QPainterPath &path, bool first, const QLineF &direction) const;

};
Q_DECLARE_METATYPE(DSR::Edge);

#endif // GRAPHEDGE_H
