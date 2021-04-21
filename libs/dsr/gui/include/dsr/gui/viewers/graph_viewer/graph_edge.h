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

class DoRTStuff : public  QTableWidget
{
Q_OBJECT
public:
    DoRTStuff(std::shared_ptr<DSR::DSRGraph> graph_, const DSR::IDType &from_, const DSR::IDType &to_, std::string label_) :
            graph(std::move(graph_)), rt(graph->get_rt_api()), from(from_), to(to_), label(std::move(label_))
    {
        qRegisterMetaType<DSR::IDType>("DSR::IDType");
        qRegisterMetaType<uint64_t>("uint64_t");

//      qRegisterMetaType<DSR::AttribsMap>("DSR::Attribs");

        std::optional<Node> n = graph->get_node(from);
        std::optional<Node> n2 = graph->get_node(to);
        this->horizontalHeader()->hide();
        this->verticalHeader()->hide();

        if (n.has_value() &&  n2.has_value())
        {
            //TODO: Comprobar esto
            //setWindowModality(Qt::ApplicationModal);
            setWindowTitle("RT: " + QString::fromStdString(n.value().name()) + "(" + QString::fromStdString(n.value().type()) + ") to " + QString::fromStdString(n2.value().name()) + "(" + QString::fromStdString(n2.value().type()) + ")");
            setColumnCount(4);
            setRowCount(9);
            setHorizontalHeaderLabels(QStringList{"a", "b", "c", "d", "", "T", "", "R"});
            setVerticalHeaderLabels(QStringList{"a", "b", "c", "d"});
            horizontalHeader()->setStretchLastSection(true);
            resizeRowsToContents();
            resizeColumnsToContents();
            int width = (this->model()->columnCount() - 1) + this->verticalHeader()->width();
            int height = (this->model()->rowCount() - 1) + this->horizontalHeader()->height();
            for(int column = 0; column < this->model()->columnCount(); column++)
                width = width + this->columnWidth(column);
            for(int row = 0; row < this->model()->rowCount(); row++)
                height = height + this->rowHeight(row);
            this->setMinimumWidth(width);
            this->setMinimumHeight(height);
            QObject::connect(graph.get(), &DSR::DSRGraph::update_edge_signal, this, &DoRTStuff::drawSLOT);
            drawSLOT(from, to);
            show();
            std::cout << __FILE__ << " " << __FUNCTION__ << " End ofDoRTStuff Constructor " << std::endl;
            resize_widget();
        }
    };

    void closeEvent (QCloseEvent *event) override
    {
        disconnect(graph.get(), 0, this, 0);
        graph.reset();
        QTableWidget::closeEvent(event);
    };

    void resizeEvent(QResizeEvent* event)
    {
        const auto &columns = columnCount();
        for(auto &&index : iter::range(columns))
            setColumnWidth(index, ((width()-verticalHeader()->width())/columns)-2);
    }

public slots:
    void drawSLOT(const std::int32_t &from_, const std::int32_t &to_)
    {
        std::cout << __FILE__ << " " << __FUNCTION__ << std::endl;
        if (from==from_ and to==to_)     //ADD LABEL
        {
            try {
                std::optional<Node> node = graph->get_node(from);
                if (node.has_value()) {
                    auto mat = rt->get_edge_RT_as_rtmat(rt->get_edge_RT(node.value(), to).value()).value();
                    // draw RT values
                    for (auto i : iter::range(mat.rows()))
                        for (auto j : iter::range(mat.cols()))
                            if (item(i, j)==0)
                                this->setItem(i, j, new QTableWidgetItem(QString::number(mat(i, j), 'f', 5)));
                            else
                                this->item(i, j)->setText(QString::number(mat(i, j), 'f', 5));
                    // draw translation values
                    auto trans = mat.translation();
                    std::vector<QString> ts{"tx", "ty", "tz"};
                    std::vector<QString> rs{"rx", "ry", "rz"};
                    std::vector<double> rot(mat.rotation().data(), mat.rotation().data() + mat.rotation().size() );
                    for (auto i: iter::range(3)) {
                        if (this->item(4, i)==0) {
                            auto green = new QTableWidgetItem();
                            green->setBackground(QBrush(QColor("lightGreen")));
                            this->setItem(4, i, green);
                        }
                        if (this->item(5, i)==nullptr)
                            this->setItem(5, i, new QTableWidgetItem(ts[i]));
                        else
                            this->item(5, i)->setText(ts[i]);
                        if (this->item(6, i)==nullptr)
                            this->setItem(6, i, new QTableWidgetItem(QString::number(trans[i], 'f', 5)));
                        else
                            this->item(6, i)->setText(QString::number(trans[i], 'f', 5));
                        if (this->item(7, i)==nullptr)
                            this->setItem(7, i, new QTableWidgetItem(rs[i]));
                        else
                            this->item(7, i)->setText(rs[i]);
                        if (this->item(8, i)==0)
                            this->setItem(8, i, new QTableWidgetItem(QString::number(rot[i], 'f', 5)));
                        else
                            this->item(8, i)->setText(QString::number(rot[i], 'f', 5));
                    }
                }
            }
            catch (const std::exception& e) {
                std::cout << "Exception: " << e.what() << " Cannot find attribute named RT in edge going " << from
                          << " to " << to << std::endl;
            }
        }
        this->resize_widget();
    }
private:
    std::shared_ptr<DSR::DSRGraph> graph;
    std::unique_ptr<RT_API> rt;
    int from, to;
    std::string label;
    void resize_widget()
    {
        resizeRowsToContents();
        resizeColumnsToContents();
        int width = (this->model()->columnCount() - 1) + this->verticalHeader()->width() + 4;
        int height = (this->model()->rowCount() - 1) + this->horizontalHeader()->height() ;
        for(int column = 0; column < this->model()->columnCount(); column++)
            width = width + this->columnWidth(column) + 2;
        for(int row = 0; row < this->model()->rowCount(); row++)
            height = height + this->rowHeight(row) ;
        if(abs(width-this->width())>1 or abs(height-this->height())>1) {
            this->setFixedSize(QSize(width, height));
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
        qRegisterMetaType<DSR::IDType>("DSR::IDType");
        
        connect(graph.get(), &DSR::DSRGraph::update_edge_signal, this, &DoRTStuff2::add_or_assign_edge_slot);
        //Inner Api
        inner_eigen = graph->get_inner_eigen_api();
        
        std::optional<Node> n = graph->get_node(from);
        std::optional<Node> n2 = graph->get_node(to);

        if (n.has_value() &&  n2.has_value())
        {
            from_string = n2.value().name();
            to_string = n.value().name();
            
            //TODO: Comprobar esto
            setWindowTitle("RT: " + QString::fromStdString(from_string) + "(" + QString::fromStdString(n.value().type()) + ") to " + QString::fromStdString(to_string) + "(" + QString::fromStdString(n2.value().type()) + ")");
            
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
            update_combo(references_cb->currentText());
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
        graph.reset();
        disconnect(graph.get(), 0, this, 0);
    };
public slots:
    void update_combo(const QString& combo_text)
    {
        this->reference = to_string;
        if (combo_text == "world")
            this->reference = graph->get_node_root().value().name();
        generate_node_transform_list(this->reference, this->from_string);
        add_or_assign_edge_slot(from, to, edge_type);
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
    void add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& edge_type)
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
    int from, to;
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


#endif // GRAPHEDGE_H
