//
// Created by robolab on 2/6/21.
//

#ifndef DSR_GRAPHEDGEWIDGET_H
#define DSR_GRAPHEDGEWIDGET_H

class GraphEdgeWidget : public QTableWidget
{
    Q_OBJECT
public:
    GraphEdgeWidget(std::shared_ptr<DSR::DSRGraph> graph_, const DSR::IDType &from_, const DSR::IDType &to_, const std::string &label_) :
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
        qDebug()<<__FUNCTION__ <<" GraphEdgeWidget"<<from_node.has_value()<<to_node.has_value()<<edge.has_value();
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
            QObject::connect(graph.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &GraphEdgeWidget::update_edge_attr_slot);
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
            spin->setMinimum(std::numeric_limits<int>::min());
            spin->setMaximum(std::numeric_limits<int>::max());
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
            spin->setMaximum(std::numeric_limits<uint32_t>::max());
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
            removeRow(rc);
        }
    }
};

#endif //DSR_GRAPHEDGEWIDGET_H
