//
// Created by robolab on 2/6/21.
//

#ifndef DSR_GRAPHEDGERTWIDGET_H
#define DSR_GRAPHEDGERTWIDGET_H

#include <QLabel>

class GraphEdgeRTWidget : public  QWidget
{
    Q_OBJECT
public:
    GraphEdgeRTWidget(std::shared_ptr<DSR::DSRGraph> graph_, const DSR::IDType &from_, const DSR::IDType &to_, const std::string &label_) :
            graph(std::move(graph_)), from(from_), to(to_), edge_type(label_)
    {
        qRegisterMetaType<std::int32_t>("std::int32_t");
        qRegisterMetaType<std::uint32_t>("std::uint32_t");
        qRegisterMetaType<std::uint64_t>("std::uint64_t");
        qRegisterMetaType<uint64_t>("uint64_t");
        qRegisterMetaType<std::string>("std::string");
        qRegisterMetaType<std::map<std::string, DSR::Attribute>>("Attribs");

        connect(graph.get(), &DSR::DSRGraph::update_edge_signal, this, &GraphEdgeRTWidget::add_or_assign_edge_slot, Qt::QueuedConnection);
        //Inner Api
        inner_eigen = graph->get_inner_eigen_api();

        std::optional<DSR::Node> from_node = graph->get_node(from);
        std::optional<DSR::Node> to_node = graph->get_node(to);
        std::optional<DSR::Edge> edge = graph->get_edge(from, to, edge_type);
        qDebug()<<__FUNCTION__ <<from_node.has_value()<<to_node.has_value()<<edge.has_value();
        if (edge.has_value() and from_node.has_value() and to_node.has_value())
        {
            from_string = to_node.value().name();
            to_string = from_node.value().name();

            //TODO: Check this
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
        this->reference = to_string;
        if (combo_text == "world")
        {
            auto root_opt = graph->get_node_root();
            if(root_opt.has_value())
                this->reference = graph->get_node_root().value().name();
        }
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
    void add_or_assign_edge_slot( std::uint64_t from,  std::uint64_t to, const std::string& edge_type)
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

#endif //DSR_GRAPHEDGERTWIDGET_H
