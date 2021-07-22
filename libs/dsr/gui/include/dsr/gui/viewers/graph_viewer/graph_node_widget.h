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

#ifndef GRAPHNODEWIDGET_H
#define GRAPHNODEWIDGET_H

class GraphNodeWidget : public  QTableWidget
{
  Q_OBJECT
  public:
    GraphNodeWidget(std::shared_ptr<DSR::DSRGraph> graph_, DSR::IDType node_id_) : graph(std::move(graph_)), node_id(node_id_)
    {
      qRegisterMetaType<std::int32_t>("std::int32_t");
      qRegisterMetaType<std::uint32_t>("std::uint32_t");
      qRegisterMetaType<std::uint64_t>("std::uint64_t");
      qRegisterMetaType<uint64_t>("uint64_t");
      qRegisterMetaType<std::string>("std::string");
      qRegisterMetaType<std::map<std::string, Attribute>>("Attribs");

      //setWindowFlags(Qt::Widget | Qt::FramelessWindowHint);
      std::optional<Node> n = graph->get_node(node_id_);
      if (n.has_value()) 
      {
          setWindowTitle("Node " + QString::fromStdString(n.value().type()) + " [" + QString::number(node_id) + "]");
          setColumnCount(2);
          std::map<std::string, DSR::Attribute>& attribs = n.value().attrs();
          // show id type and name as attributes
          attribs["ID"] = Attribute(ValType(node_id), 0, 0);
          attribs["type"] = Attribute(ValType(n.value().type()), 0, 0);
          attribs["name"] = Attribute(ValType(n.value().name()), 0, 0);
          setHorizontalHeaderLabels(QStringList{"Key", "Value"});
          for (auto &&[k, v] : attribs) {
                //TODO: check value range and attributes that could be editable
                insert_attribute(k, std::move(v));
          }
          horizontalHeader()->setStretchLastSection(true);
          resize_widget();
          //TODO: comprobar QObject::connect(graph.get(), &DSR::DSRGraph::update_attrs_signal, this, &GraphNodeWidget::drawSLOT);
          //QObject::connect(graph.get(), &DSR::DSRGraph::update_node_signal, this, &GraphNodeWidget::update_node_slot);
          //QObject::connect(graph.get(), &DSR::DSRGraph::update_node_signal, this, &GraphNodeWidget::update_node_slot);
          QObject::connect(graph.get(), &DSR::DSRGraph::update_node_attr_signal, this, &GraphNodeWidget::update_node_attr_slot);
          show();
      }
    };

  public slots:
    void update_node_attr_slot(std::uint64_t node_id, const std::vector<std::string> &type)
    {
        if (node_id != this->node_id)
            return;
        std::optional<Node> n = graph->get_node(node_id);
        if (n.has_value()) {
            auto &attrs = n.value().attrs();
            for (const std::string &attrib_name :type )
            {
                auto value = attrs.find(attrib_name);
                if (value != attrs.end()) {
                    auto &&av = value->second;
                    if (widget_map.count(attrib_name)) {
                        update_attribute_value(attrib_name, std::move(av));
                    } else {
                        insert_attribute(attrib_name, std::move(av));
                    }

                }
            }
        }
    }
    void update_node_slot(std::uint64_t node_id, const std::string &type)
    {
        if (node_id != this->node_id)
            return;
        std::optional<Node> n = graph->get_node(node_id);
        if (n.has_value())
        {
            for (auto &&[k, v] : n.value().attrs()) {
                if(widget_map.count( k )) {
                    update_attribute_value(k, std::move(v));
                }
                else{
                    insert_attribute(k, std::move(v));
                }
            }
        }
    };
    void save_attribute_slot(const std::string &attrib_name)
    {
        std::cout<<"SAVE"<<attrib_name<<std::endl;
    }
    void resizeEvent(QResizeEvent* event) override
    {
      const auto &columns = columnCount();
      for(auto &&index : iter::range(columns))
          setColumnWidth(index, (width()-verticalHeader()->width()-4)/columns);
    }

    void closeEvent (QCloseEvent *event) override
    {
        disconnect(graph.get(), nullptr, this, nullptr);
        //graph.reset();
    };
  private:
    std::shared_ptr<DSR::DSRGraph> graph;
    std::uint64_t node_id;
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
                //auto * widget = qobject_cast<QWidget*>(widget_map[k]);
                //auto * layout = widget->layout();
                //Esto no esta bien. No se controlan nuevos elementos, ni borrados, etc.
                /*if (!v.float_vec().empty() and v.float_vec().size() <= 10) {
                    for (size_t i = 0 ; i < v.float_vec().size(); ++i) {
                        qobject_cast<QDoubleSpinBox *>(layout->itemAt(i)->widget())->setValue(v.float_vec()[i]);
                    }
                }*/
                break;
            }
            case 4: {
                qobject_cast<QComboBox *>(widget_map[k])->setCurrentText(v.bl() ? "true" : "false");
                break;
            }
            case 5: {
                //auto * widget = qobject_cast<QWidget*>(widget_map[k]);
                //auto * layout = widget->layout();
                //Esto no esta bien. No se controlan nuevos elementos, ni borrados, etc.
                /*if (!v.byte_vec().empty() and v.byte_vec().size() <= 10) {
                    for (size_t i = 0 ; i < v.byte_vec().size(); ++i) {
                        qobject_cast<QSpinBox *>(layout->itemAt(i)->widget())->setValue(v.byte_vec()[i]);
                    }
                }*/
                break;
            }
            case 6:{
                qobject_cast<QSpinBox *>(widget_map[k])->setValue((int)v.uint());
                break;
            }
            case 7:{
                qobject_cast<QLineEdit *>(widget_map[k])->setText(QString::fromStdString(std::to_string(v.uint64())));
                break;
            }
            case 8:{
                qobject_cast<QDoubleSpinBox *>(widget_map[k])->setValue(std::round(v.dob()));
                break;
            }
            //TODO: Update u64vec, vec2, vec3, vec4, vec6
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
                    graph->runtime_checked_modify_attrib_local(n.value(), k, text.toStdString());
                    graph->update_node(n.value());
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
                    graph->runtime_checked_modify_attrib_local(n.value(), k, value);
                    graph->update_node(n.value());
                });
                break;
            }
            case 2: {
                QDoubleSpinBox *spin = new QDoubleSpinBox();
                spin->setMinimum(-10000);
                spin->setMaximum(10000);
                spin->setValue(std::round(static_cast<double>(v.fl()) * 1000000) / 1000000);
                setCellWidget(rc, 1, spin);
                widget_map[k] = spin;
                connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, k](double value){
                     std::optional<Node> n = graph->get_node(node_id);
                     graph->runtime_checked_modify_attrib_local(n.value(), k, (float)value);
                     graph->update_node(n.value());
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
                        spin->setMinimum(-10000);
                        spin->setMaximum(10000);
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
                    graph->runtime_checked_modify_attrib_local(n.value(), k, val);
                    graph->update_node(n.value());
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
                    graph->runtime_checked_modify_attrib_local(n.value(), k, (unsigned int)value);
                    graph->update_node(n.value());
                });
                break;
            }
            case 7:
            {
                QLineEdit *ledit = new QLineEdit(QString::fromStdString(std::to_string(v.uint64())));
                setCellWidget(rc, 1, ledit);
                widget_map[k] = ledit;
                break;
            }
            case 8:
            {
                QDoubleSpinBox *spin = new QDoubleSpinBox();
                spin->setMinimum(-10000);
                spin->setMaximum(10000);
                spin->setValue(v.dob());
                setCellWidget(rc, 1, spin);
                widget_map[k] = spin;
                connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, k](double value){
                    std::optional<Node> n = graph->get_node(node_id);
                    graph->runtime_checked_modify_attrib_local(n.value(), k, (double)value);
                    graph->update_node(n.value());
                });
                break;
            }
            case 9:
            {
                QWidget *widget = new QWidget();
                QHBoxLayout *layout = new QHBoxLayout;
                widget->setLayout(layout);
                if (!v.u64_vec().empty() and v.u64_vec().size() <= 10) {
                    for (unsigned char i : v.u64_vec()) {
                        QDoubleSpinBox *spin = new QDoubleSpinBox();
                        spin->setMinimum(0);
                        //spin->setMaximum(100000000000000000);
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
            case 10: {
                QWidget *widget = new QWidget();
                QHBoxLayout *layout = new QHBoxLayout;
                widget->setLayout(layout);
                for (float i : v.vec2()) {
                    QDoubleSpinBox *spin = new QDoubleSpinBox();
                    spin->setMinimum(-10000);
                    spin->setMaximum(10000);
                    spin->setValue(i);
                    layout->addWidget(spin);
                }
                setCellWidget(rc, 1, widget);
                widget_map[k] = widget;
                break;
            }
            case 11: {
                QWidget *widget = new QWidget();
                QHBoxLayout *layout = new QHBoxLayout;
                widget->setLayout(layout);
                for (float i : v.vec3()) {
                    QDoubleSpinBox *spin = new QDoubleSpinBox();
                    spin->setMinimum(-10000);
                    spin->setMaximum(10000);
                    spin->setValue(i);
                    layout->addWidget(spin);
                }
                setCellWidget(rc, 1, widget);
                widget_map[k] = widget;
                break;
            }
            case 12: {
                QWidget *widget = new QWidget();
                QHBoxLayout *layout = new QHBoxLayout;
                widget->setLayout(layout);
                for (float i : v.vec4()) {
                    QDoubleSpinBox *spin = new QDoubleSpinBox();
                    spin->setMinimum(-10000);
                    spin->setMaximum(10000);
                    spin->setValue(i);
                    layout->addWidget(spin);
                }
                setCellWidget(rc, 1, widget);
                widget_map[k] = widget;
                break;
            }
            case 13: {
                QWidget *widget = new QWidget();
                QHBoxLayout *layout = new QHBoxLayout;
                widget->setLayout(layout);
                for (float i : v.vec6()) {
                    QDoubleSpinBox *spin = new QDoubleSpinBox();
                    spin->setMinimum(-10000);
                    spin->setMaximum(10000);
                    spin->setValue(i);
                    layout->addWidget(spin);
                }
                setCellWidget(rc, 1, widget);
                widget_map[k] = widget;
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
#endif // GRAPHNODEWIDGET_H
