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

#ifndef GRAPHNODERGBDWIDGET_H
#define GRAPHNODERGBDWIDGET_H

class GraphNodeRGBDWidget : public QWidget
{
  Q_OBJECT
  QLabel rgbd_label, depth_label;
  QMenuBar *mainMenu;
  QAction *show_rgb;
  QAction *show_depth;
  std::chrono::system_clock::time_point last_update_rgb, last_update_d ;

  public:
    GraphNodeRGBDWidget(std::shared_ptr<DSR::DSRGraph> graph_, DSR::IDType node_id_) : graph(std::move(graph_)), node_id(node_id_)
    {
      //cam = graph->get_camera_api(graph->get_nodes_by_type("rgbd").at(0));
      setWindowTitle(QString::fromStdString(graph->get_agent_name()) + "-RGBD");
      //QObject::connect(graph.get(), &DSR::DSRGraph::update_node_signal, this, &GraphNodeRGBDWidget::drawRGBDSLOT);
      QObject::connect(graph.get(), &DSR::DSRGraph::update_node_attr_signal, this, &GraphNodeRGBDWidget::drawRGBDSLOT);
      QHBoxLayout *layout = new QHBoxLayout();
      layout->addWidget(&rgbd_label);
      layout->addWidget(&depth_label);
      setLayout(layout);
      //MenuBar
      mainMenu = new QMenuBar(this);
      QMenu *viewMenu = mainMenu->addMenu(GraphNodeRGBDWidget::tr("&View"));
      show_rgb = new QAction("RGB", this);
      show_rgb->setStatusTip(tr("Show RGB image"));
      show_rgb->setCheckable(true);
      show_rgb->setChecked(true);
      viewMenu->addAction(show_rgb);
      show_depth = new QAction("Depth", this);
      show_depth->setStatusTip(tr("Show DEPTH image"));
      show_depth->setCheckable(true);
      show_depth->setChecked(true);
      viewMenu->addAction(show_depth);
      show();
    };

    void closeEvent (QCloseEvent *event) override
    {
        disconnect(graph.get(), nullptr, this, nullptr);
        //graph.reset();
        //cam.reset();
    };

  public slots:
    void drawRGBDSLOT( uint64_t id, const std::vector<std::string> &type)
    {
      if( static_cast<uint64_t>(id) != node_id) return;
      bool rgb = std::any_of(type.begin(), type.end(), [](auto& e){ return e == cam_rgb_att::attr_name;});
      bool d = std::any_of(type.begin(), type.end(), [](auto& e){ return e == cam_depth_att::attr_name;});

      auto now = std::chrono::system_clock::now();
      bool update_rgb = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_rgb)  > std::chrono::milliseconds(20);
      bool update_d = d = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_d)  > std::chrono::milliseconds(20);


      if ( (rgb and update_rgb) or (d and update_d) )
      {
          std::optional<Node> n = (show_rgb->isChecked() or show_depth->isChecked()) ?  graph->get_node(id) :  std::nullopt;
          if (n.has_value()) {
              Node node = n.value();
              //auto t = get_unix_timestamp();
              //std::cout << "[DRAW IMG] " << timestamp << ", " << t << ": " << static_cast<double>(t - timestamp) / 1000000 << std::endl;
              if (cam == nullptr) {
                  if (graph->get_attrib_by_name<cam_rgb_focalx_att>(node).has_value())
                      cam = graph->get_camera_api(node);
                  else return;
              }
              //rgb
              if (rgb and update_rgb) {
                  if (show_rgb->isChecked()) {
                      const auto rgb_data = graph->get_attrib_by_name<cam_rgb_att>(node);//cam->get_rgb_image();
                      const auto rgb_width = graph->get_attrib_by_name<cam_rgb_width_att>(node);
                      const auto rgb_height = graph->get_attrib_by_name<cam_rgb_height_att>(node);

                      if (rgb_data.has_value() and rgb_width.has_value() and rgb_height.has_value()) {
                          const std::vector<uint8_t> &img = rgb_data.value();//.get();
                          auto pix = QPixmap::fromImage(
                                  QImage(&img[0], rgb_width.value(), rgb_height.value(), QImage::Format_RGB888));
                          rgbd_label.setPixmap(pix);
                          last_update_rgb = now;
                      }
                  } else
                      rgbd_label.clear();
              }
              //depth
              if (d and update_d) {
                  if (show_depth->isChecked()) {
                      const auto depth_width = graph->get_attrib_by_name<cam_depth_width_att>(n.value());
                      const auto depth_height = graph->get_attrib_by_name<cam_depth_height_att>(n.value());
                      const std::optional<std::vector<std::uint8_t>> gray_scale = cam->get_depth_as_gray_image();
                      if (gray_scale.has_value() and depth_width.has_value() and depth_height.has_value()) {
                          depth_label.setPixmap(QPixmap::fromImage(
                                  QImage(&gray_scale.value()[0], depth_width.value(), depth_height.value(),
                                         QImage::Format_Indexed8)));
                          last_update_d = now;
                      }
                  } else {
                      depth_label.clear();
                  }
              }
              this->adjustSize();
              show();
          } else {
              rgbd_label.clear();
              depth_label.clear();
          }
      }
    };

  private:
    QLabel label;
    std::shared_ptr<DSR::DSRGraph> graph;
    std::unique_ptr<CameraAPI> cam;
    DSR::IDType node_id;
};      

#endif // GRAPHNODERGBDWIDGET_H
