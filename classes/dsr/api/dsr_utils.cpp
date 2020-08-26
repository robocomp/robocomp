////////////////////////////////////////////////////////////////////////
//// UTILITIES FOR DSRGraph
////////////////////////////////////////////////////////////////////////

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <fstream>
#include "dsr_utils.h"
#include "dsr_api.h"

using namespace DSR;

Utilities::Utilities(DSR::DSRGraph *G_)
{ G = G_; }


QJsonDocument Utilities::file_to_QJsonDocument(const std::string &json_file_path)
{
	// Open file and make initial checks
	QFile file;
	file.setFileName(QString::fromStdString(json_file_path));
	if (not file.open(QIODevice::ReadOnly | QIODevice::Text))
		throw std::runtime_error("File " + json_file_path + " not found. Cannot continue.");

	QString val = file.readAll();
	file.close();

	QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
	return doc;
}

void Utilities::read_from_json_file(const std::string &json_file_path,  const std::function<std::optional<int>(const Node&)>& insert_node)
{
    qDebug() << __FUNCTION__ << " Reading json file: " << QString::fromStdString(json_file_path);

	QJsonDocument doc = Utilities::file_to_QJsonDocument(json_file_path);
    QJsonObject jObject = doc.object();

    QJsonObject dsrobject = jObject.value("DSRModel").toObject();
	QJsonObject symbolMap = dsrobject.value("symbols").toObject();
    QJsonArray linksArray;
    // Read symbols (just symbols, then links in other loop)
    for (const QString &key : symbolMap.keys())
    {
        QJsonObject sym_obj = symbolMap[key].toObject();
        int id = sym_obj.value("id").toInt();
        std::string type = sym_obj.value("type").toString().toStdString();
        std::string name = sym_obj.value("name").toString().toStdString();

        if (id == -1)
        {
            std::cout << __FILE__ << " " << __FUNCTION__ << " Invalid ID Node: " << std::to_string(id);
            continue;
        }
        qDebug() << __FILE__ << " " << __FUNCTION__ << ", Node: " << id << " " <<  QString::fromStdString(type);
        Node n;
        n.type(type);
        n.id(id);
        n.agent_id(G->get_agent_id());
        n.name(name);
        //G->name_map[name] = id;
        //G->id_map[id] = name;

            //std::map<string, Attrib> attrs;
/*
 * THIS INFORMATION MUST BE ON JSON FILE 
        G->insert_or_assign_attrib_by_name(n, "level", std::int32_t(0));
        G->insert_or_assign_attrib_by_name(n, "parent", std::int32_t(0));
        

 *        std::string full_name = type + " [" + std::to_string(id) + "]";
        G->insert_or_assign_attrib_by_name(n, "name", full_name);
        // G->add_attrib(attrs, "name",full_name);

        // color selection
        std::string color = "coral";
        if(type == "world") color = "SeaGreen";
        else if(type == "transform") color = "SteelBlue";
        else if(type == "plane") color = "Khaki";
        else if(type == "differentialrobot") color = "GoldenRod";
        else if(type == "laser") color = "GreenYellow";
        else if(type == "mesh") color = "LightBlue";
        else if(type == "imu") color = "LightSalmon";

        //G->add_attrib(attrs, "color", color);
        G->insert_or_assign_attrib_by_name(n, "color", color);
*/
            // node atributes
            QVariantMap attributesMap = sym_obj.value("attribute").toObject().toVariantMap();
            for (QVariantMap::const_iterator iter = attributesMap.begin(); iter != attributesMap.end(); ++iter) {
                std::string attr_key = iter.key().toStdString();
                QVariant attr_value = iter.value().toMap()["value"];
                int attr_type = iter.value().toMap()["type"].toInt();

                switch (attr_type) {
                    case 0: {
                        G->runtime_checked_add_attrib_local(n,  attr_key, attr_value.toString().toStdString());
                        break;
                    }
                    case 1: {
                        G->runtime_checked_add_attrib_local(n, attr_key,  attr_value.toInt());
                        break;
                    }
                    case 2: {
                        G->runtime_checked_add_attrib_local(n,  attr_key,  attr_value.toString().replace(",", ".").toFloat());
                        break;
                    }
                    case 3: {
                        std::vector<float> v;
                        for (const QVariant &value : attr_value.toList())
                            v.push_back(value.toFloat());
                        G->runtime_checked_add_attrib_local(n,  attr_key,  v);
                        break;
                    }
                    case 4: {
                        G->runtime_checked_add_attrib_local(n, attr_key,   attr_value.toBool());
                        break;
                    }
                    case 5: {
                        std::vector<uint8_t> v;
                        for (const QVariant &value : attr_value.toList())
                            v.push_back(static_cast<uint8_t>(value.toUInt()));
                        G->runtime_checked_add_attrib_local(n,  attr_key,  v);
                        break;
                    }
                    case 6: {
                        G->runtime_checked_add_attrib_local(n,  attr_key,  static_cast<std::uint32_t>(attr_value.toUInt()));
                        break;
                    }
                    default:
                        G->runtime_checked_add_attrib_local(n,  attr_key,  attr_value.toString().toStdString());
                }
            }
            //n.attrs(attrs);
            insert_node(n);
        // get links
        QJsonArray nodeLinksArray = sym_obj.value("links").toArray();
        std::copy(nodeLinksArray.begin(), nodeLinksArray.end(), std::back_inserter(linksArray));
    }
    // Read links
    for (const auto &linkValue : linksArray)
    {
            QJsonObject link_obj = linkValue.toObject();
            int srcn = link_obj.value("src").toInt();
            int dstn = link_obj.value("dst").toInt();
            std::string edgeName = link_obj.value("label").toString().toStdString();
            std::map<string, CRDTAttribute> attrs;

            Edge edge(dstn, srcn, edgeName, {}, G->get_agent_id());
//            Edge edge;
//            edge.from(srcn);
//            edge.to(dstn);
//            edge.type(edgeName);

            // link atributes
            QVariantMap linkAttributesMap = link_obj.value("linkAttribute").toObject().toVariantMap();
            for (QVariantMap::const_iterator iter = linkAttributesMap.begin(); iter != linkAttributesMap.end(); ++iter)
            {
                std::string attr_key = iter.key().toStdString();
                QVariant attr_value = iter.value().toMap()["value"];
                int attr_type = iter.value().toMap()["type"].toInt();

                CRDTAttribute av;
                av.type(attr_type);
                CRDTValue value;
                CRDTAttribute at;


                switch (attr_type) {
                    case 0: {
                        G->runtime_checked_add_attrib_local(edge, attr_key,  attr_value.toString().toStdString());
                        break;
                    }
                    case 1: {
                        G->runtime_checked_add_attrib_local(edge, attr_key,  attr_value.toInt());
                        break;
                    }
                    case 2: {
                        G->runtime_checked_add_attrib_local(edge,  attr_key,  attr_value.toString().replace(",", ".").toFloat());
                        break;
                    }
                    case 3: {
                        std::vector<float> v;
                        for (const QVariant &val : attr_value.toList())
                            v.push_back(val.toFloat());
                        G->runtime_checked_add_attrib_local(edge, attr_key,   v);
                        break;
                    }
                    case 4: {
                        G->runtime_checked_add_attrib_local(edge,  attr_key,  attr_value.toBool());
                        break;
                    }
                    case 5: {
                        std::vector<uint8_t> v;
                        for (const QVariant &val : attr_value.toList())
                            v.push_back(static_cast<uint8_t>(val.toUInt()));
                        G->runtime_checked_add_attrib_local(edge, attr_key,   v);
                        break;
                    }
                    case 6: {
                        G->runtime_checked_add_attrib_local(edge,  attr_key,   static_cast<std::uint32_t>(attr_value.toUInt()));
                        break;
                    }
                    default:
                        G->runtime_checked_add_attrib_local(edge,   attr_key, attr_value.toString().toStdString());
                }
            }
            qDebug() << __FILE__ << " " << __FUNCTION__ << "Edge from " << srcn << " to " << dstn << " label "  << QString::fromStdString(edgeName);
            //edge.attrs(attrs);
            G->insert_or_assign_edge(edge);

        } //foreach(links)
}

QJsonObject Utilities::Edge_to_QObject(const Edge& edge)
{
    QJsonObject link;
    link["src"] = static_cast<int>(edge.from());
    link["dst"] = static_cast<int>(edge.to());
    link["label"] = QString::fromStdString(edge.type());

    QJsonObject lattrsObject;
    for (auto &[key, value]: edge.attrs()) {
        QJsonObject content;
        QJsonValue val;
        switch (value.value().index()) {
            case 0:
                val = QString::fromStdString(get<std::string>(value.value()));
                break;
            case 1:
                val = get<std::int32_t>(value.value());
                break;
            case 2:
                val = std::round(static_cast<double>(get<float>(value.value())) * 1000000) / 1000000;
                break;
            case 4:
                val = get<bool>(value.value());
                break;
            case 3: {
                QJsonArray array;
                for (const float &value : get<std::vector<float>>(value.value()))
                    array.push_back(value);
                val = array;
                break;
            }
            case 5: {
                QJsonArray array;
                for (const uint8_t &value : get<std::vector<uint8_t>>(value.value()))
                    array.push_back(static_cast<qint64>(value));
                val = array;
                break;
            }
            case 6:
                val = static_cast<std::int32_t>(get<std::uint32_t>(value.value()));
                break;
        }
        content["type"] = static_cast<qint64>(value.value().index());
        content["value"] = val;
        lattrsObject[QString::fromStdString(key)] = content;
    }
    link["linkAttribute"] = lattrsObject;
    return link;
}

QJsonObject Utilities::Node_to_QObject(const Node& node)
{
    QJsonObject symbol;
    symbol["id"] = static_cast<qint64>(node.id());
    symbol["type"] = QString::fromStdString(node.type());
    symbol["name"] = QString::fromStdString(node.name());
    // symbol attribute
    QJsonObject attrsObject;
    for (const auto &[key, value]: node.attrs())
    {
        QJsonObject content;
        QJsonValue val;
        switch (value.value().index()) {
            case 0:
                val = QString::fromStdString(get<std::string>(value.value()));
                break;
            case 1:
                val = get<std::int32_t>(value.value());
                break;
            case 2:
                val = std::round(static_cast<double>(get<float>(value.value())) * 1000000) / 1000000;
                break;
            case 4:
                val = get<bool>(value.value());
                break;
            case 3: {
                QJsonArray array;
                for (const float &value : get<std::vector<float>>(value.value()))
                    array.push_back(value);
                val = array;
                break;
            }
            case 5: {
                QJsonArray array;
                for (const uint8_t &value : get<std::vector<uint8_t>>(value.value()))
                    array.push_back(static_cast<qint64>(value));
                val = array;
                break;
            }
            case 6:
                val = static_cast<std::int32_t>(get<std::uint32_t>(value.value()));
                break;

        }
        content["type"] = static_cast<qint64>(value.value().index());
        content["value"] = val;
        attrsObject[QString::fromStdString(key)] = content;
    }
    symbol["attribute"] = attrsObject;
    //link
    QJsonArray nodeLinksArray;
    for (const auto &[key, value]: node.fano()) {
        QJsonObject link = Edge_to_QObject(value);
        nodeLinksArray.push_back(link);
    }
    symbol["links"] = nodeLinksArray;
    return symbol;
}


QJsonDocument Utilities::DSRGraph_to_QJsonDocument(DSR::DSRGraph *G_)
{
    std::setlocale(LC_NUMERIC, "en_US.UTF-8");
    //create json object
    QJsonObject dsrObject;
    QJsonArray linksArray;
    QJsonObject symbolsMap;
    for (const auto& kv : G_->getCopy()) {
        Node node = kv.second;
        // symbol data
        QJsonObject symbol = Node_to_QObject(node);
        symbolsMap[QString::number(node.id())] = symbol;
    }
    dsrObject["symbols"] = symbolsMap;

    QJsonObject jsonObject;
    jsonObject["DSRModel"] = dsrObject;
    //write to file
    QJsonDocument jsonDoc(jsonObject);
    return jsonDoc;
}

void Utilities::write_to_json_file(const std::string &json_file_path)
{
	QJsonDocument jsonDoc = DSRGraph_to_QJsonDocument(G);
    QFile jsonFile(QString::fromStdString(json_file_path));
    jsonFile.open(QFile::WriteOnly | QFile::Text);
    jsonFile.write(/*qCompress(*/jsonDoc.toJson())/*)*/;
    jsonFile.close();
    auto now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    qDebug() << __FILE__ << " " << __FUNCTION__ << "File: " << QString::fromStdString(json_file_path)<< " written to disk at " << now_c;
}

void Utilities::print()
{
    for (const auto &[_,v] : G->getCopy())
    {
        Node node = v;
        std::cout << "Node: " << node.id() << std::endl;
        std::cout << "  Type:" << node.type() << std::endl;
        std::cout << "  Name:" << node.name() << std::endl;
        std::cout << "  Agent_id:" << node.agent_id() << std::endl;
        for (auto &[key, val] : node.attrs())
            std::cout << "      [ " << key << ", " << DSR::TYPENAMES_UNION[val.value().index()] << ", " << val << " ]"<< std::endl;
        for (auto &[key, val] : node.fano())
        {
            std::cout << "          Edge-type->" << val.type() << " from:" << val.from() << " to:"<< val.to() << std::endl;
            for (auto[k, v] : val.attrs())
                std::cout << "              Key->" << k << " Type->" << DSR::TYPENAMES_UNION[v.value().index()] << " Value->"<< v << std::endl;
        }
    }
    std::cout << "------------------------------------------------" << std::endl;
}

void Utilities::print_edge(const Edge &edge) {
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Edge-type->" << edge.type() << " from->" << edge.from() << " to->" << edge.to() << std::endl;
    for (auto[k, v] : edge.attrs())
        std::cout << "              Key->" << k << " Type->" << DSR::TYPENAMES_UNION[v.value().index()] << " Value->" << v<< std::endl;
    std::cout << "------------------------------------" << std::endl;
}

void Utilities::print_node(const Node &node) {
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Node-> " << node.id() << std::endl;
    std::cout << "  Type->" << node.type() << std::endl;
    std::cout << "  Name->" << node.name() << std::endl;
    std::cout << "  Agent_id->" << node.agent_id() << std::endl;
    for (auto [key, val] : node.attrs())
        std::cout << "      Key->" << key << " Type->" << DSR::TYPENAMES_UNION[val.value().index()] << " Value->" << val << std::endl;
    for (auto [key, val] : node.fano())
    {
        std::cout << "          Edge-type->" << val.type() << " from->" << val.from() << " to->" << val.to() << std::endl;
        for (auto [k, v] : val.attrs())
            std::cout << "              Key->" << k << " Type->" << DSR::TYPENAMES_UNION[v.value().index()] << " Value->" << v << std::endl;
    }
}

void Utilities::print_node(const std::int32_t id)
{
    auto node = G->get_node(id);
    if(node.has_value())
        print_node(node.value());
}

void Utilities::print_RT(const std::int32_t id)
{
    std::cout << "-------------- Printing RT tree ------------------" << std::endl;
    auto node = G->get_node(id);
    if(node.has_value())
    {
        print_node(node.value());
        print_RT(node.value());
    }
    else
        throw std::runtime_error("Print_RT. Unable to traverse the tree at node: " + std::to_string(id));
    std::cout << "-------------- End printing RT tree ------------------" << std::endl;
}

void Utilities::print_RT(const Node& node)
{
    for(auto &edge: G->get_node_edges_by_type(node, "RT"))
	{
        auto child = G->get_node(edge.to());
        if(child.has_value())
        {
            print_node(child.value());
            print_RT(child.value());
        }
        else
            throw std::runtime_error("Unable to traverse the tree at node: " + std::to_string(edge.to()));
	}
}

