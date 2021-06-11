//
// Created by juancarlos on 6/5/21.
//

#include <dsr/api/dsr_agent_info_api.h>
#include <unistd.h>

namespace DSR {


    void AgentInfoAPI::stopTimer()
    {
        timer.stop_timer();
    }

    bool AgentInfoAPI::isRunning()
    {
        return timer.is_running();
    }
    /*void AgentInfoAPI::setPriod(uint32_t period_)
    {
        period = period_;
        timer.setInterval(static_cast<int32_t>(period_));
    }*/

    std::string AgentInfoAPI::exec(const char* cmd)
    {
        std::array<char, 128> buffer{};
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        return result;
    }

    void AgentInfoAPI::create_or_update_agent()
    {
        auto str = "Participant_" + std::to_string(G->get_agent_id()) + " ( " + G->get_agent_name() + " )";
        pid_t pid = getpid();

        int nprocs = -1;
        int64_t memory_kb = -1;
        float cpu = -1.0;

        const char* w = " \t\n\r\f\v";
        const auto trim = [&](std::string & str) {
            str.erase(str.find_last_not_of(w) + 1);
            str.erase(0, str.find_first_not_of(w));
        };

        try {
            std::string memory_kb_and_cpu = exec(
                    ("top -p " + std::to_string(pid) +" -b -c -n1 | grep  "+ std::to_string(pid) + " | awk '{print $6 \";\" $9}'").c_str());

            trim(memory_kb_and_cpu);
            const char delimiter = ';';
            const uint32_t pos = memory_kb_and_cpu.find(delimiter);
            memory_kb = std::stoi(memory_kb_and_cpu.substr(0, pos));
            memory_kb_and_cpu.erase(0, pos+1);
            std::replace(memory_kb_and_cpu.begin(), memory_kb_and_cpu.end(), ',', '.');
            cpu = std::stof(memory_kb_and_cpu);

        } catch (const std::runtime_error &e)
        {
            std::cerr << "Error in popen. " << __FILE__ << ":" << __LINE__  << std::endl;
        } catch (...)
        {
            std::cerr << "Error in stoi or stof. " << __FILE__ << ":" << __LINE__  << std::endl;
        }

        try {
            std::string number_of_process = exec(("echo $((`pstree -p "+ std::to_string(pid) +" | wc -l` + 1))").c_str());
            trim(number_of_process);
            nprocs = std::stoi(number_of_process);

        } catch (const std::runtime_error &e)
        {
            std::cerr << "Error in popen. " << __FILE__ << ":" << __LINE__  << std::endl;
        } catch (...)
        {
            std::cerr << "Error in stoi or stof. " << __FILE__ << ":" << __LINE__  << std::endl;
        }


        if (auto node = G->get_node(str))
        {

            uint64_t times = get_unix_timestamp();
            auto &node_ref = node.value();
            G->add_or_modify_attrib_local<timestamp_agent_att>(node_ref, times);
            G->add_or_modify_attrib_local<timestamp_alivetime_att>(node_ref,
               static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::nanoseconds(times - timestamp_start)).count()));
            if (cpu >= 0.0)
            {
                G->add_or_modify_attrib_local<cpu_usage_att>(node_ref, cpu);
            }
            //memory usage
            if (memory_kb >= 0)
            {
                G->add_or_modify_attrib_local<memory_usage_att>(node_ref, static_cast<uint32_t>(memory_kb));
            }
            //num_threads
            if (nprocs >= 0)
            {
                G->add_or_modify_attrib_local<num_procs_att>(node_ref, static_cast<uint32_t>(nprocs));
            }

            G->update_node(node_ref);
        } else
        {
            // edge
            std::uint64_t parent_id;
            if(auto parent = G->get_node("mind"); parent.has_value())
                parent_id = parent.value().id();
            else parent_id = G->get_node_root().value().id();

            // node
            DSR::Node new_node = Node::create<agent_node_type> ({}, {}, str);
            timestamp_start = get_unix_timestamp();
            G->add_or_modify_attrib_local<timestamp_agent_att>(new_node, timestamp_start);
            G->add_or_modify_attrib_local<timestamp_creation_att>(new_node, timestamp_start);
            G->add_or_modify_attrib_local<timestamp_alivetime_att>(new_node, static_cast<uint64_t>(0));
            G->add_or_modify_attrib_local<agent_id_att>(new_node, static_cast<uint32_t>(G->get_agent_id()));
            G->add_or_modify_attrib_local<agent_name_att>(new_node, str);
            G->add_or_modify_attrib_local<parent_att>(new_node, parent_id);
            G->add_or_modify_attrib_local<agent_description_att>(new_node, std::string{"TODO"});
            //CPU usage
            if (cpu >= 0.0)
            {
                G->add_or_modify_attrib_local<cpu_usage_att>(new_node, cpu);
            }
            //memory usage
            if (memory_kb >= 0)
            {
                G->add_or_modify_attrib_local<memory_usage_att>(new_node, static_cast<uint32_t>(memory_kb));
            }
            //num_threads
            if (nprocs >= 0)
            {
                G->add_or_modify_attrib_local<num_procs_att>(new_node, static_cast<uint32_t>(nprocs));
            }
            G->insert_node(new_node);
            DSR::Edge edge = DSR::Edge::create<has_edge_type>(parent_id, new_node.id());
            G->insert_or_assign_edge(edge);
        }
    }
}

