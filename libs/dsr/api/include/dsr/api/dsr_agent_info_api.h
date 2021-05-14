//
// Created by juancarlos on 6/5/21.
//

#ifndef DSR_AGENTINFO_API_H
#define DSR_AGENTINFO_API_H


#include <dsr/core/types/type_checking/dsr_node_type.h>
#include <dsr/core/types/type_checking/dsr_attr_name.h>
#include <dsr/api/dsr_api.h>
#include <sys/syscall.h>
#include <sys/types.h>

namespace DSR {

    class DSRGraph;

    class AgentInfoAPI {

    public:
        explicit AgentInfoAPI(DSR::DSRGraph *g, uint32_t period_ = 1500) : G(g), period(period_)
        {
            timer.callOnTimeout([this](){ create_or_update_agent(); });
            timer.start(period);
        }

        void stopTimer();
        void setPriod(uint32_t period_);

    private:

        static std::string exec(const char* cmd);
        void create_or_update_agent();

        DSRGraph *G;
        uint64_t timestamp_start{0};
        uint32_t period;
        QTimer timer;
    };

}

#endif //DSR_AGENTINFO_API_H
