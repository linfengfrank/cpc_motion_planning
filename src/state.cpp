#include <cpc_motion_planning/state.h>
#include <cpc_motion_planning/pipeline.h>

void State::issue_token(Pipeline* pipe)
{
    // Update the TokenLog
    if (m_tl.issue_cycle != pipe->get_cycle())
    {
        m_tl.issue_cycle = pipe->get_cycle();
        m_tl.issue_num = 1; //Issue one token straight away
    }
    else
    {
        m_tl.issue_num++;
    }

    // Check how many token we have issued in this cycle
    if (m_tl.issue_num > 1)
    {
        std::cerr<<"In state: "<<typeid(*this).name()<<", circle detected due to token issueing!"<<std::endl;
        exit(-1);
    }

    // Allow piple line to add a token
    pipe->add_token();
}
