#ifndef UGV_STATE_H
#define UGV_STATE_H
#include <functional>
#include <vector>
#include <iostream>

#define PRINT_STATE_DEBUG_INFO
//Resize the proposition vector container
#define INIT_PROPOSITION_VECTOR() \
{ \
  m_props.resize(MAX_SIZE); \
  }

//Add a proposition (enum, evaluation_function) pair into the container
#define ADD_PROPOSITION(PPS, eva_function) \
{ \
  Proposition p; \
  p.eva_func = std::bind(eva_function, this, std::placeholders::_1); \
  m_props[PPS] = p; \
  } \

class Pipeline;

struct Proposition
{
  bool checked; // Whether the proposition has been checked
  bool state; // State of the proposition
  std::function<bool(Pipeline*)> eva_func; // Function pointer to evaluation function
  void evaulate(Pipeline* pipe) // Call evaluation function
  {
    state = eva_func(pipe);
    checked = true;
  }

  void manual_set(bool val) // Manual set state
  {
    state = val;
    checked = true;
  }
};

struct TokenLog
{
  int issue_cycle = -1; // The execution cycle when the last token is issued
  int issue_num = 0; // How many token have been issued in the "issue_cycle"
};

class State
{
public:
  virtual void on_enter(Pipeline* pipe) = 0; // The stuff a state will execute during entrance
  virtual void on_exit(Pipeline* pipe) = 0; // The stuff a state will execute during exit
  virtual State& toggle(Pipeline* pipe) = 0; // Determine the next state
  virtual void check_props(Pipeline* pipe) // Check propositions
  {
    for (Proposition &p : m_props)
      p.evaulate(pipe);
  }

  void check_prop(int i, Pipeline* pipe)
  {
    m_props[i].evaulate(pipe);
  }

  void set_prop(int i, bool val)
  {
    m_props[i].manual_set(val);
  }

  void prepare_props_for_checking() // Set all propositions as unchecked
  {
    for (Proposition &p : m_props)
      p.checked = false;
  }

  void exam_props_all_checked() // Check whether all propositions are checked
  {
    for (size_t i=0; i<m_props.size(); i++)
    {
      if (!m_props[i].checked)
      {
        std::cerr<<"In state: "<<typeid(*this).name()<<", proposition "<<i<<" is not checked!"<<std::endl;
        exit(-1);
      }
    }
  }
  virtual ~State() {}

protected:
  std::vector<Proposition> m_props; // Proposition vector
  TokenLog m_tl; // Log for token issuing
  bool is_true(size_t p) // Check the state of a proposition
  {
    return m_props[p].state;
  }

  void issue_token(Pipeline* pipe); // Issue out a token to the pipeline so that the task loop run again

};

#endif // UGV_STATE_H
