#ifndef INITIATOR_H_
#define INITIATOR_H_
#include <systemc>
using namespace sc_core;

#include "tlm"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/tlm_quantumkeeper.h"         //quantumkeeper

class Initiator : public sc_module {
public:
  tlm_utils::simple_initiator_socket<Initiator> i_skt;

  SC_HAS_PROCESS(Initiator);
  Initiator(sc_module_name n);
// initiate read or write transactions
  int read_from_socket(unsigned long int addr, unsigned char mask[],
                       unsigned char rdata[], int dataLen);

  int write_to_socket(unsigned long int addr, unsigned char mask[],
                      unsigned char wdata[], int dataLen);

  void do_trans(tlm::tlm_generic_payload &trans);
  tlm::tlm_generic_payload trans;
  tlm_utils::tlm_quantumkeeper m_qk;     // m_qk
  sc_time delay = sc_time(10, SC_NS);    // sc_time
};
#endif
