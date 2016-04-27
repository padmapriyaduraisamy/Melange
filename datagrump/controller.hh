#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <pthread.h>
#include <mutex>
#include <vector>

/* Congestion controller interface */

class Controller
{
private:
  bool debug_; /* Enables debugging output */
  double Dmax_cur, Dmax_prev;
  double dDelay, epoch_max_delay;
  bool slow_start, loss_recovery, new_epoch;
  std::vector<double> w_delay;
  double Dest, Dmin, rto;
  double wnd,s_wnd;
  std::mutex lock_delay_vars;
  std::vector<uint64_t> sent_list;

public:
  /* Public interface for the congestion controller */
  /* You can change these if you prefer, but will need to change
     the call site as well (in sender.cc) */

  /* Default constructor */
  Controller( const bool debug );

  /* Get current window size, in datagrams */
  unsigned int window_size( void );

  /* A datagram was sent */
  void datagram_was_sent( const uint64_t sequence_number,
			  const uint64_t send_timestamp );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms( void );
  pthread_t wind_estimation_tid;                                                  
  static void* wind_estimation_thread (void* arg);  
};

#endif
