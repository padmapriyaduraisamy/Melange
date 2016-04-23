#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>

#define PKT_SIZE 1500*8

/* Congestion controller interface */
class Controller
{
private:
  bool debug_; /* Enables debugging output */
  double cwnd;    /* Variable window size */
  uint64_t prev_ack;
  double trp;
  double trc;
  double alpha;
  double beta;
  /* Add member variables here */

public:
  /* Public interface for the congestion controller */
  /* You can change these if you prefer, but will need to change
     the call site as well (in sender.cc) */

  /* Default constructor */
  Controller( const bool debug, double cwndow = 10, uint64_t prev_ack = 0, double trp =0, double trc = 0, double alpha = 3, double beta = 1);

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
  void timeout_event (void);
};

#endif
