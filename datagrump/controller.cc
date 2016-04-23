#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug, double cwndow, uint64_t prv_ack, double prev, double cur, double a, double b)
  : debug_( debug ), cwnd (cwndow),prev_ack (prv_ack), trp (prev), trc (cur), alpha (a), beta (b) 
{
  if ( debug_ ) {
    cerr << "Initial window is " << cwnd << endl;
  }
}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
//  /* Default: fixed window size of 100 outstanding datagrams */
//  unsigned int the_window_size = 50;

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << cwnd << endl;
  }

//  return the_window_size;
  return (int)cwnd;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << endl;
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  uint64_t time_interval = (timestamp_ack_received - prev_ack);
  if (time_interval == 0)
    time_interval = 1;
  trc = PKT_SIZE / time_interval;
  double dtr = trc - trp;
  double incr;
  if (dtr == 0 || dtr > alpha) 
    incr = 2/cwnd;
  else if (dtr > beta)
    incr = 1/cwnd;
  else 
    incr = 0;
  cwnd = cwnd + incr;
  trp = trc;
  prev_ack = timestamp_ack_received;

  if (timestamp_ack_received - send_timestamp_acked > 90)
    timeout_event (); 
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << ", window is " << cwnd 
         << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 100; /* timeout of one second */
}

void Controller::timeout_event( void)
{
  cwnd = cwnd * 0.85;
}
