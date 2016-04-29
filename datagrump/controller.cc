#include <iostream>
#include <math.h>

#include "controller.hh"
#include "timestamp.hh"

#define MIN_RTT 50
#define MAX_RTT 5000
#define ALPHA   1.0/8.0
#define BETA    1.0/4.0

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), 
    cwnd (2),
    prev_ack (0), 
    trp (0), 
    trc (0), 
    alpha (3), 
    beta (1),
    first_measurement (true),
    SRTT (0),
    RTTVAR (0),
    RTO (1000)
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
  return floor (cwnd);
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

  double delay = timestamp_ack_received - send_timestamp_acked;
  rtt_estimate (delay);
  if (delay > 90)
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
  return RTO; /* timeout of one second */
}

void Controller::timeout_event( void)
{
  cwnd = cwnd * 0.85;
}

void Controller::rtt_estimate (double rtt_cur)
{
  if (first_measurement) 
    {
      SRTT = rtt_cur;
      RTTVAR = rtt_cur / 2;
      first_measurement = false;
    }
  else 
    {
      RTTVAR = (1 - BETA) * RTTVAR + (BETA * fabs(SRTT - rtt_cur));
      SRTT = (1 - ALPHA)*SRTT + (ALPHA * rtt_cur);
    }
    RTO = SRTT + 4* RTTVAR; 
    if (RTO < MIN_RTT)
      RTO = MIN_RTT;
    else if (RTO > MAX_RTT)
      RTO = MAX_RTT; 
}
