#include <iostream>
#include <math.h>

#include "controller.hh"
#include "timestamp.hh"

#define MIN_RTT 50       /* Min RTT */
#define MAX_RTT 5000     /* Max RTT */
#define ALPHA   1.0/8.0  /* Alpha for RTT estimate */
#define BETA    1.0/4.0  /* Beta for RTT variance weighting */ 

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), 
    cwnd (2),
    link_rate_prev (0), 
    link_rate_ewma (10), 
    first_measurement (true),
    SRTT (0),
    RTTVAR (0),
    RTO (1000),
    q_occupancy (0),
    q_occup_map (),
    slow_start (true)
{
  if ( debug_ ) {
    cerr << "Initial window is " << cwnd << endl;
  }
}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  if ( debug_ ) {
    cout << "At time " << timestamp_ms()
	 << " window size is " << cwnd << endl;
  }
  return floor (cwnd);
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  q_occupancy++;
  q_occup_map [sequence_number] = q_occupancy;

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
  q_occupancy--;
  double delay = timestamp_ack_received - send_timestamp_acked;
  double link_rate_cur = q_occup_map [sequence_number_acked] / delay;
  /* Remove packet */
  q_occup_map.erase(sequence_number_acked);

  double dtr = 1000 * (link_rate_cur - link_rate_prev);
  link_rate_ewma = ALPHA * dtr + (1-ALPHA) * link_rate_ewma;
  double incr = 0;

  /* Update window */
  if (slow_start)
    incr = 1;
  else if (dtr > link_rate_ewma*0.6) 
    incr = 3/cwnd;
  else if (dtr > link_rate_ewma*0.3)
    incr = 2/cwnd;
  else if (dtr > 0)
    incr = 1/cwnd;
  
  cwnd = cwnd + incr;
  
  link_rate_prev = link_rate_cur;

  rtt_estimate (delay);
  
  /* Adjust window */
  if (delay > 120)
    window_decrease ();
  
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
  return RTO;
}

/* Update window on delay trigger */
void Controller::window_decrease(void)
{
  if (slow_start)
    slow_start = false;
  cwnd *= 0.85;
}

/* Estimate RTT */
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

