#include <iostream>
#include <math.h>

#include "controller.hh"
#include "timestamp.hh"

#define ALPHA 1.0/8.0
#define BETA  1.0/4.0

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ),
    q_occupancy (0),
    rtt (-10),
    rtt_var (0),
    rto (1000),
    link_rate_prev (0),
    wnd (2),
    q_occup_map ()
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << floor (wnd) << endl;
  }

  return floor (wnd); 
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
  double packet_delay = timestamp_ack_received - send_timestamp_acked;
  double link_rate_cur = q_occup_map [sequence_number_acked] / packet_delay;
  
  double d_link_rate = link_rate_cur - link_rate_prev;

  double d_wnd = d_link_rate * rtt_estimate (packet_delay) / 2;
  wnd += d_wnd;

  //cout << "dR " << d_link_rate << " dwnd " << d_wnd << " wnd " << wnd << endl;

  link_rate_prev = link_rate_cur;

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return rto; /* timeout of one second */
}

/* RTT estimate.*/
double Controller::rtt_estimate (double delay)
{
  if (rtt < 0)
    {
      rtt = delay;
      rtt_var = delay / 2.0;
    }
  else
    {
      rtt = ALPHA * delay + (1-ALPHA) * rtt;
      rtt_var = (1-BETA) * rtt_var + BETA * fabs (rtt - delay);
    }

  rto = rtt + 4 * rtt_var;
  return rtt;
}













