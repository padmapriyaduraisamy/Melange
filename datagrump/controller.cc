#include <iostream>
#include <math.h>
#include <algorithm>

#include "controller.hh"
#include "timestamp.hh"

#define ALPHA (1.0/8.0)
#define BETA (1.0/4.0)
#define MIN_RTT 50   /* ms */
#define MAX_RTT 5000 /* ms */

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug)
  : debug_( debug ), 
    cwnd (1.0),
    prev_ack_time (0), 
    trp (0), 
    trc (0), 
    alpha_e (3.0), 
    beta_e (1.0),
    SRTT (1000),
    RTTVAR (500),
    RTO (1000),
    first_measurement (true), 
    ss (true),
    rtt_hist (5,0) {}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << cwnd << endl;
  }
  return floor(cwnd);
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
  uint64_t time_interval = (timestamp_ack_received - prev_ack_time);
  if (time_interval == 0)
    time_interval = 1;
  trc = PKT_SIZE / time_interval;
  double dtr = trc - trp;
  double incr;
  if (ss)
    incr = 1;
  else if (dtr == 0 || dtr > alpha_e) 
    incr = 2/cwnd;
  else if (dtr > beta_e)
    incr = 1/cwnd;
  else 
    incr = 0;

//cerr<<"trp "<<trp<<" trc "<<trc<<" timeInt "<<time_interval<<" incr "<<incr<<" cwnd "<<cwnd<<endl;

  cwnd = cwnd + incr;
  trp = trc;
  prev_ack_time = timestamp_ack_received;

  rtt_hist.erase(rtt_hist.begin ());
  uint64_t rtt_cur = timestamp_ack_received - send_timestamp_acked;
  rtt_hist.push_back (rtt_cur);
  if (rtt_cur > 100)
    window_decrease (); 
  rtt_estimation (rtt_cur);
  
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

void Controller::window_decrease( void )
{
//cerr<<"Window Decrease" <<endl;
  uint64_t sum = 0;
  for (uint64_t &t : rtt_hist)
    sum += t;
  //double rtt_avg = sum / N;
  //double drtt = rtt_hist [N-1] - rtt_avg;
  double drtt = rtt_hist [N-1] - SRTT;
  double incf =  (cwnd/ rtt_hist [N-1]) * drtt;
  if (incf < 1)
    incf = 1;
  cwnd = max((cwnd - incf), 2.0);
  ss = false;
}


void Controller::rtt_estimation (uint64_t rtt_cur) 
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

void Controller:: timeout_event (void)
{
//cerr<<"Timeout Event" <<endl;
  cwnd = 2.0;
  ss = true;
}
